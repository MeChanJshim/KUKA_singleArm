#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <lbr_fri_idl/msg/lbr_joint_position_command.hpp>

#include <QApplication>
#include <QWidget>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QDial>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QTimer>

#include <mutex>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

using lbr_fri_idl::msg::LBRJointPositionCommand;

class KukaCommanderNode : public rclcpp::Node {
public:
  KukaCommanderNode() : Node("single_joint_commander")
  {
    this->declare_parameter<std::string>("robot_name", "kuka");
    this->declare_parameter<int>("numOfJoints", 7);
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    robot_name_   = this->get_parameter("robot_name").as_string();
    num_joints_   = this->get_parameter("numOfJoints").as_int();
    publish_rate_ = this->get_parameter("publish_rate_hz").as_double();

    // KUKA 명령 메시지의 고정 길이 확인 (보통 7)
    {
      LBRJointPositionCommand tmp;
      const size_t N = tmp.joint_position.size();
      if (num_joints_ != static_cast<int>(N)) {
        RCLCPP_WARN(this->get_logger(),
          "numOfJoints(%d) != LBRJointPositionCommand::joint_position.size()(%zu). "
          "퍼블리시 길이를 %zu로 강제합니다.",
          num_joints_, N, N);
        num_joints_ = static_cast<int>(N);
      }
    }

    current_angles_.assign(num_joints_, 0.0);
    target_angles_.assign(num_joints_, 0.0);
    joint_mapping_.assign(num_joints_, -1);

    // --- Subscriber: JointState
    std::string jointState_TP = robot_name_ + "/joint_states";
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    JointState_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      jointState_TP, qos, std::bind(&KukaCommanderNode::JointStateCB, this, std::placeholders::_1));

    // --- Publisher: KUKA Joint Position Command
    cmd_topic_ = robot_name_ + "/command/joint_position";
    kuka_pub_ = this->create_publisher<LBRJointPositionCommand>(cmd_topic_, 1);

    RCLCPP_INFO(this->get_logger(), "Subscribing: %s", jointState_TP.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", cmd_topic_.c_str());
  }

  // JointState 콜백 (사용자가 준 패턴 반영)
  void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!current_angles_received_) current_angles_received_ = true;

    if (static_cast<int>(msg->position.size()) < num_joints_ ||
        msg->name.size() != msg->position.size()) {
      RCLCPP_WARN(this->get_logger(), "Invalid joint state message");
      return;
    }

    if (!mapping_initialized_) {
      initializeJointMapping(msg);
    }

    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < num_joints_; ++i) {
      int idx = joint_mapping_[i];
      if (idx >= 0 && idx < static_cast<int>(msg->position.size())) {
        current_angles_[i] = msg->position[idx];
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Invalid mapping for joint %d", i);
      }
    }

    // 첫 업데이트 시 target을 current로 미러링(점프 방지)
    if (!target_initialized_) {
      target_angles_ = current_angles_;
      target_initialized_ = true;
    }

    last_js_names_ = msg->name; // UI 표시용
  }

  // UI에서 선택된 조인트의 목표(rad)만 갱신
  void setSelectedJointRad(int joint_index, double rad) {
    if (joint_index < 0 || joint_index >= num_joints_) return;
    std::lock_guard<std::mutex> lk(mtx_);
    target_angles_[joint_index] = rad;
  }

  // UI용 조인트 이름
  std::vector<std::string> jointNamesForUI() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> names(num_joints_);
    if (mapping_initialized_ && !last_js_names_.empty()) {
      for (int i = 0; i < num_joints_; ++i) {
        int idx = joint_mapping_[i];
        names[i] = (idx >= 0 && idx < static_cast<int>(last_js_names_.size()))
                  ? last_js_names_[idx]
                  : ("J" + std::to_string(i));
      }
    } else {
      for (int i = 0; i < num_joints_; ++i) names[i] = "J" + std::to_string(i);
    }
    return names;
  }

  // 1회 퍼블리시(선택 조인트만 바뀌고 나머지는 유지)
  void publishOnce() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!target_initialized_) return;

    LBRJointPositionCommand msg;
    const std::size_t N = msg.joint_position.size(); // 보통 7

    for (std::size_t i = 0; i < N; ++i) {
      double v = (i < target_angles_.size()) ? target_angles_[i] : 0.0;
      if (!std::isfinite(v)) {
        v = (i < current_angles_.size()) ? current_angles_[i] : 0.0;
      }
      msg.joint_position[i] = v;
    }
    kuka_pub_->publish(msg);
  }

  int numJoints() const { return num_joints_; }
  double publishRate() const { return publish_rate_; }

private:
  void initializeJointMapping(const sensor_msgs::msg::JointState::SharedPtr& msg) {
    // 가장 단순한 매핑: 앞에서부터 num_joints_개를 1:1 매핑
    int n = std::min<int>(num_joints_, msg->name.size());
    for (int i = 0; i < n; ++i) joint_mapping_[i] = i;
    mapping_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Joint mapping initialized (0..%d -> msg indices)", n-1);
  }

private:
  // params
  std::string robot_name_;
  int num_joints_{7};
  double publish_rate_{50.0};

  // topics
  std::string cmd_topic_;

  // ROS IO
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr JointState_sub_;
  rclcpp::Publisher<LBRJointPositionCommand>::SharedPtr kuka_pub_;

  // states
  std::vector<double> current_angles_;
  std::vector<double> target_angles_;
  std::vector<int>    joint_mapping_;
  std::vector<std::string> last_js_names_;
  bool current_angles_received_{false};
  bool mapping_initialized_{false};
  bool target_initialized_{false};

  std::mutex mtx_;
};

// ---------------- UI ----------------
class CommanderUI : public QWidget {
  Q_OBJECT
public:
  CommanderUI(std::shared_ptr<KukaCommanderNode> node,
              rclcpp::executors::SingleThreadedExecutor* exec)
  : node_(std::move(node)), exec_(exec)
  {
    setWindowTitle("Single Joint Commander (ROS2 + Qt)");
    auto* root = new QVBoxLayout();

    // Joint selector
    auto* row1 = new QHBoxLayout();
    row1->addWidget(new QLabel("Select Joint:"));
    joint_combo_ = new QComboBox();
    row1->addWidget(joint_combo_);
    root->addLayout(row1);

    // Dial + Slider
    auto* row2 = new QHBoxLayout();
    dial_ = new QDial(); dial_->setRange(-180, 180); dial_->setNotchesVisible(true);
    slider_ = new QSlider(Qt::Horizontal); slider_->setRange(-180, 180);
    row2->addWidget(dial_, 0);
    row2->addWidget(slider_, 1);
    root->addLayout(row2);

    // Value labels
    value_deg_ = new QLabel("0.0°");
    value_rad_ = new QLabel("0.000 rad");
    auto* row3 = new QHBoxLayout();
    row3->addWidget(new QLabel("Value:"));
    row3->addWidget(value_deg_);
    row3->addWidget(value_rad_);
    root->addLayout(row3);

    // Controls
    auto* row4 = new QHBoxLayout();
    continuous_chk_ = new QCheckBox("Continuous publish");
    send_btn_ = new QPushButton("Send Once");
    row4->addWidget(continuous_chk_);
    row4->addStretch();
    row4->addWidget(send_btn_);
    root->addLayout(row4);

    setLayout(root);
    resize(520, 220);

    // Signals
    connect(dial_,   &QDial::valueChanged,   this, &CommanderUI::onValueChanged);
    connect(slider_, &QSlider::valueChanged, this, &CommanderUI::onValueChanged);
    connect(send_btn_, &QPushButton::clicked, this, &CommanderUI::onSendOnce);

    // Sync dial & slider
    connect(dial_, &QDial::valueChanged, slider_, &QSlider::setValue);
    connect(slider_, &QSlider::valueChanged, dial_, &QDial::setValue);

    // Timer: ROS spin + UI refresh
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &CommanderUI::onSpinAndRefresh);
    spin_timer_->start(10); // 100Hz로 spin_some/라벨 갱신

    // Timer: publish
    pub_timer_ = new QTimer(this);
    connect(pub_timer_, &QTimer::timeout, this, &CommanderUI::onPublishTick);
    const int pub_ms = (int)std::round(1000.0 / std::max(1.0, node_->publishRate()));
    pub_timer_->start(pub_ms);

    refreshJointNamesIfReady();
  }

private slots:
  void onValueChanged(int deg) {
    int idx = joint_combo_->currentIndex();
    if (idx < 0) return;
    double rad = deg * M_PI / 180.0;
    value_deg_->setText(QString::number(deg, 'f', 1) + "°");
    value_rad_->setText(QString::number(rad, 'f', 3) + " rad");
    node_->setSelectedJointRad(idx, rad); // 선택된 조인트만 갱신
  }

  void onSendOnce() {
    node_->publishOnce();
  }

  void onPublishTick() {
    if (continuous_chk_->isChecked()) {
      node_->publishOnce();
    }
  }

  void onSpinAndRefresh() {
    if (exec_) exec_->spin_some();
    refreshJointNamesIfReady();
  }

private:
  void refreshJointNamesIfReady() {
    if (combo_initialized_) return;
    auto names = node_->jointNamesForUI();
    if ((int)names.size() == node_->numJoints()) {
      joint_combo_->clear();
      for (auto& n : names) joint_combo_->addItem(QString::fromStdString(n));
      joint_combo_->setCurrentIndex(0);
      combo_initialized_ = true;
    }
  }

private:
  std::shared_ptr<KukaCommanderNode> node_;
  rclcpp::executors::SingleThreadedExecutor* exec_{nullptr};

  QComboBox* joint_combo_{nullptr};
  QDial*     dial_{nullptr};
  QSlider*   slider_{nullptr};
  QLabel*    value_deg_{nullptr};
  QLabel*    value_rad_{nullptr};
  QCheckBox* continuous_chk_{nullptr};
  QPushButton* send_btn_{nullptr};

  QTimer* spin_timer_{nullptr};
  QTimer* pub_timer_{nullptr};

  bool combo_initialized_{false};
};

// ---- main ----
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<KukaCommanderNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  QApplication app(argc, argv);
  CommanderUI ui(node, &exec);
  ui.show();

  int ret = app.exec();
  exec.cancel();
  rclcpp::shutdown();
  return ret;
}

// ✅ Q_OBJECT가 .cpp 안에 있으므로 moc를 포함 (AUTOMOC 사용)
#include "single_joint_commander.moc"
