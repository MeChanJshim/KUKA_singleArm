#include "Y2RobMotion/kuka_motion.hpp"
#include "lbr_fri_ros2/utils.hpp"

/**** ur_motion (ur_motion) ****/
#define ROBOT_MODEL "/lbr"
#define CONTROL_PERIOD 0.01
#define NUMBER_OF_JOINTS 7

/* EE to TCP HTM setting (translation: mm) */
const YMatrix EE2TCP = {
    {-1.0,  0.0,  0.0,  0.0 },
    { 0.0,  1.0,  0.0,  0.0 },
    { 0.0,  0.0, -1.0,  90  },
    { 0.0,  0.0,  0.0,  1.0 }
};

// const YMatrix EE2TCP = {
//     { 1.0,  0.0,  0.0,  0.0},
//     { 0.0,  1.0,  0.0,  0.0},
//     { 0.0,  0.0,  1.0,  0.0},
//     { 0.0,  0.0,  0.0,  1.0}
// };

class singleArm_motion
{
    public:
        singleArm_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, 
            int numOfJoint, const YMatrix& HTMEE2TCP);

        bool jointsReceived() const {
            return KUKAMotion->jointsReceived();
        }
    
        void start(bool start_flag = true) {
            KUKAMotion->start(start_flag);
        }

    private:
    
        std::unique_ptr<kuka_motion> KUKAMotion;
        double kuka_ctrPeriod;
        rclcpp::Node::SharedPtr node_;
        
};

singleArm_motion::singleArm_motion(rclcpp::Node::SharedPtr node, const std::string& RB_name, 
        int numOfJoint, const YMatrix& HTMEE2TCP)
: node_(node)
{
    std::string CMService_TP = RB_name + "/controller_manager";
    auto update_rate = lbr_fri_ros2::retrieve_paramter(node_, CMService_TP, "update_rate").as_int();
    kuka_ctrPeriod = 1.0 / static_cast<double>(update_rate);
    KUKAMotion = std::make_unique<kuka_motion>(node_,RB_name,kuka_ctrPeriod,numOfJoint,HTMEE2TCP);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    /* ROS node generation as hip instance */

    auto node = std::make_shared<rclcpp::Node>("urSingleArm");
    singleArm_motion kuka_SingleArm(node,ROBOT_MODEL,NUMBER_OF_JOINTS, EE2TCP);
    // auto node = std::make_shared<singleArm_motion>(
    //     node_name, ROBOT_MODEL, NUMBER_OF_JOINTS, EE2TCP);
    RCLCPP_INFO(node->get_logger(), "Waiting for joint states...");
    
    /* Node generation */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    /* Bringup execution check */
    while (rclcpp::ok() && !kuka_SingleArm.jointsReceived()) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "Joint states received!");
    
    /* Inqury of starting */
    bool start_boolean = false;
    std::cout << "\033[33m [" << ROBOT_MODEL 
              << "] start? (1: start, 0: quit) \033[0m";
    std::cin >> start_boolean;
    
    if (!start_boolean) {
        rclcpp::shutdown();
        return 0;
    }
    
    /* Node execution */
    kuka_SingleArm.start(true);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}