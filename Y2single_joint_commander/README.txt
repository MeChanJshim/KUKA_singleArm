# 필요한 패키지
sudo apt-get install -y qtbase5-dev

# 워크스페이스에서
colcon build --packages-select single_joint_commander
source install/setup.bash

# 파라미터로 로봇 이름/조인트 개수/주기 조정 가능
ros2 run Y2single_joint_commander Y2single_joint_commander \
  --ros-args -p robot_name:=/lbr -p numOfJoints:=7 -p publish_rate_hz:=10.0



[ 동작 방식 (중요) ]

JointState를 받아 current_angles에 저장(네가 쓰는 매핑 로직 유지).

UI에서 조인트 하나 선택 → 노브/슬라이더를 돌리면 그 조인트의 target_angles[idx]만 갱신.

전송 시(Send Once 혹은 Continuous 체크):

msg.joint_position[i] = (i==선택조인트) ? target_angles[i] : current_angles[i]

즉 선택한 조인트만 바꾸고 나머지는 현재값으로 채워서 안전하게 한 축씩 명령.