cmake_minimum_required(VERSION 3.8)
project(Y2RobMotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Y2Matrix REQUIRED)
find_package(Y2Kinematics REQUIRED)
find_package(Y2Trajectory REQUIRED)
find_package(Y2ForceCon REQUIRED)
find_package(osqp REQUIRED)

# find dependencies for kuka
find_package(FRIClient REQUIRED)
find_package(lbr_fri_idl REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)

set(AMENT_DEP
rclcpp sensor_msgs geometry_msgs
Y2Matrix Y2Kinematics Y2Trajectory Y2ForceCon
)

include_directories(
  include
)

# Main controller sources
set(MAIN_SOURCES
src/kuka_motion.cpp
src/hand_guiding.cpp
src/force_control.cpp
src/state_monitoring.cpp)

# === Create libraries === 
# kuka_motion library
add_library(kuka_motion ${MAIN_SOURCES})
target_include_directories(kuka_motion PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(kuka_motion ${AMENT_DEP} lbr_fri_idl lbr_fri_ros2)
target_link_libraries(kuka_motion FRIClient::FRIClient)

# robot_command library
add_library(robot_command src/robot_command.cpp)
target_include_directories(robot_command PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_command ${AMENT_DEP})

# === Create executions ===
# singleArm_motion executable
add_executable(singleArm_motion src/singleArm_motion.cpp)
target_link_libraries(singleArm_motion kuka_motion)

# singleArm_cmd executable
add_executable(singleArm_cmd src/singleArm_cmd.cpp)
target_link_libraries(singleArm_cmd robot_command)

# robot_measure executable
add_executable(robot_measure src/robot_measure.cpp)
ament_target_dependencies(robot_measure rclcpp std_msgs)

# === Create installs ===
# install libraries
install(TARGETS kuka_motion robot_command
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install executables
install(TARGETS singleArm_motion singleArm_cmd robot_measure
  DESTINATION lib/${PROJECT_NAME}
)

# install includes
install(DIRECTORY include/
DESTINATION include/)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEP})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
