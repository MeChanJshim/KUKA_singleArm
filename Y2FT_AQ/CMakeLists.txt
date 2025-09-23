cmake_minimum_required(VERSION 3.8)
project(Y2FT_AQ)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Y2Filters REQUIRED)
find_package(Y2Matrix REQUIRED)

include_directories(include)

# === Create libraries ===
# FT_rawGet library
add_library(FT_rawGet src/FT_rawGet.cpp)
target_include_directories(FT_rawGet PUBLIC
  $<BUILD_INTERFACE:${CAMKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(FT_rawGet rclcpp Y2Matrix)

# === Create executables ===
# FT_rawGet_Example
add_executable(FT_rawGet_Example examples/FT_rawGet_Example.cpp)
target_link_libraries(FT_rawGet_Example FT_rawGet)
ament_target_dependencies(FT_rawGet_Example rclcpp)

# FTGetMain
add_executable(FTGetMain src/FTGetMain.cpp)
target_link_libraries(FTGetMain FT_rawGet)
ament_target_dependencies(FTGetMain rclcpp Y2Filters geometry_msgs std_msgs Y2Matrix)

# === Create install ===
# install libraries
install(TARGETS FT_rawGet
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install executables
install(TARGETS FT_rawGet_Example FTGetMain
  DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
  DESTINATION include/
)

# === Export targets & dependencies === 
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs Y2Filters Y2Matrix)



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
