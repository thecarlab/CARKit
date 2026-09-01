# CARKit learning annotation: defines build, dependency, and install rules for carkit_control_center.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_control_center)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(control_center_node src/control_center_node.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(
  control_center_node
  ackermann_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS control_center_node DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
