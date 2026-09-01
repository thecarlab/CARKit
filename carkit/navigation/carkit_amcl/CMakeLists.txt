# CARKit learning annotation: defines build, dependency, and install rules for carkit_amcl.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_amcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(twist_to_ackermann src/twist_to_ackermann.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(twist_to_ackermann ackermann_msgs geometry_msgs rclcpp)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster geometry_msgs nav_msgs rclcpp tf2_ros)

add_executable(foxglove_waypoints src/foxglove_waypoints.cpp)
ament_target_dependencies(
  foxglove_waypoints
  action_msgs
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  visualization_msgs
)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(
  TARGETS twist_to_ackermann odom_tf_broadcaster foxglove_waypoints
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY behavior_trees config launch
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
