# CARKit learning annotation: defines build, dependency, and install rules for carkit_web_bridge.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_web_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(carkit_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(vesc_msgs QUIET)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(web_bridge_node src/web_bridge_node.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(
  web_bridge_node
  ackermann_msgs
  carkit_perception_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(
  web_bridge_node
  Boost::system
  nlohmann_json::nlohmann_json
  Threads::Threads
)

if(vesc_msgs_FOUND)
  ament_target_dependencies(web_bridge_node vesc_msgs)
  target_compile_definitions(web_bridge_node PRIVATE CARKIT_HAS_VESC_MSGS=1)
endif()

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS web_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
