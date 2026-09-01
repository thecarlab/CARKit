# CARKit learning annotation: defines build, dependency, and install rules for carkit_behavior.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_behavior)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(carkit_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_library(behavior_engine
  src/behavior_engine.cpp
  src/path_geometry.cpp
)
target_include_directories(behavior_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(behavior_center_node src/behavior_center_node.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
target_link_libraries(behavior_center_node behavior_engine)
ament_target_dependencies(
  behavior_center_node
  ackermann_msgs
  carkit_perception_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS behavior_engine behavior_center_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_behavior_engine test/test_behavior_engine.cpp)
  target_link_libraries(test_behavior_engine behavior_engine)
  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp)
  target_link_libraries(test_path_geometry behavior_engine)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(behavior_engine)
# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
