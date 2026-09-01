# CARKit learning annotation: defines build, dependency, and install rules for carkit_intro2av_cpp.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_intro2av_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(carkit_perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(planning_node src/planning.cpp src/planning_algorithm.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(planning_node geometry_msgs nav_msgs rclcpp)

add_executable(control_node src/control.cpp src/control_algorithm.cpp)
ament_target_dependencies(control_node ackermann_msgs nav_msgs rclcpp)

add_executable(perception_node src/perception.cpp src/perception_algorithm.cpp)
ament_target_dependencies(
  perception_node carkit_perception_msgs rclcpp sensor_msgs)

foreach(target planning_node control_node perception_node)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS planning_node control_node perception_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
