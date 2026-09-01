# CARKit learning annotation: defines build, dependency, and install rules for carkit_scan_filter.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_scan_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(scan_footprint_filter_node src/scan_footprint_filter_node.cpp)
target_compile_features(scan_footprint_filter_node PUBLIC cxx_std_17)
target_include_directories(scan_footprint_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(scan_footprint_filter_node rclcpp sensor_msgs)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS scan_footprint_filter_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footprint_filter test/test_footprint_filter.cpp)
  target_compile_features(test_footprint_filter PUBLIC cxx_std_17)
  target_include_directories(test_footprint_filter PUBLIC include)
endif()

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
