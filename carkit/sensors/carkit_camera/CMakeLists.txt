# CARKit learning annotation: defines build, dependency, and install rules for carkit_camera.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_camera)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(low_latency_camera_node src/low_latency_camera_node.cpp)
target_compile_features(low_latency_camera_node PUBLIC cxx_std_17)
target_include_directories(low_latency_camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
ament_target_dependencies(
  low_latency_camera_node camera_info_manager rclcpp sensor_msgs)
target_link_libraries(low_latency_camera_node ${OpenCV_LIBRARIES})

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS low_latency_camera_node DESTINATION lib/${PROJECT_NAME})

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
