# CARKit learning annotation: defines build, dependency, and install rules for carkit_perception_msgs.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_perception_msgs)

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# CARKit build note: generates ROS language bindings for the listed message and service definitions.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloDetection2D.msg"
  "msg/YoloTrafficLightDetection2D.msg"
  "msg/YoloDetection2DArray.msg"
  DEPENDENCIES std_msgs
)

# CARKit build note: exports dependency information for packages that consume this package.
ament_export_dependencies(rosidl_default_runtime)
# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()
