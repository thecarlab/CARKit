# CARKit learning annotation: defines build, dependency, and install rules for carkit_slam.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(carkit_slam)

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(
  DIRECTORY config launch rviz
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
