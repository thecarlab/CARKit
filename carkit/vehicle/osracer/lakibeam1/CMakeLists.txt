# CARKit learning annotation: defines build, dependency, and install rules for lakibeam1.
cmake_minimum_required(VERSION 3.8)
# CARKit build note: sets the package identity and compiler languages used below.
project(lakibeam1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CARKit build note: resolves build-time dependencies and makes their CMake targets available.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # This package is a hardware-vendor drop. Its Python launch files and bundled
  # RapidJSON headers do not follow ament formatting, and uncrustify cannot
  # parse the vendored headers. Keep structural/CMake/XML checks enabled while
  # avoiding false failures over code maintained outside CARKit.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# CARKit build note: finalizes the ament package after all targets and install rules are declared.
ament_package()

include_directories(
# include
  ${ament_INCLUDE_DIRS}
)

# CARKit build note: defines a compiled target and the source files that implement it.
add_executable(${PROJECT_NAME}_scan_node src/lakibeam1_scan.cpp src/remote.cpp)
# CARKit build note: attaches ROS dependencies and their transitive include/link settings to the target.
target_link_libraries(${PROJECT_NAME}_scan_node ${ament_LIBRARIES} curl)
ament_target_dependencies(${PROJECT_NAME}_scan_node rclcpp sensor_msgs std_msgs)

# CARKit build note: copies runtime targets and resources into the ROS install space.
install(TARGETS ${PROJECT_NAME}_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)
