cmake_minimum_required(VERSION 3.8)

project(carkit_joy_rate_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_rate_filter_node src/joy_rate_filter_node.cpp)
ament_target_dependencies(joy_rate_filter_node rclcpp sensor_msgs)

install(
  TARGETS joy_rate_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
