cmake_minimum_required(VERSION 3.8)
project(carkit_latency_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(carkit_perception_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(stop_latency_monitor src/stop_latency_monitor.cpp)
ament_target_dependencies(stop_latency_monitor
  builtin_interfaces
  carkit_perception_msgs
  nav_msgs
  rclcpp
)

install(TARGETS stop_latency_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
