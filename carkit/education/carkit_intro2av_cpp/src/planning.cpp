// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "carkit_intro2av_cpp/planning_algorithm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
using namespace std::chrono_literals;

class Intro2AvPlanning : public rclcpp::Node
{
public:
  Intro2AvPlanning() : Node("carkit_intro2av_cpp_planning")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    const auto rate = std::max(0.1, declare_parameter("planning_rate_hz", 2.0));
    declare_parameter("occupancy_threshold", 65);
    declare_parameter("allow_unknown", false);
    declare_parameter("inflation_radius_m", 0.25);
    declare_parameter("waypoint_spacing_m", 0.10);
    declare_parameter("goal_tolerance_m", 0.15);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {map_ = message;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {odom_ = message;});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        goal_ = message;
        ++goal_revision_;
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {update_plan();});
    RCLCPP_INFO(
      get_logger(),
      "Intro2AV C++ planner ready; implement planning_algorithm.cpp");
  }

private:
  carkit_intro2av_cpp::PlanningConfig configuration() const
  {
    carkit_intro2av_cpp::PlanningConfig config;
    config.occupancy_threshold = get_parameter("occupancy_threshold").as_int();
    config.allow_unknown = get_parameter("allow_unknown").as_bool();
    config.inflation_radius_m = get_parameter("inflation_radius_m").as_double();
    config.waypoint_spacing_m = get_parameter("waypoint_spacing_m").as_double();
    config.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
    return config;
  }

  nav_msgs::msg::Path empty_path() const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;
    return path;
  }

  void update_plan()
  {
    if (!map_ || !odom_ || !goal_) {
      return;
    }
    const auto monotonic_now = std::chrono::steady_clock::now();
    if (
      planned_revision_ == goal_revision_ &&
      monotonic_now - last_plan_time_ < 1s)
    {
      return;
    }
    last_plan_time_ = monotonic_now;
    auto path = empty_path();
    try {
      auto poses = carkit_intro2av_cpp::compute_path(*map_, *odom_, *goal_, configuration());
      for (auto & pose : poses) {
        if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
          throw std::runtime_error("planner returned a non-finite waypoint");
        }
        pose.header = path.header;
        path.poses.push_back(std::move(pose));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Planning algorithm failed: %s", error.what());
      path = empty_path();
    }
    planned_revision_ = goal_revision_;
    plan_pub_->publish(path);
  }

  std::string map_topic_, odom_topic_, goal_topic_, plan_topic_, global_frame_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;
  uint64_t goal_revision_{0};
  uint64_t planned_revision_{static_cast<uint64_t>(-1)};
  std::chrono::steady_clock::time_point last_plan_time_{};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intro2AvPlanning>());
  rclcpp::shutdown();
  return 0;
}
