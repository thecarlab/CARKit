// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "carkit_intro2av_cpp/control_algorithm.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class Intro2AvControl : public rclcpp::Node
{
public:
  Intro2AvControl() : Node("carkit_intro2av_cpp_control")
  {
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "/drive");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto rate = std::max(1.0, declare_parameter("control_rate_hz", 10.0));
    declare_parameter("input_timeout_sec", 0.5);
    declare_parameter("wheelbase_m", 0.325);
    declare_parameter("lookahead_m", 0.55);
    declare_parameter("target_speed_mps", 0.45);
    declare_parameter("maximum_speed_mps", 1.0);
    declare_parameter("maximum_steering_rad", 0.34);
    declare_parameter("goal_tolerance_m", 0.15);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, 10, [this](nav_msgs::msg::Path::ConstSharedPtr message) {
        path_ = message;
        path_received_at_ = std::chrono::steady_clock::now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_ = message;
        odom_received_at_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {publish_command();});
    RCLCPP_INFO(
      get_logger(),
      "Intro2AV C++ controller ready; implement control_algorithm.cpp");
  }

private:
  carkit_intro2av_cpp::ControlConfig configuration() const
  {
    carkit_intro2av_cpp::ControlConfig config;
    config.wheelbase_m = get_parameter("wheelbase_m").as_double();
    config.lookahead_m = get_parameter("lookahead_m").as_double();
    config.target_speed_mps = get_parameter("target_speed_mps").as_double();
    config.maximum_speed_mps = get_parameter("maximum_speed_mps").as_double();
    config.maximum_steering_rad = get_parameter("maximum_steering_rad").as_double();
    config.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
    return config;
  }

  void publish_command()
  {
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_;
    carkit_intro2av_cpp::ControlCommand output;
    const auto monotonic_now = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(
      get_parameter("input_timeout_sec").as_double());
    const bool fresh =
      path_ && odom_ && !path_->poses.empty() &&
      monotonic_now - path_received_at_ <= timeout &&
      monotonic_now - odom_received_at_ <= timeout;
    if (fresh) {
      try {
        output = carkit_intro2av_cpp::compute_command(*odom_, *path_, configuration());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Control algorithm failed: %s", error.what());
      }
    }
    if (!std::isfinite(output.speed_mps) || !std::isfinite(output.steering_angle_rad)) {
      RCLCPP_ERROR(get_logger(), "Control algorithm returned non-finite output");
      output = {};
    }
    const double speed_limit = std::abs(get_parameter("maximum_speed_mps").as_double());
    const double steering_limit = std::abs(get_parameter("maximum_steering_rad").as_double());
    message.drive.speed = std::clamp(output.speed_mps, -speed_limit, speed_limit);
    message.drive.steering_angle =
      std::clamp(output.steering_angle_rad, -steering_limit, steering_limit);
    drive_pub_->publish(message);
  }

  std::string plan_topic_, odom_topic_, drive_topic_, base_frame_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::chrono::steady_clock::time_point path_received_at_{};
  std::chrono::steady_clock::time_point odom_received_at_{};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intro2AvControl>());
  rclcpp::shutdown();
  return 0;
}
