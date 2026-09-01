// Copyright 2026 University of Delaware
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class TwistToAckermann : public rclcpp::Node
{
public:
  TwistToAckermann()
  : Node("twist_to_ackermann")
  {
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto ackermann_topic = declare_parameter<std::string>("ackermann_topic", "/drive");
    wheelbase_ = declare_parameter("wheelbase", 0.25);
    max_speed_ = std::abs(declare_parameter("max_speed", 1.0));
    max_reverse_speed_ = std::abs(declare_parameter("max_reverse_speed", 0.4));
    max_steering_angle_ = std::abs(declare_parameter("max_steering_angle", 0.27));
    min_speed_for_steering_ = std::abs(
      declare_parameter("min_speed_for_steering", 0.05));

    publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      ackermann_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {on_command(*message);});
    RCLCPP_INFO(
      get_logger(), "C++ Twist adapter: %s -> %s", cmd_vel_topic.c_str(),
      ackermann_topic.c_str());
  }

private:
  /// Convert yaw rate to steering angle with the bicycle model: tan(delta)=L*w/v.
  /// Near zero speed the equation is ill-conditioned, so steering safely returns zero.
  double steering_from_twist(double speed, double yaw_rate) const
  {
    if (std::abs(speed) < min_speed_for_steering_ || std::abs(yaw_rate) < 1.0e-6) {
      return 0.0;
    }
    return std::clamp(
      std::atan(wheelbase_ * yaw_rate / speed), -max_steering_angle_, max_steering_angle_);
  }

  void on_command(const geometry_msgs::msg::Twist & message)
  {
    const double raw_speed = std::isfinite(message.linear.x) ? message.linear.x : 0.0;
    const double raw_yaw_rate = std::isfinite(message.angular.z) ? message.angular.z : 0.0;
    const double speed = std::clamp(raw_speed, -max_reverse_speed_, max_speed_);

    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.drive.speed = speed;
    output.drive.steering_angle = steering_from_twist(speed, raw_yaw_rate);
    publisher_->publish(output);
  }

  double wheelbase_{0.25};
  double max_speed_{1.0};
  double max_reverse_speed_{0.4};
  double max_steering_angle_{0.27};
  double min_speed_for_steering_{0.05};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckermann>());
  rclcpp::shutdown();
  return 0;
}
