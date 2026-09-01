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

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class OdomTfBroadcaster : public rclcpp::Node
{
public:
  OdomTfBroadcaster()
  : Node("odom_tf_broadcaster")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    use_message_stamp_ = declare_parameter("use_message_stamp", true);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odometry(*message);});
    RCLCPP_INFO(
      get_logger(), "C++ TF adapter: %s -> %s from %s", odom_frame_.c_str(),
      base_frame_.c_str(), odom_topic.c_str());
  }

private:
  /// Republish the odometry pose as the odom-to-base transform expected by Nav2.
  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    geometry_msgs::msg::TransformStamped output;
    if (use_message_stamp_) {
      output.header.stamp = message.header.stamp;
    } else {
      output.header.stamp = now();
    }
    output.header.frame_id = odom_frame_.empty() ? message.header.frame_id : odom_frame_;
    output.child_frame_id = base_frame_.empty() ? message.child_frame_id : base_frame_;
    output.transform.translation.x = message.pose.pose.position.x;
    output.transform.translation.y = message.pose.pose.position.y;
    output.transform.translation.z = message.pose.pose.position.z;
    output.transform.rotation = message.pose.pose.orientation;
    broadcaster_->sendTransform(output);
  }

  std::string odom_frame_;
  std::string base_frame_;
  bool use_message_stamp_{true};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
