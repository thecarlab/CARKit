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
#include <cctype>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class FoxgloveWaypoints : public rclcpp::Node
{
public:
  using Navigate = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  FoxgloveWaypoints()
  : Node("foxglove_waypoints")
  {
    const auto goal_topic = declare_parameter<std::string>(
      "goal_topic", "/foxglove/waypoints/goal");
    const auto command_topic = declare_parameter<std::string>(
      "command_topic", "/foxglove/waypoints/command");
    const auto marker_topic = declare_parameter<std::string>(
      "marker_topic", "/foxglove/waypoints/markers");
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/foxglove/waypoints/status");
    const auto action_name = declare_parameter<std::string>(
      "action_name", "/navigate_through_poses");
    default_frame_ = declare_parameter<std::string>("default_frame", "map");
    maximum_poses_ = std::max<int64_t>(1, declare_parameter("maximum_poses", 50));

    auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, status_qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, status_qos);
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {on_pose(*message);});
    command_subscription_ = create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_command(message->data);});
    action_client_ = rclcpp_action::create_client<Navigate>(this, action_name);
    publish_markers();
    publish_status("Ready: publish poses to accumulate a route");
  }

private:
  static std::string lower(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
    return value;
  }

  void on_pose(const geometry_msgs::msg::PoseStamped & message)
  {
    if (pending_.size() >= static_cast<std::size_t>(maximum_poses_)) {
      publish_status("Pose limit reached (" + std::to_string(maximum_poses_) + ")");
      return;
    }
    auto pose = message;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = default_frame_;
    }
    const auto & orientation = pose.pose.orientation;
    if (orientation.x == 0.0 && orientation.y == 0.0 &&
      orientation.z == 0.0 && orientation.w == 0.0)
    {
      pose.pose.orientation.w = 1.0;
    }
    pending_.push_back(std::move(pose));
    publish_markers();
    publish_status("Accumulated " + std::to_string(pending_.size()) + " pose(s)");
  }

  void on_command(const std::string & raw_command)
  {
    const auto command = lower(raw_command);
    if (command == "start" || command == "run" || command == "navigate") {
      start();
    } else if (command == "clear" || command == "reset") {
      pending_.clear();
      publish_markers();
      publish_status("Pending poses cleared");
    } else if (command == "cancel" || command == "stop") {
      cancel();
    } else {
      publish_status("Unknown command \"" + raw_command + "\"; use start, clear, or cancel");
    }
  }

  void start()
  {
    if (sending_ || active_goal_) {
      publish_status("A route is already starting or running");
      return;
    }
    if (pending_.empty()) {
      publish_status("No poses accumulated");
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      publish_status("Nav2 /navigate_through_poses is unavailable");
      return;
    }

    submitted_ = std::move(pending_);
    pending_.clear();
    Navigate::Goal goal;
    goal.poses = submitted_;
    sending_ = true;
    publish_markers();
    publish_status("Starting route with " + std::to_string(goal.poses.size()) + " pose(s)");

    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
        sending_ = false;
        if (!goal_handle) {
          pending_.insert(pending_.begin(), submitted_.begin(), submitted_.end());
          submitted_.clear();
          publish_markers();
          publish_status("Nav2 rejected the route");
          return;
        }
        active_goal_ = goal_handle;
        active_ = std::move(submitted_);
        submitted_.clear();
        publish_markers();
        publish_status("Navigating through " + std::to_string(active_.size()) + " pose(s)");
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        std::string status;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            status = "Route completed";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            status = "Route canceled";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            status = "Route aborted by Nav2";
            break;
          default:
            status = "Route ended with an unknown status";
            break;
        }
        active_goal_.reset();
        active_.clear();
        publish_markers();
        publish_status(status);
      };
    action_client_->async_send_goal(goal, options);
  }

  void cancel()
  {
    if (!active_goal_) {
      publish_status("No active route to cancel");
      return;
    }
    publish_status("Canceling active route");
    action_client_->async_cancel_goal(active_goal_);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void append_markers(
    visualization_msgs::msg::MarkerArray & output,
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    const std::string & state, float red, float green, float blue, int & marker_id) const
  {
    int index = 1;
    for (const auto & pose : poses) {
      visualization_msgs::msg::Marker arrow;
      arrow.header = pose.header;
      arrow.ns = "foxglove_waypoints_" + state;
      arrow.id = marker_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = pose.pose;
      arrow.scale.x = 0.65;
      arrow.scale.y = 0.14;
      arrow.scale.z = 0.14;
      arrow.color.r = red;
      arrow.color.g = green;
      arrow.color.b = blue;
      arrow.color.a = 1.0;
      output.markers.push_back(arrow);

      auto label = arrow;
      label.ns += "_labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += 0.45;
      label.scale.x = 0.0;
      label.scale.y = 0.0;
      label.scale.z = 0.35;
      label.text = std::to_string(index++);
      output.markers.push_back(label);
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    int marker_id = 0;
    append_markers(output, active_, "active", 0.1F, 0.55F, 1.0F, marker_id);
    append_markers(output, pending_, "pending", 0.1F, 1.0F, 0.35F, marker_id);
    marker_publisher_->publish(output);
  }

  std::string default_frame_;
  int64_t maximum_poses_{50};
  bool sending_{false};
  std::vector<geometry_msgs::msg::PoseStamped> pending_;
  std::vector<geometry_msgs::msg::PoseStamped> active_;
  std::vector<geometry_msgs::msg::PoseStamped> submitted_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp_action::Client<Navigate>::SharedPtr action_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FoxgloveWaypoints>());
  rclcpp::shutdown();
  return 0;
}
