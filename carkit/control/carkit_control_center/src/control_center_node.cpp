// Copyright 2026 ADA

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
using Drive = ackermann_msgs::msg::AckermannDriveStamped;

namespace
{
constexpr char kHumanControl[] = "HUMAN_CONTROL";
constexpr char kAutoDrive[] = "AUTO_DRIVE";
constexpr char kEmergencyStop[] = "EMERGENCY_STOP";

double finite_clamp(double value, double limit)
{
  return std::isfinite(value) ? std::clamp(value, -limit, limit) : 0.0;
}

template<typename MessageT>
struct TimedMessage
{
  typename MessageT::SharedPtr message;
  double received_at{0.0};

  void update(typename MessageT::SharedPtr value, double now)
  {
    message = std::move(value);
    received_at = now;
  }

  bool fresh(double now, double timeout) const
  {
    return message && now - received_at <= timeout;
  }
};
}  // namespace

class ControlCenterNode : public rclcpp::Node
{
public:
  ControlCenterNode()
  : Node("control_center_node")
  {
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 30.0);
    status_rate_hz_ = std::max(0.1, declare_parameter("status_publish_rate_hz", 2.0));
    auto_button_ = declare_parameter("auto_button", 0);
    human_button_ = declare_parameter("human_button", 1);
    estop_button_ = declare_parameter("estop_button", 2);
    clear_estop_button_ = declare_parameter("clear_estop_button", 3);
    teleop_timeout_ = declare_parameter("teleop_timeout_sec", 0.30);
    nav2_timeout_ = declare_parameter("nav2_timeout_sec", 0.50);
    behavior_timeout_ = declare_parameter("behavior_timeout_sec", 0.50);
    max_speed_ = std::abs(declare_parameter("max_speed", 1.0));
    max_steering_ = std::abs(declare_parameter("max_steering_angle", 0.34));
    state_ = declare_parameter<std::string>("initial_state", kHumanControl);
    use_enable_topic_ = declare_parameter("use_autonomy_enable_topic", true);
    enable_topic_ = declare_parameter<std::string>(
      "autonomy_enable_topic", "enable_autonomous_control");
    if (state_ != kHumanControl && state_ != kAutoDrive && state_ != kEmergencyStop) {
      RCLCPP_WARN(get_logger(), "Invalid initial_state=%s; using HUMAN_CONTROL", state_.c_str());
      state_ = kHumanControl;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(std::move(msg));});
    teleop_sub_ = create_subscription<Drive>(
      "/teleop", 10, [this](Drive::SharedPtr msg) {teleop_.update(std::move(msg), seconds());});
    drive_sub_ = create_subscription<Drive>(
      "/drive", 10, [this](Drive::SharedPtr msg) {nav2_.update(std::move(msg), seconds());});
    behavior_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/behavior/override_active", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        behavior_active_.update(std::move(msg), seconds());
      });
    behavior_cmd_sub_ = create_subscription<Drive>(
      "/behavior/override_cmd", 10,
      [this](Drive::SharedPtr msg) {behavior_cmd_.update(std::move(msg), seconds());});
    if (use_enable_topic_) {
      enable_sub_ = create_subscription<std_msgs::msg::Int8>(
        enable_topic_, 10,
        [this](std_msgs::msg::Int8::SharedPtr msg) {on_enable(*msg);});
    }

    command_pub_ = create_publisher<Drive>("/ackermann_cmd", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/control_center/main_state", 10);
    selected_pub_ = create_publisher<std_msgs::msg::String>("/control_center/selected_cmd", 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>("/control_center/debug", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
      [this]() {publish_command();});
    RCLCPP_INFO(
      get_logger(), "C++ control center started in %s at %.1f Hz",
      state_.c_str(), publish_rate_hz_);
  }

private:
  double seconds() {return get_clock()->now().seconds();}

  bool rising_edge(const std::vector<int32_t> & buttons, int index) const
  {
    if (index < 0 || static_cast<size_t>(index) >= buttons.size()) {
      return false;
    }
    const int previous = static_cast<size_t>(index) < previous_buttons_.size() ?
      previous_buttons_[index] : 0;
    return buttons[index] != 0 && previous == 0;
  }

  void on_joy(sensor_msgs::msg::Joy::SharedPtr message)
  {
    if (rising_edge(message->buttons, estop_button_)) {
      state_ = kEmergencyStop;
    }
    if (rising_edge(message->buttons, clear_estop_button_) && state_ == kEmergencyStop) {
      state_ = use_enable_topic_ && last_enable_ == 1 ? kAutoDrive : kHumanControl;
    }
    if (!use_enable_topic_ && state_ != kEmergencyStop) {
      if (rising_edge(message->buttons, human_button_)) {
        state_ = kHumanControl;
        last_enable_ = 0;
      }
      if (rising_edge(message->buttons, auto_button_)) {
        state_ = kAutoDrive;
        last_enable_ = 1;
      }
    }
    previous_buttons_ = message->buttons;
  }

  void on_enable(const std_msgs::msg::Int8 & message)
  {
    if (message.data != 0 && message.data != 1) {
      return;
    }
    last_enable_ = message.data;
    if (state_ != kEmergencyStop) {
      const std::string previous = state_;
      state_ = last_enable_ == 1 ? kAutoDrive : kHumanControl;
      if (state_ != previous) {
        RCLCPP_INFO(get_logger(), "Mode switched to %s", state_.c_str());
      }
    }
  }

  Drive zero_command() const
  {
    Drive command;
    command.drive.speed = 0.0;
    command.drive.steering_angle = 0.0;
    return command;
  }

  void publish_text(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    publisher->publish(message);
  }

  void publish_status(const std::string & selected, double current_time)
  {
    const bool changed = state_ != last_state_ || selected != last_selected_;
    const bool heartbeat = !last_status_time_.has_value() ||
      current_time - *last_status_time_ >= 1.0 / status_rate_hz_;
    if (!changed && !heartbeat) {
      return;
    }
    publish_text(state_pub_, state_);
    publish_text(selected_pub_, selected);
    last_status_time_ = current_time;
    last_state_ = state_;
    last_selected_ = selected;
  }

  void publish_debug(const std::string & selected, double current_time)
  {
    if (++debug_counter_ % std::max(1, static_cast<int>(publish_rate_hz_)) != 0) {
      return;
    }
    std::ostringstream stream;
    stream << "state=" << state_ << " selected=" << selected
           << " teleop_fresh=" << teleop_.fresh(current_time, teleop_timeout_)
           << " nav2_fresh=" << nav2_.fresh(current_time, nav2_timeout_)
           << " behavior_active_fresh=" << behavior_active_.fresh(current_time, behavior_timeout_)
           << " behavior_cmd_fresh=" << behavior_cmd_.fresh(current_time, behavior_timeout_);
    publish_text(debug_pub_, stream.str());
  }

  /// Arbitrate exactly one hardware command using safety-first priority:
  /// emergency stop, fresh human input, behavior override, then fresh Nav2 input.
  void publish_command()
  {
    const double current_time = seconds();
    std::string selected;
    Drive command;
    if (state_ == kEmergencyStop) {
      command = zero_command();
      selected = "emergency_stop";
    } else if (state_ == kHumanControl) {
      if (teleop_.fresh(current_time, teleop_timeout_)) {
        command = *teleop_.message;
        selected = "teleop";
      } else {
        command = zero_command();
        selected = "teleop_stale_zero";
      }
    } else if (state_ == kAutoDrive) {
      const bool override_active = behavior_active_.fresh(current_time, behavior_timeout_) &&
        behavior_active_.message->data;
      if (override_active && behavior_cmd_.fresh(current_time, behavior_timeout_)) {
        command = *behavior_cmd_.message;
        selected = "behavior_override";
      } else if (nav2_.fresh(current_time, nav2_timeout_)) {
        command = *nav2_.message;
        selected = "nav2_drive";
      } else {
        command = zero_command();
        selected = "nav2_stale_zero";
      }
    } else {
      command = zero_command();
      selected = "invalid_state_zero";
    }
    command.drive.speed = finite_clamp(command.drive.speed, max_speed_);
    command.drive.steering_angle = finite_clamp(command.drive.steering_angle, max_steering_);
    command.header.stamp = now();
    command_pub_->publish(command);
    publish_status(selected, current_time);
    publish_debug(selected, current_time);
  }

  double publish_rate_hz_{30.0};
  double status_rate_hz_{2.0};
  double teleop_timeout_{0.3};
  double nav2_timeout_{0.5};
  double behavior_timeout_{0.5};
  double max_speed_{1.0};
  double max_steering_{0.34};
  int auto_button_{0};
  int human_button_{1};
  int estop_button_{2};
  int clear_estop_button_{3};
  int last_enable_{0};
  int debug_counter_{0};
  bool use_enable_topic_{true};
  std::string enable_topic_;
  std::string state_;
  std::string last_state_;
  std::string last_selected_;
  std::optional<double> last_status_time_;
  std::vector<int32_t> previous_buttons_;
  TimedMessage<Drive> teleop_;
  TimedMessage<Drive> nav2_;
  TimedMessage<Drive> behavior_cmd_;
  TimedMessage<std_msgs::msg::Bool> behavior_active_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<Drive>::SharedPtr teleop_sub_;
  rclcpp::Subscription<Drive>::SharedPtr drive_sub_;
  rclcpp::Subscription<Drive>::SharedPtr behavior_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr behavior_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr enable_sub_;
  rclcpp::Publisher<Drive>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlCenterNode>());
  rclcpp::shutdown();
  return 0;
}
