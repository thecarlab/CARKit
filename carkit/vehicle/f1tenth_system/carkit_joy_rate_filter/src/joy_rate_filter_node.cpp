#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace carkit_joy_rate_filter
{

class JoyRateFilterNode : public rclcpp::Node
{
public:
  JoyRateFilterNode()
  : Node("joy_rate_filter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "joy_device");
    const auto output_topic = declare_parameter<std::string>("output_topic", "joy");
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.25);

    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    if (input_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("input_timeout_sec must be positive");
    }

    auto joy_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    joy_qos.reliable();
    joy_qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::Joy>(output_topic, joy_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      input_topic,
      joy_qos,
      std::bind(&JoyRateFilterNode::joy_callback, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      publish_period,
      std::bind(&JoyRateFilterNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Filtering %s to %s at %.1f Hz with reliable QoS (input timeout %.2f s)",
      input_topic.c_str(), output_topic.c_str(), publish_rate_, input_timeout_sec_);
  }

private:
  using Joy = sensor_msgs::msg::Joy;
  using SteadyClock = std::chrono::steady_clock;

  void joy_callback(const Joy::ConstSharedPtr msg)
  {
    bool publish_immediately = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_immediately = !latest_message_ || msg->buttons != previous_buttons_;
      latest_message_ = msg;
      previous_buttons_ = msg->buttons;
      last_input_time_ = SteadyClock::now();
    }

    // Button transitions carry mode and emergency-stop edges. Publish them
    // immediately instead of waiting for the periodic axis update.
    if (publish_immediately) {
      publish_message(msg);
    }
  }

  void timer_callback()
  {
    Joy::ConstSharedPtr message;
    const auto now = SteadyClock::now();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_message_) {
        return;
      }

      const std::chrono::duration<double> input_age = now - last_input_time_;
      if (input_age.count() > input_timeout_sec_) {
        return;
      }

      message = latest_message_;
    }

    publish_message(message);
  }

  void publish_message(const Joy::ConstSharedPtr & message)
  {
    Joy output = *message;
    output.header.stamp = now();
    publisher_->publish(std::move(output));
  }

  double publish_rate_;
  double input_timeout_sec_;
  std::mutex mutex_;
  Joy::ConstSharedPtr latest_message_;
  std::vector<int32_t> previous_buttons_;
  SteadyClock::time_point last_input_time_;
  rclcpp::Publisher<Joy>::SharedPtr publisher_;
  rclcpp::Subscription<Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace carkit_joy_rate_filter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carkit_joy_rate_filter::JoyRateFilterNode>());
  rclcpp::shutdown();
  return 0;
}
