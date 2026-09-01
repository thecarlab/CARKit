#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "carkit_scan_filter/footprint_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
class ScanFootprintFilterNode : public rclcpp::Node
{
public:
  ScanFootprintFilterNode()
  : Node("scan_footprint_filter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan/raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan");
    vehicle_length_ = declare_parameter<double>("vehicle_length_m", 0.50);
    vehicle_width_ = declare_parameter<double>("vehicle_width_m", 0.25);
    padding_ = declare_parameter<double>("padding_m", 0.0);
    if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0) {
      throw std::invalid_argument("Vehicle length and width must be positive");
    }
    if (padding_ < 0.0) {
      throw std::invalid_argument("Footprint padding must not be negative");
    }

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, qos,
      std::bind(&ScanFootprintFilterNode::scan_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Filtering %s to %s with centered %.2f m x %.2f m footprint",
      input_topic.c_str(), output_topic.c_str(), vehicle_length_, vehicle_width_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    const bool geometry_changed =
      message->ranges.size() != beam_count_ ||
      message->angle_min != angle_min_ ||
      message->angle_increment != angle_increment_;
    if (geometry_changed) {
      beam_count_ = message->ranges.size();
      angle_min_ = message->angle_min;
      angle_increment_ = message->angle_increment;
      range_limits_ = carkit_scan_filter::footprint_range_limits(
        beam_count_, angle_min_, angle_increment_, vehicle_length_, vehicle_width_, padding_);
    }

    auto filtered = *message;
    carkit_scan_filter::apply_footprint_limits(filtered.ranges, range_limits_);
    publisher_->publish(filtered);
  }

  double vehicle_length_;
  double vehicle_width_;
  double padding_;
  std::size_t beam_count_{0};
  float angle_min_{0.0F};
  float angle_increment_{0.0F};
  std::vector<float> range_limits_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFootprintFilterNode>());
  rclcpp::shutdown();
  return 0;
}
