#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "carkit_scan_filter/footprint_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
    center_x_ = declare_parameter<double>("footprint_center_x_m", 0.0);
    center_y_ = declare_parameter<double>("footprint_center_y_m", 0.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    if (!std::isfinite(vehicle_length_) || !std::isfinite(vehicle_width_) ||
      vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0)
    {
      throw std::invalid_argument("Vehicle length and width must be positive");
    }
    if (!std::isfinite(padding_) || padding_ < 0.0 ||
      !std::isfinite(center_x_) || !std::isfinite(center_y_))
    {
      throw std::invalid_argument("Footprint padding must not be negative");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, qos,
      std::bind(&ScanFootprintFilterNode::scan_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Filtering %s to %s in base frame with %.2f m x %.2f m footprint",
      input_topic.c_str(), output_topic.c_str(), vehicle_length_, vehicle_width_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    carkit_scan_filter::ScanProjection projection;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp));
      const auto & q = transform.transform.rotation;
      const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
      projection = {transform.transform.translation.x, transform.transform.translation.y,
        rotation[0][0], rotation[0][1], rotation[1][0], rotation[1][1]};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for laser-to-base transform: %s", error.what());
      return;  // Never clear obstacles using an assumed sensor origin.
    }

    auto filtered = *message;
    carkit_scan_filter::filter_in_base_frame(
      filtered.ranges, filtered.angle_min, filtered.angle_increment, projection,
      vehicle_length_, vehicle_width_, center_x_, center_y_, padding_);
    publisher_->publish(filtered);
  }

  double vehicle_length_;
  double vehicle_width_;
  double padding_;
  double center_x_, center_y_;
  std::string base_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
