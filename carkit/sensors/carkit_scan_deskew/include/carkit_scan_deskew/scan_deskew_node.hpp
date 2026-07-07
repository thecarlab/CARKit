#ifndef CARKIT_SCAN_DESKEW__SCAN_DESKEW_NODE_HPP_
#define CARKIT_SCAN_DESKEW__SCAN_DESKEW_NODE_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace carkit_scan_deskew
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct LaserExtrinsic
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class ScanDeskewNode : public rclcpp::Node
{
public:
  explicit ScanDeskewNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  bool lookupLaserExtrinsic(
    const std::string & laser_frame,
    const rclcpp::Time & stamp,
    LaserExtrinsic & extrinsic);

  bool lookupPose(
    const rclcpp::Time & stamp,
    Pose2D & pose,
    std::string & error) const;

  static double normalizeAngle(double angle);
  static Pose2D interpolatePoses(
    const Pose2D & a,
    const Pose2D & b,
    double ratio);
  static void transformBasePoint(
    const Pose2D & from_pose,
    const Pose2D & to_pose,
    double px,
    double py,
    double & out_x,
    double & out_y);

  std::string scan_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  double odom_buffer_duration_sec_{2.0};
  double transform_timeout_sec_{0.1};
  bool use_tf_extrinsic_{true};

  mutable std::mutex odom_mutex_;
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace carkit_scan_deskew

#endif  // CARKIT_SCAN_DESKEW__SCAN_DESKEW_NODE_HPP_
