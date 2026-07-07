#include "carkit_scan_deskew/scan_deskew_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace carkit_scan_deskew
{

namespace
{

constexpr double kEpsilon = 1e-9;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

bool isValidRange(float range, float range_min, float range_max)
{
  return std::isfinite(range) &&
         range >= range_min &&
         range <= range_max;
}

}  // namespace

ScanDeskewNode::ScanDeskewNode(const rclcpp::NodeOptions & options)
: Node("scan_deskew_node", options)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  output_topic_ = declare_parameter<std::string>("output_topic", "/scan_deskewed");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  laser_frame_ = declare_parameter<std::string>("laser_frame", "");
  odom_buffer_duration_sec_ = declare_parameter<double>("odom_buffer_duration_sec", 2.0);
  transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.1);
  use_tf_extrinsic_ = declare_parameter<bool>("use_tf_extrinsic", true);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanDeskewNode::odomCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanDeskewNode::scanCallback, this, std::placeholders::_1));

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    output_topic_,
    rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(),
    "Scan deskew node started: %s -> %s using odom %s",
    scan_topic_.c_str(),
    output_topic_.c_str(),
    odom_topic_.c_str());
}

void ScanDeskewNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_buffer_.push_back(*msg);

  const rclcpp::Time cutoff = rclcpp::Time(msg->header.stamp) -
    rclcpp::Duration::from_seconds(odom_buffer_duration_sec_);
  while (
    !odom_buffer_.empty() &&
    rclcpp::Time(odom_buffer_.front().header.stamp) < cutoff)
  {
    odom_buffer_.pop_front();
  }
}

double ScanDeskewNode::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Pose2D ScanDeskewNode::interpolatePoses(
  const Pose2D & a,
  const Pose2D & b,
  double ratio)
{
  Pose2D out;
  out.x = a.x + ratio * (b.x - a.x);
  out.y = a.y + ratio * (b.y - a.y);
  const double delta_yaw = normalizeAngle(b.yaw - a.yaw);
  out.yaw = normalizeAngle(a.yaw + ratio * delta_yaw);
  return out;
}

bool ScanDeskewNode::lookupPose(
  const rclcpp::Time & stamp,
  Pose2D & pose,
  std::string & error) const
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (odom_buffer_.empty()) {
    error = "odometry buffer is empty";
    return false;
  }

  if (stamp < rclcpp::Time(odom_buffer_.front().header.stamp)) {
    const auto & first = odom_buffer_.front();
    pose.x = first.pose.pose.position.x;
    pose.y = first.pose.pose.position.y;
    pose.yaw = yawFromQuaternion(first.pose.pose.orientation);
    error = "requested stamp is older than odom buffer; using oldest odom";
    return true;
  }

  if (stamp >= rclcpp::Time(odom_buffer_.back().header.stamp)) {
    const auto & last = odom_buffer_.back();
    pose.x = last.pose.pose.position.x;
    pose.y = last.pose.pose.position.y;
    pose.yaw = yawFromQuaternion(last.pose.pose.orientation);
    error = "requested stamp is newer than odom buffer; using newest odom";
    return true;
  }

  for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
    const auto & prev = odom_buffer_[i - 1];
    const auto & next = odom_buffer_[i];
    const rclcpp::Time prev_stamp(prev.header.stamp);
    const rclcpp::Time next_stamp(next.header.stamp);
    if (stamp < next_stamp) {
      const double dt = (next_stamp - prev_stamp).seconds();
      const double ratio = dt > kEpsilon ?
        (stamp - prev_stamp).seconds() / dt :
        0.0;

      Pose2D prev_pose{
        prev.pose.pose.position.x,
        prev.pose.pose.position.y,
        yawFromQuaternion(prev.pose.pose.orientation)};
      Pose2D next_pose{
        next.pose.pose.position.x,
        next.pose.pose.position.y,
        yawFromQuaternion(next.pose.pose.orientation)};

      pose = interpolatePoses(prev_pose, next_pose, ratio);
      return true;
    }
  }

  error = "failed to interpolate odom pose";
  return false;
}

bool ScanDeskewNode::lookupLaserExtrinsic(
  const std::string & laser_frame,
  const rclcpp::Time & stamp,
  LaserExtrinsic & extrinsic)
{
  if (!use_tf_extrinsic_ || laser_frame.empty() || laser_frame == base_frame_) {
    extrinsic = LaserExtrinsic{};
    return true;
  }

  try {
    const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      base_frame_,
      laser_frame,
      stamp,
      rclcpp::Duration::from_seconds(transform_timeout_sec_));

    extrinsic.x = tf.transform.translation.x;
    extrinsic.y = tf.transform.translation.y;
    extrinsic.yaw = yawFromQuaternion(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Laser extrinsic lookup failed (%s -> %s): %s",
      base_frame_.c_str(),
      laser_frame.c_str(),
      ex.what());
    return false;
  }
}

void ScanDeskewNode::transformBasePoint(
  const Pose2D & from_pose,
  const Pose2D & to_pose,
  double px,
  double py,
  double & out_x,
  double & out_y)
{
  const double cos_from = std::cos(from_pose.yaw);
  const double sin_from = std::sin(from_pose.yaw);
  const double wx = from_pose.x + cos_from * px - sin_from * py;
  const double wy = from_pose.y + sin_from * px + cos_from * py;

  const double dx = wx - to_pose.x;
  const double dy = wy - to_pose.y;
  const double cos_to = std::cos(to_pose.yaw);
  const double sin_to = std::sin(to_pose.yaw);
  out_x = cos_to * dx + sin_to * dy;
  out_y = -sin_to * dx + cos_to * dy;
}

void ScanDeskewNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const std::size_t num_rays = msg->ranges.size();
  if (num_rays == 0) {
    return;
  }

  const std::string laser_frame = laser_frame_.empty() ?
    msg->header.frame_id :
    laser_frame_;

  const rclcpp::Time scan_stamp(msg->header.stamp);
  std::string error;
  Pose2D ref_pose{};
  if (!lookupPose(scan_stamp, ref_pose, error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping deskew: %s",
      error.c_str());
    scan_pub_->publish(*msg);
    return;
  }

  LaserExtrinsic extrinsic{};
  if (!lookupLaserExtrinsic(laser_frame, scan_stamp, extrinsic)) {
    scan_pub_->publish(*msg);
    return;
  }

  const double cos_bl = std::cos(extrinsic.yaw);
  const double sin_bl = std::sin(extrinsic.yaw);

  sensor_msgs::msg::LaserScan deskewed = *msg;
  deskewed.header.stamp = msg->header.stamp;
  deskewed.header.frame_id = laser_frame;

  const double time_increment = msg->time_increment;
  const double scan_time = msg->scan_time;
  const bool use_time_increment = time_increment > kEpsilon;
  const double ray_time_step = (!use_time_increment && num_rays > 1 && scan_time > kEpsilon) ?
    scan_time / static_cast<double>(num_rays - 1) :
    0.0;

  for (std::size_t i = 0; i < num_rays; ++i) {
    const float range = msg->ranges[i];
    if (!isValidRange(range, msg->range_min, msg->range_max)) {
      continue;
    }

    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double px_l = static_cast<double>(range) * std::cos(angle);
    const double py_l = static_cast<double>(range) * std::sin(angle);

    const double px_base = cos_bl * px_l - sin_bl * py_l + extrinsic.x;
    const double py_base = sin_bl * px_l + cos_bl * py_l + extrinsic.y;

    const double ray_offset = use_time_increment ?
      static_cast<double>(i) * time_increment :
      static_cast<double>(i) * ray_time_step;
    const rclcpp::Time ray_stamp = scan_stamp + rclcpp::Duration::from_seconds(ray_offset);

    Pose2D ray_pose{};
    if (!lookupPose(ray_stamp, ray_pose, error)) {
      continue;
    }

    double deskewed_x = 0.0;
    double deskewed_y = 0.0;
    transformBasePoint(ray_pose, ref_pose, px_base, py_base, deskewed_x, deskewed_y);

    const double lx = cos_bl * (deskewed_x - extrinsic.x) + sin_bl * (deskewed_y - extrinsic.y);
    const double ly = -sin_bl * (deskewed_x - extrinsic.x) + cos_bl * (deskewed_y - extrinsic.y);
    const double deskewed_range = std::hypot(lx, ly);

    if (deskewed_range >= msg->range_min && deskewed_range <= msg->range_max) {
      deskewed.ranges[i] = static_cast<float>(deskewed_range);
    }
  }

  scan_pub_->publish(deskewed);
}

}  // namespace carkit_scan_deskew

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carkit_scan_deskew::ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
