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
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "carkit_behavior/behavior_engine.hpp"
#include "carkit_behavior/path_geometry.hpp"
#include "carkit_perception_msgs/msg/yolo_detection2_d.hpp"
#include "carkit_perception_msgs/msg/yolo_detection2_d_array.hpp"
#include "carkit_perception_msgs/msg/yolo_traffic_light_detection2_d.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace cb = carkit_behavior;
using Drive = ackermann_msgs::msg::AckermannDriveStamped;
using Detection = carkit_perception_msgs::msg::YoloDetection2D;
using DetectionArray = carkit_perception_msgs::msg::YoloDetection2DArray;
using TrafficDetection = carkit_perception_msgs::msg::YoloTrafficLightDetection2D;

namespace
{
constexpr char kAutoDrive[] = "AUTO_DRIVE";
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}

/// Convert a quaternion orientation into planar yaw radians.
double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double finite_clamp(double value, double lower, double upper)
{
  return std::isfinite(value) ? std::clamp(value, lower, upper) : lower;
}

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

struct MapPoint
{
  double x{0.0};
  double y{0.0};
};

struct ObjectLocation
{
  double range_m{0.0};
  double scan_angle_rad{0.0};
  double x{0.0};
  double y{0.0};
};

struct WeightedTrack
{
  double x{0.0};
  double y{0.0};
  double weight{0.01};
  int observations{1};

  WeightedTrack(double input_x, double input_y, double confidence)
  : x(input_x), y(input_y), weight(std::max(0.01, confidence)) {}

  void update(double input_x, double input_y, double confidence)
  {
    const double input_weight = std::max(0.01, confidence);
    const double total = weight + input_weight;
    x = (x * weight + input_x * input_weight) / total;
    y = (y * weight + input_y * input_weight) / total;
    weight = total;
    ++observations;
  }
};

struct StopSignTrack : WeightedTrack
{
  using WeightedTrack::WeightedTrack;
  bool stopped{false};
  bool rearm_for_new_plan{false};
  std::optional<double> last_distance_m;
  std::optional<double> min_distance_m;
};

struct TrafficLightTrack : WeightedTrack
{
  TrafficLightTrack(double x, double y, double confidence, uint8_t input_color)
  : WeightedTrack(x, y, confidence), color(input_color) {}
  uint8_t color{TrafficDetection::TRAFFIC_LIGHT_UNKNOWN};
  std::optional<double> last_distance_m;
};

struct SpeedSignTrack : WeightedTrack
{
  using WeightedTrack::WeightedTrack;
  bool passed{false};
  std::optional<double> last_distance_m;
  std::optional<double> last_range_m;
  std::optional<double> min_range_m;
};

struct ConeTrack : WeightedTrack
{
  using WeightedTrack::WeightedTrack;
};

visualization_msgs::msg::MarkerArray object_markers(
  const std::string & frame, const rclcpp::Time & stamp, double x, double y,
  const std::string & object_namespace, const std::string & label,
  float red, float green, float blue)
{
  visualization_msgs::msg::MarkerArray output;
  visualization_msgs::msg::Marker symbol;
  symbol.header.frame_id = frame;
  symbol.header.stamp = stamp;
  symbol.ns = object_namespace;
  symbol.id = 0;
  symbol.type = visualization_msgs::msg::Marker::SPHERE;
  symbol.action = visualization_msgs::msg::Marker::ADD;
  symbol.pose.position.x = x;
  symbol.pose.position.y = y;
  symbol.pose.position.z = 0.22;
  symbol.pose.orientation.w = 1.0;
  symbol.scale.x = symbol.scale.y = symbol.scale.z = 0.45;
  symbol.color.r = red;
  symbol.color.g = green;
  symbol.color.b = blue;
  symbol.color.a = 1.0;
  symbol.frame_locked = true;
  output.markers.push_back(symbol);

  auto text = symbol;
  text.id = 1;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.position.z = 0.62;
  text.scale.x = text.scale.y = 0.0;
  text.scale.z = 0.25;
  text.color.r = text.color.g = text.color.b = 1.0;
  text.text = label;
  output.markers.push_back(text);
  return output;
}
}  // namespace

class BehaviorCenterNode : public rclcpp::Node
{
public:
  BehaviorCenterNode()
  : Node("behavior_center_node"), tf_buffer_(get_clock())
  {
    declare_parameters();
    load_parameters();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    main_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/control_center/main_state", 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        main_state_ = message->data;
        set_inputs_active(main_state_ == kAutoDrive);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        if (message->width > 0 && message->height > 0) {
          camera_image_width_ = message->width;
          camera_cx_ = message->k[2];
          camera_fx_ = message->k[0];
        }
      });

    state_pub_ = create_publisher<std_msgs::msg::String>("/behavior/state", 10);
    override_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/behavior/override_active", 10);
    override_cmd_pub_ = create_publisher<Drive>("/behavior/override_cmd", 10);
    stop_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/behavior/stop_sign_position", 10);
    light_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/behavior/traffic_light_position", 10);
    speed_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/behavior/speed_sign_position", 10);
    cone_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/behavior/cone_position", 10);
    const auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
    stop_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/behavior/stop_sign_markers", marker_qos);
    light_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/behavior/traffic_light_markers", marker_qos);
    speed_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/behavior/speed_sign_markers", marker_qos);
    cone_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/behavior/cone_markers", marker_qos);

    for (auto & rule : cb::build_behavior_rules(behavior_rule_names_)) {
      behavior_engine_.register_rule(std::move(rule));
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {on_timer();});

    std::ostringstream names;
    const auto active_rules = behavior_engine_.rule_names();
    for (std::size_t index = 0; index < active_rules.size(); ++index) {
      if (index) {names << ", ";}
      names << active_rules[index];
    }
    RCLCPP_INFO(
      get_logger(), "C++ behavior planner started with rules: %s", names.str().c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter("min_confidence", 0.55);
    declare_parameter("detection_timeout_sec", 0.5);
    declare_parameter("status_publish_rate_hz", 5.0);
    declare_parameter<std::vector<std::string>>(
      "behavior_rules", {"stop_sign", "traffic_light", "cone", "speed_sign"});
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter("scan_timeout_sec", 0.5);
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    declare_parameter("camera_horizontal_fov_deg", 69.4);
    declare_parameter("camera_to_scan_yaw_offset_rad", 0.0);
    declare_parameter("camera_forward_offset_m", 0.08);
    declare_parameter("camera_lateral_offset_m", 0.0);
    declare_parameter<std::string>("global_plan_topic", "/plan");
    declare_parameter<std::string>("nav2_drive_topic", "/drive");
    declare_parameter("stop_sign_stop_before_distance_m", 0.5);
    declare_parameter("stop_sign_stop_line_tolerance_m", 0.25);
    declare_parameter("stop_sign_rearm_distance_m", 1.0);
    declare_parameter("stop_sign_lidar_angle_window_deg", 8.0);
    declare_parameter("stop_sign_lidar_min_range_m", 0.15);
    declare_parameter("stop_sign_lidar_max_range_m", 10.0);
    declare_parameter("stop_sign_stop_duration_sec", 5.0);
    declare_parameter("stop_sign_cooldown_sec", 10.0);
    declare_parameter("stop_sign_min_confidence", 0.75);
    declare_parameter<std::string>("stop_sign_map_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter("stop_sign_required_observations", 3);
    declare_parameter("stop_sign_track_match_distance_m", 1.0);
    declare_parameter("stop_sign_clear_distance_m", 10.0);
    declare_parameter("traffic_light_min_confidence", 0.6);
    declare_parameter("traffic_light_lidar_angle_window_deg", 8.0);
    declare_parameter("traffic_light_lidar_min_range_m", 0.15);
    declare_parameter("traffic_light_lidar_max_range_m", 10.0);
    declare_parameter("traffic_light_stop_ahead_distance_m", 2.0);
    declare_parameter("traffic_light_required_observations", 3);
    declare_parameter("traffic_light_stop_required_frames", 3);
    declare_parameter("traffic_light_green_required_frames", 3);
    declare_parameter("traffic_light_track_match_distance_m", 1.0);
    declare_parameter("traffic_light_clear_distance_m", 10.0);
    declare_parameter("plan_goal_change_distance_m", 0.25);
    declare_parameter("speed_sign_min_confidence", 0.6);
    declare_parameter("speed_sign_lidar_angle_window_deg", 8.0);
    declare_parameter("speed_sign_lidar_min_range_m", 0.15);
    declare_parameter("speed_sign_lidar_max_range_m", 10.0);
    declare_parameter("speed_sign_required_observations", 2);
    declare_parameter("speed_sign_track_match_distance_m", 1.0);
    declare_parameter("speed_sign_clear_distance_m", 10.0);
    declare_parameter("speed_sign_pass_tolerance_m", 0.25);
    declare_parameter("speed_sign_pass_near_distance_m", 1.5);
    declare_parameter("speed_sign_pass_range_rearm_m", 0.4);
    declare_parameter("speed_sign_fallback_range_m", 3.0);
    declare_parameter("speed_sign_override_min_speed_mps", 1.0);
    declare_parameter("speed_sign_override_multiplier", 1.5);
    declare_parameter("speed_sign_override_max_speed_mps", 3.0);
    declare_parameter("speed_sign_override_duration_sec", 3.0);
    declare_parameter("cone_min_confidence", 0.6);
    declare_parameter("cone_lidar_angle_window_deg", 8.0);
    declare_parameter("cone_lidar_min_range_m", 0.15);
    declare_parameter("cone_lidar_max_range_m", 10.0);
    declare_parameter("cone_required_observations", 2);
    declare_parameter("cone_track_match_distance_m", 1.0);
    declare_parameter("cone_clear_distance_m", 10.0);
    declare_parameter("cone_override_speed_mps", 0.8);
    declare_parameter("cone_fallback_range_m", 2.0);
  }

  template<typename T>
  T parameter(const std::string & name) const
  {
    return get_parameter(name).get_value<T>();
  }

  void load_parameters()
  {
    min_confidence_ = parameter<double>("min_confidence");
    detection_timeout_ = parameter<double>("detection_timeout_sec");
    status_rate_ = std::max(0.1, parameter<double>("status_publish_rate_hz"));
    behavior_rule_names_ = parameter<std::vector<std::string>>("behavior_rules");
    scan_topic_ = parameter<std::string>("scan_topic");
    odom_topic_ = parameter<std::string>("odom_topic");
    camera_info_topic_ = parameter<std::string>("camera_info_topic");
    global_plan_topic_ = parameter<std::string>("global_plan_topic");
    nav2_drive_topic_ = parameter<std::string>("nav2_drive_topic");
    scan_timeout_ = parameter<double>("scan_timeout_sec");
    camera_fov_ = parameter<double>("camera_horizontal_fov_deg") * kPi / 180.0;
    camera_scan_yaw_ = parameter<double>("camera_to_scan_yaw_offset_rad");
    camera_forward_ = parameter<double>("camera_forward_offset_m");
    camera_lateral_ = parameter<double>("camera_lateral_offset_m");
    stop_before_ = parameter<double>("stop_sign_stop_before_distance_m");
    stop_tolerance_ = parameter<double>("stop_sign_stop_line_tolerance_m");
    stop_rearm_ = parameter<double>("stop_sign_rearm_distance_m");
    stop_window_ = parameter<double>("stop_sign_lidar_angle_window_deg") * kPi / 180.0;
    stop_min_range_ = parameter<double>("stop_sign_lidar_min_range_m");
    stop_max_range_ = parameter<double>("stop_sign_lidar_max_range_m");
    stop_duration_ = parameter<double>("stop_sign_stop_duration_sec");
    stop_confidence_ = parameter<double>("stop_sign_min_confidence");
    map_frame_ = parameter<std::string>("stop_sign_map_frame");
    robot_frame_ = parameter<std::string>("robot_base_frame");
    stop_required_ = parameter<int64_t>("stop_sign_required_observations");
    stop_match_ = parameter<double>("stop_sign_track_match_distance_m");
    stop_clear_ = parameter<double>("stop_sign_clear_distance_m");
    light_confidence_ = parameter<double>("traffic_light_min_confidence");
    light_window_ = parameter<double>("traffic_light_lidar_angle_window_deg") * kPi / 180.0;
    light_min_range_ = parameter<double>("traffic_light_lidar_min_range_m");
    light_max_range_ = parameter<double>("traffic_light_lidar_max_range_m");
    light_stop_ahead_ = parameter<double>("traffic_light_stop_ahead_distance_m");
    light_required_ = parameter<int64_t>("traffic_light_required_observations");
    light_stop_frames_required_ = parameter<int64_t>("traffic_light_stop_required_frames");
    light_green_frames_required_ = parameter<int64_t>("traffic_light_green_required_frames");
    light_match_ = parameter<double>("traffic_light_track_match_distance_m");
    light_clear_ = parameter<double>("traffic_light_clear_distance_m");
    goal_change_ = parameter<double>("plan_goal_change_distance_m");
    speed_confidence_ = parameter<double>("speed_sign_min_confidence");
    speed_window_ = parameter<double>("speed_sign_lidar_angle_window_deg") * kPi / 180.0;
    speed_min_range_ = parameter<double>("speed_sign_lidar_min_range_m");
    speed_max_range_ = parameter<double>("speed_sign_lidar_max_range_m");
    speed_required_ = parameter<int64_t>("speed_sign_required_observations");
    speed_match_ = parameter<double>("speed_sign_track_match_distance_m");
    speed_clear_ = parameter<double>("speed_sign_clear_distance_m");
    speed_pass_tolerance_ = parameter<double>("speed_sign_pass_tolerance_m");
    speed_pass_near_ = parameter<double>("speed_sign_pass_near_distance_m");
    speed_range_rearm_ = parameter<double>("speed_sign_pass_range_rearm_m");
    speed_fallback_range_ = parameter<double>("speed_sign_fallback_range_m");
    speed_override_min_ = parameter<double>("speed_sign_override_min_speed_mps");
    speed_multiplier_ = parameter<double>("speed_sign_override_multiplier");
    speed_override_max_ = parameter<double>("speed_sign_override_max_speed_mps");
    speed_override_duration_ = parameter<double>("speed_sign_override_duration_sec");
    cone_confidence_ = parameter<double>("cone_min_confidence");
    cone_window_ = parameter<double>("cone_lidar_angle_window_deg") * kPi / 180.0;
    cone_min_range_ = parameter<double>("cone_lidar_min_range_m");
    cone_max_range_ = parameter<double>("cone_lidar_max_range_m");
    cone_required_ = parameter<int64_t>("cone_required_observations");
    cone_match_ = parameter<double>("cone_track_match_distance_m");
    cone_clear_ = parameter<double>("cone_clear_distance_m");
    cone_override_speed_ = parameter<double>("cone_override_speed_mps");
    cone_fallback_range_ = parameter<double>("cone_fallback_range_m");
  }

  void set_inputs_active(bool active)
  {
    if (active == inputs_active_) {return;}
    if (active) {
      detection_sub_ = create_subscription<DetectionArray>(
        "/yolo/detections_2d", 10,
        [this](DetectionArray::ConstSharedPtr message) {
          latest_detections_ = message;
          latest_detection_time_ = seconds();
          ++detection_sequence_;
          stale_handled_ = false;
        });
      nav_drive_sub_ = create_subscription<Drive>(
        nav2_drive_topic_, 10,
        [this](Drive::ConstSharedPtr message) {latest_nav_drive_ = message;});
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
          latest_scan_ = message;
          latest_scan_time_ = seconds();
        });
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          velocity_ = message->twist.twist.linear.x;
          pose_frame_ = message->header.frame_id;
          robot_x_ = message->pose.pose.position.x;
          robot_y_ = message->pose.pose.position.y;
        });
      plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        global_plan_topic_, 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr message) {on_plan(std::move(message));});
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    } else {
      detection_sub_.reset();
      nav_drive_sub_.reset();
      scan_sub_.reset();
      odom_sub_.reset();
      plan_sub_.reset();
      tf_listener_.reset();
      latest_detections_.reset();
      latest_scan_.reset();
      latest_nav_drive_.reset();
    }
    inputs_active_ = active;
  }

  void on_plan(nav_msgs::msg::Path::ConstSharedPtr message)
  {
    latest_plan_ = std::move(message);
    if (latest_plan_->poses.empty()) {return;}
    const auto & position = latest_plan_->poses.back().pose.position;
    const MapPoint new_goal{position.x, position.y};
    if (!active_goal_) {
      active_goal_ = new_goal;
      return;
    }
    if (std::hypot(new_goal.x - active_goal_->x, new_goal.y - active_goal_->y) <= goal_change_) {
      return;
    }
    active_goal_ = new_goal;
    for (auto & track : stop_tracks_) {
      if (track.stopped) {track.rearm_for_new_plan = true;}
    }
  }

  void on_timer()
  {
    const double current_time = seconds();
    if (main_state_ != kAutoDrive) {
      publish_state(cb::kNormalNav2, false, current_time);
      last_behavior_state_ = cb::kNormalNav2;
      return;
    }
    update_world_model(current_time);
    cb::BehaviorContext context;
    context.stop_sign_triggered = [this]() {return stop_sign_triggered();};
    context.stop_sign_stop_duration_sec = stop_duration_;
    context.traffic_light_stop_active = [this]() {return traffic_light_stop_active();};
    context.cone_speed_override_active = [this]() {return cone_speed_override_active();};
    context.cone_override_speed_mps = cone_override_speed_;
    context.speed_sign_pass_triggered = [this]() {return speed_sign_pass_triggered();};
    context.speed_sign_override_speed = [this]() {return speed_sign_override_speed();};
    context.speed_sign_override_duration_sec = speed_override_duration_;
    publish_decision(behavior_engine_.evaluate(context, current_time), current_time);
  }

  void update_world_model(double current_time)
  {
    const auto detections = latest_detections_ &&
      current_time - latest_detection_time_ <= detection_timeout_ ? latest_detections_ : nullptr;
    const auto scan = latest_scan_ && current_time - latest_scan_time_ <= scan_timeout_ ?
      latest_scan_ : nullptr;
    if (detections && detection_sequence_ != processed_detection_sequence_) {
      processed_detection_sequence_ = detection_sequence_;
      stale_handled_ = false;
      update_traffic_light(*detections, scan.get());
      update_stop_sign(*detections, scan.get());
      update_speed_sign(*detections, scan.get());
      update_cone(*detections, scan.get());
    } else if (!detections && !stale_handled_) {
      clear_traffic_light();
      cone_visible_ = false;
      publish_tracks();
      stale_handled_ = true;
    }
  }

  const Detection * best_detection(
    const DetectionArray & message, const std::string & exact_name,
    double minimum_confidence, bool stop_substring = false) const
  {
    const Detection * best = nullptr;
    for (const auto & detection : message.detections) {
      const auto name = lowercase(detection.class_name);
      const bool matches = stop_substring ? name.find("stop") != std::string::npos :
        name == exact_name;
      if (matches && detection.confidence >= minimum_confidence &&
        (!best || detection.confidence > best->confidence))
      {
        best = &detection;
      }
    }
    return best;
  }

  std::optional<std::pair<Detection, uint8_t>> best_traffic_light(
    const DetectionArray & message) const
  {
    std::optional<std::pair<Detection, uint8_t>> best;
    for (const auto & traffic : message.traffic_lights) {
      if (traffic.detection.confidence >= light_confidence_ &&
        (!best || traffic.detection.confidence > best->first.confidence))
      {
        best = std::make_pair(traffic.detection, traffic.traffic_light_color);
      }
    }
    return best;
  }

  std::optional<double> detection_bearing(
    const Detection & detection, double image_width) const
  {
    const double center = (detection.bbox_x_min + detection.bbox_x_max) / 2.0;
    if (camera_fx_ && camera_cx_ && *camera_fx_ > 0.0) {
      return std::atan2(center - *camera_cx_, *camera_fx_);
    }
    const double width = image_width > 0.0 ? image_width : camera_image_width_.value_or(0.0);
    if (width <= 0.0) {return std::nullopt;}
    return (center / width - 0.5) * camera_fov_;
  }

  std::optional<ObjectLocation> localize(
    const Detection & detection, double image_width, const sensor_msgs::msg::LaserScan & scan,
    double window, double min_range, double max_range,
    std::optional<double> fallback = std::nullopt) const
  {
    const auto camera_bearing = detection_bearing(detection, image_width);
    if (!camera_bearing) {return std::nullopt;}
    const double target_angle = normalize_angle(kPi - *camera_bearing + camera_scan_yaw_);
    const double front_angle = kPi + camera_scan_yaw_;
    const double target_camera_bearing = normalize_angle(front_angle - target_angle);
    const double camera_x = camera_forward_ * std::cos(front_angle) +
      camera_lateral_ * std::cos(front_angle + kPi / 2.0);
    const double camera_y = camera_forward_ * std::sin(front_angle) +
      camera_lateral_ * std::sin(front_angle + kPi / 2.0);
    std::optional<std::pair<double, double>> best;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max ||
        range < min_range || range > max_range)
      {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const double point_x = range * std::cos(angle);
      const double point_y = range * std::sin(angle);
      const double from_camera = std::atan2(point_y - camera_y, point_x - camera_x);
      const double candidate_bearing = normalize_angle(front_angle - from_camera);
      if (std::abs(normalize_angle(candidate_bearing - target_camera_bearing)) > window / 2.0) {
        continue;
      }
      if (!best || range < best->first) {
        best = std::pair<double, double>{range, angle};
      }
    }
    double range;
    double angle;
    if (best) {
      std::tie(range, angle) = *best;
    } else if (fallback) {
      range = finite_clamp(*fallback, min_range, max_range);
      angle = target_angle;
    } else {
      return std::nullopt;
    }
    return ObjectLocation{range, angle, range * std::cos(angle), range * std::sin(angle)};
  }

  std::optional<MapPoint> transform_to_map(
    const ObjectLocation & location, const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (source_frame == map_frame_) {return MapPoint{location.x, location.y};}
    auto transform = lookup_transform(map_frame_, source_frame, rclcpp::Time(stamp));
    if (!transform) {return std::nullopt;}
    const auto & translation = transform->transform.translation;
    const auto & rotation = transform->transform.rotation;
    const double yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    return MapPoint{
      translation.x + std::cos(yaw) * location.x - std::sin(yaw) * location.y,
      translation.y + std::sin(yaw) * location.x + std::cos(yaw) * location.y};
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup_transform(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp)
  {
    if (target.empty() || source.empty() || !tf_listener_) {return std::nullopt;}
    try {
      return tf_buffer_.lookupTransform(target, source, stamp);
    } catch (const tf2::TransformException &) {
      try {
        return tf_buffer_.lookupTransform(target, source, rclcpp::Time(0));
      } catch (const tf2::TransformException &) {
        return std::nullopt;
      }
    }
  }

  void update_stop_sign(const DetectionArray & detections, const sensor_msgs::msg::LaserScan * scan)
  {
    if (!scan) {publish_stop_track(); return;}
    const auto * detection = best_detection(detections, "", stop_confidence_, true);
    if (!detection) {publish_stop_track(); return;}
    const auto location = localize(
      *detection, detections.image_width, *scan, stop_window_, stop_min_range_, stop_max_range_);
    if (!location) {publish_stop_track(); return;}
    const auto point = transform_to_map(
      *location, scan->header.frame_id.empty() ? "laser" : scan->header.frame_id,
      scan->header.stamp);
    if (!point) {publish_stop_track(); return;}
    record_track(
      stop_tracks_, *point, detection->confidence, stop_required_, stop_match_, stop_clear_);
    publish_stop_track();
  }

  void update_traffic_light(
    const DetectionArray & detections, const sensor_msgs::msg::LaserScan * scan)
  {
    const auto candidate = best_traffic_light(detections);
    if (!scan || !candidate) {clear_traffic_light(); return;}
    update_light_color(candidate->second);
    const auto location = localize(
      candidate->first, detections.image_width, *scan, light_window_,
      light_min_range_, light_max_range_);
    if (!location) {clear_traffic_light(); return;}
    const auto point = transform_to_map(
      *location, scan->header.frame_id.empty() ? "laser" : scan->header.frame_id,
      scan->header.stamp);
    if (!point) {clear_traffic_light(); return;}
    record_light_track(*point, candidate->first.confidence, candidate->second);
    publish_light_track();
  }

  void update_speed_sign(
    const DetectionArray & detections,
    const sensor_msgs::msg::LaserScan * scan)
  {
    if (!scan) {publish_speed_track(); return;}
    const auto * detection = best_detection(detections, "speed_sign", speed_confidence_);
    if (!detection) {publish_speed_track(); return;}
    const auto location = localize(
      *detection, detections.image_width, *scan, speed_window_, speed_min_range_,
      speed_max_range_, speed_fallback_range_);
    if (!location) {publish_speed_track(); return;}
    const auto point = transform_to_map(
      *location, scan->header.frame_id.empty() ? "laser" : scan->header.frame_id,
      scan->header.stamp);
    if (!point) {publish_speed_track(); return;}
    auto & track = record_track(
      speed_tracks_, *point, detection->confidence, speed_required_, speed_match_, speed_clear_);
    track.last_range_m = location->range_m;
    track.min_range_m = track.min_range_m ? std::min(*track.min_range_m, location->range_m) :
      location->range_m;
    publish_speed_track();
  }

  void update_cone(const DetectionArray & detections, const sensor_msgs::msg::LaserScan * scan)
  {
    if (!scan) {cone_visible_ = false; publish_cone_track(); return;}
    const auto * detection = best_detection(detections, "traffic_cone", cone_confidence_);
    if (!detection) {cone_visible_ = false; publish_cone_track(); return;}
    cone_visible_ = true;
    const auto location = localize(
      *detection, detections.image_width, *scan, cone_window_, cone_min_range_,
      cone_max_range_, cone_fallback_range_);
    if (!location) {publish_cone_track(); return;}
    const auto point = transform_to_map(
      *location, scan->header.frame_id.empty() ? "laser" : scan->header.frame_id,
      scan->header.stamp);
    if (!point) {publish_cone_track(); return;}
    record_track(
      cone_tracks_, *point, detection->confidence, cone_required_, cone_match_, cone_clear_);
    publish_cone_track();
  }

  template<typename TrackT>
  TrackT & record_track(
    std::vector<TrackT> & tracks, const MapPoint & point, double confidence,
    int64_t required_observations, double match_distance, double clear_distance)
  {
    TrackT * reliable = reliable_track(tracks, required_observations);
    if (reliable) {
      const double distance = std::hypot(reliable->x - point.x, reliable->y - point.y);
      if (distance > clear_distance) {
        tracks.clear();
      } else {
        if (distance <= match_distance) {reliable->update(point.x, point.y, confidence);}
        return *reliable;
      }
    }
    auto nearest = tracks.end();
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (auto iterator = tracks.begin(); iterator != tracks.end(); ++iterator) {
      const double distance = std::hypot(iterator->x - point.x, iterator->y - point.y);
      if (distance < nearest_distance) {nearest = iterator; nearest_distance = distance;}
    }
    if (nearest != tracks.end() && nearest_distance <= match_distance) {
      nearest->update(point.x, point.y, confidence);
      return *nearest;
    }
    tracks.emplace_back(point.x, point.y, confidence);
    return tracks.back();
  }

  void record_light_track(const MapPoint & point, double confidence, uint8_t color)
  {
    TrafficLightTrack * track = primary_light_track();
    if (track) {
      const double distance = std::hypot(track->x - point.x, track->y - point.y);
      if (distance > light_clear_) {
        light_tracks_.clear();
      } else {
        if (distance <= light_match_) {track->update(point.x, point.y, confidence);}
        track->color = color;
        return;
      }
    }
    light_tracks_.emplace_back(point.x, point.y, confidence, color);
  }

  template<typename TrackT>
  TrackT * reliable_track(std::vector<TrackT> & tracks, int64_t required)
  {
    const int threshold = static_cast<int>(std::max<int64_t>(1, required));
    for (auto & track : tracks) {
      if (track.observations >= threshold) {return &track;}
    }
    return nullptr;
  }

  StopSignTrack * reliable_stop_track() {return reliable_track(stop_tracks_, stop_required_);}
  TrafficLightTrack * primary_light_track() {return reliable_track(light_tracks_, light_required_);}
  SpeedSignTrack * reliable_speed_track() {return reliable_track(speed_tracks_, speed_required_);}
  ConeTrack * reliable_cone_track() {return reliable_track(cone_tracks_, cone_required_);}

  void update_light_color(uint8_t color)
  {
    latest_light_color_ = color;
    if (color == TrafficDetection::TRAFFIC_LIGHT_RED ||
      color == TrafficDetection::TRAFFIC_LIGHT_YELLOW)
    {
      ++light_stop_frames_;
      light_green_frames_ = 0;
    } else if (color == TrafficDetection::TRAFFIC_LIGHT_GREEN) {
      ++light_green_frames_;
      light_stop_frames_ = 0;
    } else {
      light_stop_frames_ = 0;
      light_green_frames_ = 0;
    }
  }

  void clear_traffic_light()
  {
    if (light_stop_engaged_) {return;}
    const bool had_track = !light_tracks_.empty();
    light_tracks_.clear();
    latest_light_color_ = TrafficDetection::TRAFFIC_LIGHT_UNKNOWN;
    light_stop_frames_ = 0;
    light_green_frames_ = 0;
    if (had_track) {
      visualization_msgs::msg::MarkerArray output;
      for (int id = 0; id < 2; ++id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = now();
        marker.ns = "traffic_light";
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        output.markers.push_back(marker);
      }
      light_markers_pub_->publish(output);
    }
  }

  std::optional<MapPoint> robot_position()
  {
    if (auto transform = lookup_transform(map_frame_, robot_frame_, now())) {
      return MapPoint{transform->transform.translation.x, transform->transform.translation.y};
    }
    if (pose_frame_ == map_frame_ && robot_x_ && robot_y_) {
      return MapPoint{*robot_x_, *robot_y_};
    }
    return std::nullopt;
  }

  std::optional<double> path_distance(const MapPoint & robot, const MapPoint & object) const
  {
    if (!latest_plan_ || latest_plan_->header.frame_id != map_frame_) {return std::nullopt;}
    std::vector<cb::Point2d> points;
    points.reserve(latest_plan_->poses.size());
    for (const auto & pose : latest_plan_->poses) {
      points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    const auto robot_distance = cb::distance_along_path({robot.x, robot.y}, points);
    const auto object_distance = cb::distance_along_path({object.x, object.y}, points);
    if (!robot_distance || !object_distance) {return std::nullopt;}
    return *object_distance - *robot_distance;
  }

  bool stop_sign_triggered()
  {
    auto * track = reliable_stop_track();
    const auto robot = robot_position();
    if (!track || !robot) {return false;}
    const auto remaining = path_distance(*robot, {track->x, track->y});
    if (!remaining) {return false;}
    if (track->stopped) {
      if (track->rearm_for_new_plan && *remaining > stop_before_ + stop_rearm_) {
        track->stopped = false;
        track->rearm_for_new_plan = false;
        track->last_distance_m = *remaining;
        track->min_distance_m = *remaining;
      }
      return false;
    }
    const auto previous = track->last_distance_m;
    track->last_distance_m = *remaining;
    track->min_distance_m = track->min_distance_m ? std::min(*track->min_distance_m, *remaining) :
      *remaining;
    const bool in_region = cb::should_stop_before_line(*remaining, stop_before_, stop_tolerance_);
    const bool crossed = previous && *previous > stop_before_ && *remaining < -stop_tolerance_;
    if (in_region || crossed) {
      track->stopped = true;
      track->rearm_for_new_plan = false;
      return true;
    }
    return false;
  }

  bool traffic_light_stop_active()
  {
    auto * track = primary_light_track();
    const auto robot = robot_position();
    if (!track || !robot) {return light_stop_engaged_;}
    const auto remaining = path_distance(*robot, {track->x, track->y});
    if (!remaining) {return light_stop_engaged_;}
    track->last_distance_m = *remaining;
    const bool green = latest_light_color_ == TrafficDetection::TRAFFIC_LIGHT_GREEN &&
      light_green_frames_ >= std::max<int64_t>(1, light_green_frames_required_);
    if (green) {light_stop_engaged_ = false; return false;}
    if (light_stop_engaged_) {return true;}
    const bool stop_color =
      (latest_light_color_ == TrafficDetection::TRAFFIC_LIGHT_RED ||
      latest_light_color_ == TrafficDetection::TRAFFIC_LIGHT_YELLOW) &&
      light_stop_frames_ >= std::max<int64_t>(1, light_stop_frames_required_);
    if (stop_color && *remaining >= 0.0 && *remaining <= light_stop_ahead_) {
      light_stop_engaged_ = true;
      return true;
    }
    return false;
  }

  std::optional<double> current_speed() const
  {
    if (velocity_) {return std::abs(*velocity_);}
    if (latest_nav_drive_) {return std::abs(latest_nav_drive_->drive.speed);}
    return std::nullopt;
  }

  bool cone_speed_override_active()
  {
    const auto speed = current_speed();
    return cone_visible_ && reliable_cone_track() && speed && *speed > cone_override_speed_;
  }

  std::optional<double> speed_sign_override_speed() const
  {
    const auto speed = current_speed();
    if (!speed || *speed <= speed_override_min_) {return std::nullopt;}
    return std::min(*speed * speed_multiplier_, speed_override_max_);
  }

  bool speed_sign_pass_triggered()
  {
    auto * track = reliable_speed_track();
    if (!track || track->passed) {return false;}
    const auto robot = robot_position();
    const auto remaining = robot ? path_distance(*robot, {track->x, track->y}) : std::nullopt;
    const auto previous = track->last_distance_m;
    if (remaining) {track->last_distance_m = *remaining;}
    const bool passed_on_path = remaining && *remaining < -speed_pass_tolerance_;
    const bool crossed = remaining && previous && *previous > 0.0 &&
      *remaining < -speed_pass_tolerance_;
    const bool passed_by_range = track->last_range_m && track->min_range_m &&
      *track->min_range_m <= speed_pass_near_ &&
      *track->last_range_m >= *track->min_range_m + speed_range_rearm_ - 1.0e-9;
    if (passed_on_path || crossed || passed_by_range) {
      track->passed = true;
      return true;
    }
    return false;
  }

  void publish_point(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
    double x, double y)
  {
    geometry_msgs::msg::PointStamped message;
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    message.point.x = x;
    message.point.y = y;
    publisher->publish(message);
  }

  void publish_stop_track()
  {
    if (auto * track = reliable_stop_track()) {
      publish_point(stop_position_pub_, track->x, track->y);
      stop_markers_pub_->publish(
        object_markers(
          map_frame_, now(), track->x, track->y, "stop_sign", "STOP SIGN", 1.0F, 0.05F, 0.05F));
    }
  }

  void publish_light_track()
  {
    auto * track = primary_light_track();
    if (!track) {return;}
    publish_point(light_position_pub_, track->x, track->y);
    float red = 0.0F, green = 0.35F, blue = 1.0F;
    std::string color = "UNKNOWN";
    if (track->color == TrafficDetection::TRAFFIC_LIGHT_RED) {
      red = 1.0F; green = 0.05F; blue = 0.05F; color = "RED";
    } else if (track->color == TrafficDetection::TRAFFIC_LIGHT_YELLOW) {
      red = 1.0F; green = 0.75F; blue = 0.0F; color = "YELLOW";
    } else if (track->color == TrafficDetection::TRAFFIC_LIGHT_GREEN) {
      red = 0.05F; green = 1.0F; blue = 0.15F; color = "GREEN";
    }
    light_markers_pub_->publish(
      object_markers(
        map_frame_, now(), track->x, track->y, "traffic_light",
        "TRAFFIC LIGHT (" + color + ")", red, green, blue));
  }

  void publish_speed_track()
  {
    if (auto * track = reliable_speed_track()) {
      publish_point(speed_position_pub_, track->x, track->y);
      speed_markers_pub_->publish(
        object_markers(
          map_frame_, now(), track->x, track->y, "speed_sign", "SPEED SIGN",
          0.1F, 0.55F, 1.0F));
    }
  }

  void publish_cone_track()
  {
    if (auto * track = reliable_cone_track()) {
      publish_point(cone_position_pub_, track->x, track->y);
      cone_markers_pub_->publish(
        object_markers(
          map_frame_, now(), track->x, track->y, "cone", "CONE", 1.0F, 0.55F, 0.05F));
    }
  }

  void publish_tracks()
  {
    publish_stop_track();
    publish_speed_track();
    publish_cone_track();
  }

  void publish_decision(const cb::BehaviorDecision & decision, double current_time)
  {
    if (decision.state != last_behavior_state_) {
      RCLCPP_INFO(
        get_logger(), "[BEHAVIOR] %s -> %s (%s)", last_behavior_state_.c_str(),
        decision.state.c_str(), decision.reason.c_str());
      last_behavior_state_ = decision.state;
    }
    publish_state(decision.state, decision.override_active(), current_time);
    if (!decision.override_active()) {return;}
    Drive command;
    command.header.stamp = now();
    command.drive.speed = decision.target_speed_mps.value_or(0.0);
    command.drive.steering_angle = decision.target_speed_mps && latest_nav_drive_ ?
      latest_nav_drive_->drive.steering_angle : 0.0;
    override_cmd_pub_->publish(command);
  }

  void publish_state(const std::string & state, bool active, double current_time)
  {
    const bool changed = state != last_published_state_ ||
      !last_published_active_ || active != *last_published_active_;
    const bool heartbeat = !last_state_publish_time_ ||
      current_time - *last_state_publish_time_ >= 1.0 / status_rate_;
    if (!changed && !heartbeat) {return;}
    std_msgs::msg::String state_message;
    state_message.data = state;
    state_pub_->publish(state_message);
    std_msgs::msg::Bool active_message;
    active_message.data = active;
    override_active_pub_->publish(active_message);
    last_published_state_ = state;
    last_published_active_ = active;
    last_state_publish_time_ = current_time;
  }

  double seconds() {return get_clock()->now().seconds();}

  double min_confidence_{0.55};
  double detection_timeout_{0.5}, scan_timeout_{0.5}, status_rate_{5.0};
  double camera_fov_{1.2}, camera_scan_yaw_{0.0}, camera_forward_{0.08}, camera_lateral_{0.0};
  double stop_before_{0.5}, stop_tolerance_{0.25}, stop_rearm_{1.0};
  double stop_window_{0.14}, stop_min_range_{0.15}, stop_max_range_{10.0};
  double stop_duration_{5.0}, stop_confidence_{0.75}, stop_match_{1.0}, stop_clear_{10.0};
  double light_confidence_{0.6}, light_window_{0.14}, light_min_range_{0.15};
  double light_max_range_{10.0}, light_stop_ahead_{2.0}, light_match_{1.0}, light_clear_{10.0};
  double goal_change_{0.25};
  double speed_confidence_{0.6}, speed_window_{0.14}, speed_min_range_{0.15};
  double speed_max_range_{10.0}, speed_match_{1.0}, speed_clear_{10.0};
  double speed_pass_tolerance_{0.25}, speed_pass_near_{1.5}, speed_range_rearm_{0.4};
  double speed_fallback_range_{3.0}, speed_override_min_{1.0}, speed_multiplier_{1.5};
  double speed_override_max_{3.0}, speed_override_duration_{3.0};
  double cone_confidence_{0.6}, cone_window_{0.14}, cone_min_range_{0.15};
  double cone_max_range_{10.0}, cone_match_{1.0}, cone_clear_{10.0};
  double cone_override_speed_{0.8}, cone_fallback_range_{2.0};
  int64_t stop_required_{3}, light_required_{3}, light_stop_frames_required_{3};
  int64_t light_green_frames_required_{3}, speed_required_{2}, cone_required_{2};
  std::string scan_topic_, odom_topic_, camera_info_topic_, global_plan_topic_, nav2_drive_topic_;
  std::string map_frame_, robot_frame_, main_state_, pose_frame_;
  std::vector<std::string> behavior_rule_names_;
  bool inputs_active_{false}, stale_handled_{true}, cone_visible_{false};
  bool light_stop_engaged_{false};
  uint8_t latest_light_color_{TrafficDetection::TRAFFIC_LIGHT_UNKNOWN};
  int64_t light_stop_frames_{0}, light_green_frames_{0};
  uint64_t detection_sequence_{0}, processed_detection_sequence_{0};
  double latest_detection_time_{0.0}, latest_scan_time_{0.0};
  std::optional<double> camera_image_width_, camera_cx_, camera_fx_;
  std::optional<double> velocity_, robot_x_, robot_y_;
  std::optional<MapPoint> active_goal_;
  DetectionArray::ConstSharedPtr latest_detections_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  Drive::ConstSharedPtr latest_nav_drive_;
  nav_msgs::msg::Path::ConstSharedPtr latest_plan_;
  std::vector<StopSignTrack> stop_tracks_;
  std::vector<TrafficLightTrack> light_tracks_;
  std::vector<SpeedSignTrack> speed_tracks_;
  std::vector<ConeTrack> cone_tracks_;
  cb::BehaviorEngine behavior_engine_;
  std::string last_behavior_state_{cb::kNormalNav2}, last_published_state_;
  std::optional<bool> last_published_active_;
  std::optional<double> last_state_publish_time_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr main_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<Drive>::SharedPtr nav_drive_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_active_pub_;
  rclcpp::Publisher<Drive>::SharedPtr override_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr stop_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr light_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr speed_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cone_position_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stop_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr light_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr speed_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorCenterNode>());
  rclcpp::shutdown();
  return 0;
}
