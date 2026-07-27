#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "carkit_perception_msgs/msg/perception_latency_trace.hpp"
#include "carkit_perception_msgs/msg/stop_latency_trace.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

std::string timestamp_for_filename()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif
  std::ostringstream output;
  output << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return output.str();
}

std::string iso_timestamp(
  const std::chrono::system_clock::time_point & timestamp)
{
  const std::time_t timestamp_time =
    std::chrono::system_clock::to_time_t(timestamp);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &timestamp_time);
#else
  localtime_r(&timestamp_time, &local_time);
#endif
  const auto milliseconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    timestamp.time_since_epoch()) % 1000;

  std::ostringstream output;
  output << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S")
         << '.' << std::setfill('0') << std::setw(3)
         << milliseconds.count();
  return output.str();
}

std::string default_output_path()
{
  const auto name =
    "pipeline_topic_rate_" + timestamp_for_filename() + ".csv";
  return (
    std::filesystem::path("log") / "topic_rate_monitor" / name).string();
}

std::string default_event_output_path(const std::string & rate_output_path)
{
  const std::filesystem::path rate_path(rate_output_path);
  const std::string extension =
    rate_path.has_extension() ? rate_path.extension().string() : ".csv";
  const std::string stem =
    rate_path.has_extension() ?
    rate_path.stem().string() : rate_path.filename().string();
  return (
    rate_path.parent_path() / (stem + "_events" + extension)).string();
}

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

std::string trimmed_lowercase(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(),
    [](unsigned char character) {
      return std::isspace(character);
    });
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(),
    [](unsigned char character) {
      return std::isspace(character);
    }).base();
  if (first >= last) {
    return "";
  }
  return lowercase(std::string(first, last));
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::string csv_text(const std::string & value)
{
  std::ostringstream output;
  output << '"';
  for (const char character : value) {
    if (character == '"') {
      output << "\"\"";
    } else if (character == '\n' || character == '\r') {
      output << ' ';
    } else {
      output << character;
    }
  }
  output << '"';
  return output.str();
}

}  // namespace

class PipelineRateMonitor : public rclcpp::Node
{
public:
  PipelineRateMonitor()
  : Node("pipeline_rate_monitor"),
    started_at_(std::chrono::steady_clock::now()),
    last_sample_at_(started_at_),
    qos_(rclcpp::KeepLast(1)),
    event_qos_(rclcpp::KeepLast(10)),
    latched_event_qos_(rclcpp::KeepLast(1))
  {
    declare_parameter<std::string>("output_path", "");
    declare_parameter<std::string>("event_output_path", "");
    declare_parameter<double>("interval_sec", 1.0);
    declare_parameter<double>("motion_start_speed_mps", 0.08);
    declare_parameter<double>("motion_start_hold_sec", 0.2);
    declare_parameter<double>("motion_stop_speed_mps", 0.05);
    declare_parameter<double>("motion_stop_hold_sec", 1.0);

    output_path_ = get_parameter("output_path").as_string();
    if (output_path_.empty()) {
      output_path_ = default_output_path();
    }
    event_output_path_ = get_parameter("event_output_path").as_string();
    if (event_output_path_.empty()) {
      event_output_path_ = default_event_output_path(output_path_);
    }
    interval_sec_ = get_parameter("interval_sec").as_double();
    if (interval_sec_ < 0.2) {
      throw std::invalid_argument("interval_sec must be at least 0.2 seconds");
    }
    motion_start_speed_mps_ =
      get_parameter("motion_start_speed_mps").as_double();
    motion_start_hold_sec_ =
      get_parameter("motion_start_hold_sec").as_double();
    motion_stop_speed_mps_ =
      get_parameter("motion_stop_speed_mps").as_double();
    motion_stop_hold_sec_ =
      get_parameter("motion_stop_hold_sec").as_double();
    validate_event_parameters();

    qos_.best_effort();
    qos_.durability_volatile();
    event_qos_.reliable();
    event_qos_.durability_volatile();
    latched_event_qos_.reliable();
    latched_event_qos_.transient_local();
    open_rate_csv();
    open_event_csv();
    create_subscriptions();

    const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(interval_sec_));
    sample_timer_ = create_wall_timer(
      interval, std::bind(&PipelineRateMonitor::write_sample, this));

    RCLCPP_INFO(
      get_logger(),
      "Recording %zu pipeline rates to %s every %.3f seconds; events to %s",
      signals_.size(), output_path_.c_str(), interval_sec_,
      event_output_path_.c_str());
  }

  ~PipelineRateMonitor() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
    if (event_csv_.is_open()) {
      event_csv_.flush();
      event_csv_.close();
    }
  }

private:
  struct Signal
  {
    std::string name;
    std::string topic;
    uint64_t interval_count = 0;
    uint64_t total_count = 0;
    std::optional<std::chrono::steady_clock::time_point> last_message_at;
  };

  void validate_event_parameters() const
  {
    if (
      motion_start_speed_mps_ <= 0.0 ||
      motion_start_hold_sec_ < 0.0 ||
      motion_stop_speed_mps_ < 0.0 ||
      motion_stop_hold_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "event speed thresholds and hold times must be non-negative");
    }
    if (motion_stop_speed_mps_ >= motion_start_speed_mps_) {
      throw std::invalid_argument(
              "motion_stop_speed_mps must be lower than "
              "motion_start_speed_mps");
    }
  }

  void open_rate_csv()
  {
    const std::filesystem::path path(output_path_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error(
              "Cannot open pipeline rate CSV: " + output_path_);
    }
    csv_ << "timestamp,elapsed_s,sample,signal,source_topic,interval_s,"
            "message_count,hz,total_count,last_message_age_s\n";
    csv_.flush();
  }

  void open_event_csv()
  {
    const std::filesystem::path path(event_output_path_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    event_csv_.open(path, std::ios::out | std::ios::trunc);
    if (!event_csv_.is_open()) {
      throw std::runtime_error(
              "Cannot open pipeline event CSV: " + event_output_path_);
    }
    event_csv_ <<
      "timestamp,elapsed_s,event_id,event,source_topic,"
      "auto_session,route_session,details\n";
    event_csv_.flush();
  }

  template<typename MessageT>
  void subscribe(const std::string & signal_name, const std::string & topic)
  {
    subscribe_with_callback<MessageT>(
      signal_name, topic, qos_, [](const MessageT &) {});
  }

  template<typename MessageT, typename CallbackT>
  void subscribe_with_callback(
    const std::string & signal_name,
    const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT callback)
  {
    const size_t index = signals_.size();
    signals_.push_back(Signal{signal_name, topic});
    auto subscription = create_subscription<MessageT>(
      topic, qos,
      [this, index, callback](typename MessageT::ConstSharedPtr message) {
        record_message(index);
        callback(*message);
      });
    subscriptions_.push_back(subscription);
  }

  template<typename MessageT, typename CallbackT>
  void subscribe_event(
    const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT callback)
  {
    auto subscription = create_subscription<MessageT>(
      topic, qos,
      [callback](typename MessageT::ConstSharedPtr message) {
        callback(*message);
      });
    subscriptions_.push_back(subscription);
  }

  void create_subscriptions()
  {
    using AckermannDrive = ackermann_msgs::msg::AckermannDriveStamped;
    using Empty = std_msgs::msg::Empty;
    using PerceptionTrace =
      carkit_perception_msgs::msg::PerceptionLatencyTrace;
    using StopTrace = carkit_perception_msgs::msg::StopLatencyTrace;

    subscribe<Empty>("image_raw_rx", "/monitor/rate/image_raw_rx");
    subscribe<Empty>(
      "behavior_detection_rx", "/monitor/rate/behavior_detection_rx");
    subscribe<Empty>("behavior_scan_rx", "/monitor/rate/behavior_scan_rx");
    subscribe<Empty>("behavior_plan_rx", "/monitor/rate/behavior_plan_rx");
    subscribe<PerceptionTrace>(
      "yolo_inference_done", "/perception/latency_trace");
    subscribe<geometry_msgs::msg::Twist>("nav2_cmd_vel", "/cmd_vel");
    subscribe<AckermannDrive>("nav2_drive", "/drive");
    subscribe_with_callback<std_msgs::msg::String>(
      "behavior_state", "/behavior/state", qos_,
      [this](const std_msgs::msg::String & message) {
        handle_behavior_state(message);
      });
    subscribe<std_msgs::msg::Bool>(
      "behavior_override_active", "/behavior/override_active");
    subscribe<AckermannDrive>(
      "behavior_override_cmd", "/behavior/override_cmd");
    subscribe<std_msgs::msg::String>(
      "control_selected_cmd", "/control_center/selected_cmd");
    subscribe<AckermannDrive>(
      "control_ackermann_cmd", "/ackermann_cmd");
    subscribe_with_callback<nav_msgs::msg::Odometry>(
      "odom", "/odom", qos_,
      [this](const nav_msgs::msg::Odometry & message) {
        handle_odom(message);
      });
    subscribe<StopTrace>(
      "behavior_stop_trace", "/behavior/stop_latency_trace");

    subscribe_event<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", event_qos_,
      [this](
        const geometry_msgs::msg::PoseWithCovarianceStamped & message)
      {
        handle_initial_pose(message);
      });
    subscribe_event<std_msgs::msg::String>(
      "/control_center/main_state", latched_event_qos_,
      [this](const std_msgs::msg::String & message) {
        handle_main_state(message);
      });
    subscribe_event<std_msgs::msg::String>(
      "/foxglove/waypoints/command", event_qos_,
      [this](const std_msgs::msg::String & message) {
        handle_route_command(message);
      });
    subscribe_event<std_msgs::msg::String>(
      "/foxglove/waypoints/status", latched_event_qos_,
      [this](const std_msgs::msg::String & message) {
        handle_route_status(message);
      });
  }

  void record_message(size_t index)
  {
    Signal & signal = signals_.at(index);
    ++signal.interval_count;
    ++signal.total_count;
    signal.last_message_at = std::chrono::steady_clock::now();
  }

  void record_event(
    const std::string & event,
    const std::string & source_topic,
    const std::string & details)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto wall_now = std::chrono::system_clock::now();
    const double elapsed_s =
      std::chrono::duration<double>(steady_now - started_at_).count();
    ++event_number_;

    event_csv_ << iso_timestamp(wall_now) << ','
               << std::fixed << std::setprecision(6) << elapsed_s << ','
               << event_number_ << ','
               << csv_text(event) << ','
               << csv_text(source_topic) << ','
               << auto_session_ << ','
               << route_session_ << ','
               << csv_text(details) << '\n';
    event_csv_.flush();
    RCLCPP_INFO(
      get_logger(), "Pipeline event: %s | %s",
      event.c_str(), details.c_str());
  }

  void reset_route_observation()
  {
    route_requested_ = false;
    goal_accepted_ = false;
    route_completion_received_ = false;
    route_completed_recorded_ = false;
    route_motion_seen_ = false;
  }

  void reset_motion_observation()
  {
    vehicle_moving_ = false;
    motion_start_candidate_.reset();
    motion_stop_candidate_.reset();
  }

  void handle_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    std::ostringstream details;
    details << std::fixed << std::setprecision(6)
            << "frame=" << message.header.frame_id
            << " x=" << message.pose.pose.position.x
            << " y=" << message.pose.pose.position.y
            << " mode=" << (auto_drive_ ? "AUTO_DRIVE" : "non_auto");
    record_event("initial_pose_published", "/initialpose", details.str());
  }

  void handle_main_state(const std_msgs::msg::String & message)
  {
    const std::string new_state = message.data;
    if (new_state == main_state_) {
      return;
    }
    const std::string previous_state = main_state_;
    main_state_ = new_state;

    if (new_state == "AUTO_DRIVE" && !auto_drive_) {
      auto_drive_ = true;
      ++auto_session_;
      reset_route_observation();
      reset_motion_observation();
      last_behavior_state_.clear();
      record_event(
        "auto_drive_entered", "/control_center/main_state",
        "previous_state=" + previous_state);
      return;
    }

    if (auto_drive_ && new_state != "AUTO_DRIVE") {
      record_event(
        "auto_drive_exited", "/control_center/main_state",
        "new_state=" + new_state);
      auto_drive_ = false;
      reset_route_observation();
      reset_motion_observation();
    }
  }

  void handle_route_command(const std_msgs::msg::String & message)
  {
    const std::string command = trimmed_lowercase(message.data);
    if (command != "start" && command != "run" && command != "navigate") {
      return;
    }

    ++route_session_;
    reset_route_observation();
    route_requested_ = true;
  }

  void handle_route_status(const std_msgs::msg::String & message)
  {
    if (message.data == last_route_status_) {
      return;
    }
    last_route_status_ = message.data;
    const std::string status = trimmed_lowercase(message.data);

    if (starts_with(status, "navigating through")) {
      if (!route_requested_) {
        ++route_session_;
        reset_route_observation();
        route_requested_ = true;
      }
      goal_accepted_ = true;
      route_motion_seen_ = auto_drive_ && vehicle_moving_;
      record_event(
        "goal_accepted", "/foxglove/waypoints/status",
        "status=" + message.data);
      return;
    }

    if (starts_with(status, "route completed")) {
      route_completion_received_ = true;
      maybe_record_route_completed();
      return;
    }

    if (
      status.find("canceled") != std::string::npos ||
      status.find("aborted") != std::string::npos)
    {
      record_event(
        "navigation_failed", "/foxglove/waypoints/status",
        "status=" + message.data);
      goal_accepted_ = false;
      route_completion_received_ = false;
    }
  }

  void handle_behavior_state(const std_msgs::msg::String & message)
  {
    if (message.data == last_behavior_state_) {
      return;
    }
    const std::string previous_state = last_behavior_state_;
    last_behavior_state_ = message.data;
    if (!auto_drive_) {
      return;
    }
    record_event(
      "behavior_state_changed", "/behavior/state",
      "previous=" + previous_state + " current=" + message.data);
  }

  void handle_odom(const nav_msgs::msg::Odometry & message)
  {
    const double speed = std::abs(message.twist.twist.linear.x);
    last_odom_speed_mps_ = speed;
    if (!auto_drive_) {
      motion_start_candidate_.reset();
      motion_stop_candidate_.reset();
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!vehicle_moving_) {
      motion_stop_candidate_.reset();
      if (speed <= motion_start_speed_mps_) {
        motion_start_candidate_.reset();
        return;
      }
      if (!motion_start_candidate_) {
        motion_start_candidate_ = now;
        return;
      }
      const double held_s = std::chrono::duration<double>(
        now - *motion_start_candidate_).count();
      if (held_s < motion_start_hold_sec_) {
        return;
      }

      vehicle_moving_ = true;
      motion_start_candidate_.reset();
      route_motion_seen_ = route_motion_seen_ || goal_accepted_;
      return;
    }

    motion_start_candidate_.reset();
    if (speed >= motion_stop_speed_mps_) {
      motion_stop_candidate_.reset();
      return;
    }
    if (!motion_stop_candidate_) {
      motion_stop_candidate_ = now;
      return;
    }
    const double held_s = std::chrono::duration<double>(
      now - *motion_stop_candidate_).count();
    if (held_s < motion_stop_hold_sec_) {
      return;
    }

    vehicle_moving_ = false;
    motion_stop_candidate_.reset();
    std::ostringstream details;
    details << std::fixed << std::setprecision(6)
            << "speed_mps=" << speed
            << " threshold_mps=" << motion_stop_speed_mps_
            << " held_s=" << held_s;
    record_event("vehicle_stopped", "/odom", details.str());
    maybe_record_route_completed();
  }

  void maybe_record_route_completed()
  {
    if (
      !auto_drive_ || !goal_accepted_ || !route_completion_received_ ||
      !route_motion_seen_ || vehicle_moving_ || route_completed_recorded_)
    {
      return;
    }
    route_completed_recorded_ = true;
    record_event(
      "route_completed", "/foxglove/waypoints/status + /odom",
      "Foxglove reported Route completed and vehicle is stopped; "
      "last_speed_mps=" + std::to_string(last_odom_speed_mps_));
  }

  void write_sample()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto wall_now = std::chrono::system_clock::now();
    const std::string wall_timestamp = iso_timestamp(wall_now);
    const double elapsed_s =
      std::chrono::duration<double>(steady_now - started_at_).count();
    const double interval_s =
      std::chrono::duration<double>(steady_now - last_sample_at_).count();
    ++sample_number_;

    for (Signal & signal : signals_) {
      const double hz =
        interval_s > 0.0 ?
        static_cast<double>(signal.interval_count) / interval_s : 0.0;
      csv_ << wall_timestamp << ','
           << std::fixed << std::setprecision(6) << elapsed_s << ','
           << sample_number_ << ','
           << signal.name << ','
           << signal.topic << ','
           << interval_s << ','
           << signal.interval_count << ','
           << hz << ','
           << signal.total_count << ',';
      if (signal.last_message_at) {
        const double age_s = std::chrono::duration<double>(
          steady_now - *signal.last_message_at).count();
        csv_ << std::max(0.0, age_s);
      }
      csv_ << '\n';
      signal.interval_count = 0;
    }
    csv_.flush();
    last_sample_at_ = steady_now;
  }

  std::string output_path_;
  std::string event_output_path_;
  double interval_sec_ = 1.0;
  double motion_start_speed_mps_ = 0.08;
  double motion_start_hold_sec_ = 0.2;
  double motion_stop_speed_mps_ = 0.05;
  double motion_stop_hold_sec_ = 1.0;
  uint64_t sample_number_ = 0;
  uint64_t event_number_ = 0;
  uint64_t auto_session_ = 0;
  uint64_t route_session_ = 0;
  std::ofstream csv_;
  std::ofstream event_csv_;
  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point last_sample_at_;
  rclcpp::QoS qos_;
  rclcpp::QoS event_qos_;
  rclcpp::QoS latched_event_qos_;
  std::vector<Signal> signals_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr sample_timer_;

  std::string main_state_;
  std::string last_route_status_;
  std::string last_behavior_state_;
  bool auto_drive_ = false;
  bool route_requested_ = false;
  bool goal_accepted_ = false;
  bool route_completion_received_ = false;
  bool route_completed_recorded_ = false;
  bool route_motion_seen_ = false;
  bool vehicle_moving_ = false;
  double last_odom_speed_mps_ = 0.0;
  std::optional<std::chrono::steady_clock::time_point>
  motion_start_candidate_;
  std::optional<std::chrono::steady_clock::time_point>
  motion_stop_candidate_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PipelineRateMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("pipeline_rate_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
