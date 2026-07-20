#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "builtin_interfaces/msg/time.hpp"
#include "carkit_perception_msgs/msg/perception_latency_trace.hpp"
#include "carkit_perception_msgs/msg/stop_latency_trace.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

std::string default_output_path()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif
  std::ostringstream name;
  name << "stop_latency_" << std::put_time(&local_time, "%Y%m%d_%H%M%S")
       << ".csv";
  return (std::filesystem::path("log") / "latency" / name.str()).string();
}

std::string event_name(uint8_t event_type)
{
  using Trace = carkit_perception_msgs::msg::StopLatencyTrace;
  if (event_type == Trace::STOP_SIGN) {
    return "STOP_SIGN";
  }
  if (event_type == Trace::RED_LIGHT) {
    return "RED_LIGHT";
  }
  return "UNKNOWN";
}

}  // namespace

class StopLatencyMonitor : public rclcpp::Node
{
public:
  using PerceptionTrace =
    carkit_perception_msgs::msg::PerceptionLatencyTrace;
  using StopTrace = carkit_perception_msgs::msg::StopLatencyTrace;
  using Odometry = nav_msgs::msg::Odometry;

  StopLatencyMonitor()
  : Node("stop_latency_monitor")
  {
    declare_parameter<std::string>("output_path", "");
    declare_parameter<double>("stop_speed_threshold_mps", 0.05);
    declare_parameter<double>("stop_hold_sec", 0.3);
    declare_parameter<double>("stop_timeout_sec", 10.0);

    output_path_ = get_parameter("output_path").as_string();
    if (output_path_.empty()) {
      output_path_ = default_output_path();
    }
    stop_speed_threshold_mps_ =
      std::abs(get_parameter("stop_speed_threshold_mps").as_double());
    stop_hold_ns_ = static_cast<int64_t>(
      std::max(0.0, get_parameter("stop_hold_sec").as_double()) *
      kNanosecondsPerSecond);
    stop_timeout_sec_ =
      std::max(0.1, get_parameter("stop_timeout_sec").as_double());

    open_csv();

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    perception_sub_ = create_subscription<PerceptionTrace>(
      "/perception/latency_trace", reliable_qos,
      std::bind(&StopLatencyMonitor::perception_callback, this,
      std::placeholders::_1));
    stop_trace_sub_ = create_subscription<StopTrace>(
      "/behavior/stop_latency_trace", reliable_qos,
      std::bind(&StopLatencyMonitor::stop_trace_callback, this,
      std::placeholders::_1));
    odom_sub_ = create_subscription<Odometry>(
      "/odom", reliable_qos,
      std::bind(&StopLatencyMonitor::odom_callback, this,
      std::placeholders::_1));
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&StopLatencyMonitor::timeout_callback, this));

    RCLCPP_INFO(
      get_logger(), "Recording stop latency to %s", output_path_.c_str());
  }

private:
  struct PerceptionRecord
  {
    int64_t t1_ns;
  };

  struct OdomRecord
  {
    int64_t stamp_ns;
    double speed_mps;
  };

  struct ActiveEvent
  {
    uint32_t trial_id;
    uint8_t event_type;
    int64_t t0_ns;
    std::optional<int64_t> t1_ns;
    int64_t t5_ns;
    int64_t t6_ns;
    std::optional<int64_t> t8_ns;
    std::optional<int64_t> stop_candidate_ns;
    int64_t last_odom_stamp_ns = std::numeric_limits<int64_t>::min();
    std::string result;
    std::chrono::steady_clock::time_point started_at;
  };

  static constexpr size_t kMaxPerceptionRecords = 2048;
  static constexpr int64_t kOdomCacheDurationNs = 15 * kNanosecondsPerSecond;

  void open_csv()
  {
    const std::filesystem::path path(output_path_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Cannot open latency CSV: " + output_path_);
    }
    csv_ << "trial_id,event_type,t0_ns,t1_ns,t5_ns,t6_ns,t8_ns,"
            "l1_ms,l2_ms,l3_ms,l4_ms,result\n";
    csv_.flush();
  }

  void perception_callback(const PerceptionTrace::SharedPtr msg)
  {
    const int64_t t0_ns = stamp_to_ns(msg->header.stamp);
    const int64_t t1_ns = stamp_to_ns(msg->detection_publish_stamp);

    if (has_sequence_) {
      if (msg->detection_sequence > last_sequence_ + 1) {
        RCLCPP_WARN(
          get_logger(), "TRACE_SEQUENCE_GAP: expected %lu, received %lu",
          static_cast<unsigned long>(last_sequence_ + 1),
          static_cast<unsigned long>(msg->detection_sequence));
      } else if (msg->detection_sequence <= last_sequence_) {
        RCLCPP_WARN(
          get_logger(), "INVALID_TRACE_SEQUENCE: previous %lu, received %lu",
          static_cast<unsigned long>(last_sequence_),
          static_cast<unsigned long>(msg->detection_sequence));
      }
      if (t1_ns <= last_perception_publish_ns_) {
        RCLCPP_WARN(
          get_logger(),
          "INVALID_TRACE_SEQUENCE: publish time did not increase");
      }
    }
    has_sequence_ = true;
    last_sequence_ = msg->detection_sequence;
    last_perception_publish_ns_ = t1_ns;

    perception_records_[t0_ns] = PerceptionRecord{t1_ns};
    perception_order_.push_back(t0_ns);
    while (perception_order_.size() > kMaxPerceptionRecords) {
      const int64_t oldest = perception_order_.front();
      perception_order_.pop_front();
      perception_records_.erase(oldest);
    }
  }

  void stop_trace_callback(const StopTrace::SharedPtr msg)
  {
    if (msg->event_type == StopTrace::RESET) {
      if (active_event_) {
        finish_event("RESET");
      }
      return;
    }
    if (
      msg->event_type != StopTrace::STOP_SIGN &&
      msg->event_type != StopTrace::RED_LIGHT)
    {
      RCLCPP_WARN(get_logger(), "Ignoring unknown stop latency event type");
      return;
    }
    if (active_event_) {
      finish_event("OVERLAPPED_EVENT");
    }

    ActiveEvent event;
    event.trial_id = msg->trial_id;
    event.event_type = msg->event_type;
    event.t0_ns = stamp_to_ns(msg->source_image_stamp);
    event.t5_ns = stamp_to_ns(msg->track_ready_stamp);
    event.t6_ns = stamp_to_ns(msg->override_stamp);
    event.started_at = std::chrono::steady_clock::now();

    const auto perception = perception_records_.find(event.t0_ns);
    if (perception == perception_records_.end()) {
      event.result = "MISSING_T1";
    } else {
      event.t1_ns = perception->second.t1_ns;
      event.result = "OK";
    }

    if (
      event.t0_ns <= 0 || event.t5_ns <= 0 || event.t6_ns <= 0 ||
      event.t5_ns > event.t6_ns ||
      (event.t1_ns &&
      (event.t0_ns > *event.t1_ns || *event.t1_ns > event.t5_ns)))
    {
      active_event_ = event;
      finish_event("INVALID_TIME_ORDER");
      return;
    }

    active_event_ = event;
    for (const auto & odom : odom_records_) {
      if (!active_event_) {
        break;
      }
      process_odom_for_active_event(odom);
    }
  }

  void odom_callback(const Odometry::SharedPtr msg)
  {
    OdomRecord record{
      stamp_to_ns(msg->header.stamp),
      static_cast<double>(msg->twist.twist.linear.x)};
    odom_records_.push_back(record);
    while (
      !odom_records_.empty() &&
      record.stamp_ns - odom_records_.front().stamp_ns >
      kOdomCacheDurationNs)
    {
      odom_records_.pop_front();
    }
    process_odom_for_active_event(record);
  }

  void process_odom_for_active_event(const OdomRecord & odom)
  {
    if (!active_event_ || odom.stamp_ns < active_event_->t6_ns ||
      odom.stamp_ns <= active_event_->last_odom_stamp_ns)
    {
      return;
    }
    active_event_->last_odom_stamp_ns = odom.stamp_ns;

    if (std::abs(odom.speed_mps) < stop_speed_threshold_mps_) {
      if (!active_event_->stop_candidate_ns) {
        active_event_->stop_candidate_ns = odom.stamp_ns;
      }
      if (
        odom.stamp_ns - *active_event_->stop_candidate_ns >= stop_hold_ns_)
      {
        active_event_->t8_ns = active_event_->stop_candidate_ns;
        finish_event(active_event_->result);
      }
    } else {
      active_event_->stop_candidate_ns.reset();
    }
  }

  void timeout_callback()
  {
    if (!active_event_) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - active_event_->started_at).count();
    if (elapsed >= stop_timeout_sec_) {
      finish_event("STOP_TIMEOUT");
    }
  }

  static void write_optional_int(
    std::ostream & output, const std::optional<int64_t> & value)
  {
    if (value) {
      output << *value;
    }
  }

  static void write_latency_ms(
    std::ostream & output,
    const std::optional<int64_t> & end,
    const std::optional<int64_t> & start)
  {
    if (end && start) {
      output << std::fixed << std::setprecision(3)
             << static_cast<double>(*end - *start) / 1.0e6;
    }
  }

  void finish_event(const std::string & result)
  {
    if (!active_event_) {
      return;
    }
    const auto event = *active_event_;
    const std::optional<int64_t> t0 = event.t0_ns;
    const std::optional<int64_t> t5 = event.t5_ns;
    const std::optional<int64_t> t6 = event.t6_ns;

    csv_ << event.trial_id << ',' << event_name(event.event_type) << ','
         << event.t0_ns << ',';
    write_optional_int(csv_, event.t1_ns);
    csv_ << ',' << event.t5_ns << ',' << event.t6_ns << ',';
    write_optional_int(csv_, event.t8_ns);
    csv_ << ',';
    write_latency_ms(csv_, event.t1_ns, t0);
    csv_ << ',';
    write_latency_ms(csv_, t5, event.t1_ns);
    csv_ << ',';
    write_latency_ms(csv_, t6, t5);
    csv_ << ',';
    write_latency_ms(csv_, event.t8_ns, t6);
    csv_ << ',' << result << '\n';
    csv_.flush();

    RCLCPP_INFO(
      get_logger(), "Trial %u %s finished: %s", event.trial_id,
      event_name(event.event_type).c_str(), result.c_str());
    active_event_.reset();
  }

  std::string output_path_;
  double stop_speed_threshold_mps_ = 0.05;
  int64_t stop_hold_ns_ = 300000000LL;
  double stop_timeout_sec_ = 10.0;
  std::ofstream csv_;

  bool has_sequence_ = false;
  uint64_t last_sequence_ = 0;
  int64_t last_perception_publish_ns_ = 0;
  std::unordered_map<int64_t, PerceptionRecord> perception_records_;
  std::deque<int64_t> perception_order_;
  std::deque<OdomRecord> odom_records_;
  std::optional<ActiveEvent> active_event_;

  rclcpp::Subscription<PerceptionTrace>::SharedPtr perception_sub_;
  rclcpp::Subscription<StopTrace>::SharedPtr stop_trace_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StopLatencyMonitor>());
  rclcpp::shutdown();
  return 0;
}
