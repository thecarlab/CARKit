#include <chrono>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include "carkit_vehicle_runtime/motion_math.hpp"

class VescSpeedControl : public rclcpp::Node {
  using Command=ackermann_msgs::msg::AckermannDriveStamped;
  using Wheel=vesc_msgs::msg::VescStateStamped;
  using Clock=std::chrono::steady_clock;
  double gain_,offset_,measured_{0};
  carkit_vehicle_runtime::SpeedFeedback control_;
  std::optional<Command> command_;
  std::optional<Clock::time_point> command_time_,feedback_time_;
  std::optional<int64_t> last_stamp_;
  Clock::time_point previous_time_{Clock::now()};
  rclcpp::Publisher<Command>::SharedPtr publisher_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Wheel>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void tick() {
    const auto t=Clock::now();
    const double dt=std::chrono::duration<double>(t-previous_time_).count();previous_time_=t;
    auto message=command_.value_or(Command{});
    const bool stale_command=!command_time_||std::chrono::duration<double>(t-*command_time_).count()>.25;
    const bool stale_feedback=!feedback_time_||std::chrono::duration<double>(t-*feedback_time_).count()>.25;
    if(stale_command||stale_feedback) {
      control_.reset();message.drive.speed=0;
      if(stale_command)message.drive.steering_angle=0;
    } else message.drive.speed=control_.update(message.drive.speed,measured_,dt);
    message.header.stamp=now();publisher_->publish(message);
  }
public:
  VescSpeedControl():Node("vesc_speed_control"),
    gain_(declare_parameter("speed_to_erpm_gain",4614.0)),
    offset_(declare_parameter("speed_to_erpm_offset",0.0)),
    control_(declare_parameter("max_speed",.8),declare_parameter("max_reverse_speed",.8),
      declare_parameter("speed_kp",.35),declare_parameter("speed_ki",.6),declare_parameter("speed_correction_limit",.5)) {
    if(!std::isfinite(gain_)||gain_==0||offset_!=0||control_.forward<=0||control_.reverse<=0||control_.limit<0)
      throw std::invalid_argument("Invalid speed controller parameters");
    publisher_=create_publisher<Command>("ackermann_cmd/feedback",1);
    command_sub_=create_subscription<Command>("ackermann_cmd",1,[this](Command::ConstSharedPtr m){
      if(!std::isfinite(m->drive.speed)||!std::isfinite(m->drive.steering_angle)) {
        command_.reset();command_time_.reset();control_.reset();tick();return;
      }
      command_=*m;command_time_=Clock::now();
      if(std::abs(m->drive.speed)<1e-6){control_.reset();publisher_->publish(*m);}
    });
    wheel_sub_=create_subscription<Wheel>("sensors/core",rclcpp::SensorDataQoS(),[this](Wheel::ConstSharedPtr m){
      const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
      const double age=(now().nanoseconds()-stamp)*1e-9;
      const double speed=(m->state.speed-offset_)/gain_;
      if(!std::isfinite(speed)||age<-.05||age>.25||(last_stamp_&&stamp<=*last_stamp_)||
          std::abs(speed)>1.5*std::max(control_.forward,control_.reverse)||m->state.fault_code!=0) {
        feedback_time_.reset();control_.reset();return;
      }
      last_stamp_=stamp;measured_=speed;feedback_time_=Clock::now();
    });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
  }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VescSpeedControl>());rclcpp::shutdown();}
