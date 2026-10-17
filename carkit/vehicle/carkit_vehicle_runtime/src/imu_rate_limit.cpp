#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuRateLimit : public rclcpp::Node {
  using Imu=sensor_msgs::msg::Imu;
  using Clock=std::chrono::steady_clock;
  std::array<Imu::ConstSharedPtr,2> pending_;
  std::array<int64_t,2> last_stamp_{};
  std::array<Clock::time_point,2> arrival_;
  std::array<rclcpp::Publisher<Imu>::SharedPtr,2> publishers_;
  std::array<rclcpp::Subscription<Imu>::SharedPtr,2> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
public:
  ImuRateLimit():Node("imu_rate_limit") {
    const std::array<std::string,2> streams{"gyro","accel"};
    for(size_t i=0;i<2;i++) {
      const auto topic="/camera/camera/"+streams[i]+"/sample";
      publishers_[i]=create_publisher<Imu>(topic,rclcpp::SensorDataQoS());
      subscriptions_[i]=create_subscription<Imu>(topic+"_raw",rclcpp::SensorDataQoS(),
        [this,i](Imu::ConstSharedPtr m){
          const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
          const double age=(now().nanoseconds()-stamp)*1e-9;
          if(age<-.05||age>.25||stamp<=last_stamp_[i])return;
          last_stamp_[i]=stamp;arrival_[i]=Clock::now();pending_[i]=std::move(m);
        });
    }
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){
      for(size_t i=0;i<2;i++) {
        auto message=std::move(pending_[i]);
        if(!message)continue;
        const double age=(now().nanoseconds()-last_stamp_[i])*1e-9;
        if(age>=-.05&&age<=.25&&std::chrono::duration<double>(Clock::now()-arrival_[i]).count()<=.25)
          publishers_[i]->publish(*message);
      }
    });
  }
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ImuRateLimit>());rclcpp::shutdown();}
