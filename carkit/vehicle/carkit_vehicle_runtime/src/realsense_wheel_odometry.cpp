#include <chrono>
#include <deque>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "carkit_vehicle_runtime/motion_math.hpp"

class RealSenseOdometry : public rclcpp::Node {
  using Imu=sensor_msgs::msg::Imu;
  using Wheel=vesc_msgs::msg::VescStateStamped;
  using Clock=std::chrono::steady_clock;
  struct Sample {double stamp;Clock::time_point arrival;std::array<double,3> value;};
  double gain_,offset_,timeout_,max_speed_,bias_;
  bool bias_ready_;
  std::string base_frame_,odom_frame_,last_reason_;
  std::deque<std::array<double,5>> bias_samples_;
  std::optional<Sample> wheel_,gyro_,accel_;
  carkit_vehicle_runtime::MeasuredOdometry motion_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<Wheel>::SharedPtr wheel_sub_;
  rclcpp::Subscription<Imu>::SharedPtr gyro_sub_,accel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool valid_stamp(const builtin_interfaces::msg::Time & stamp,const std::optional<Sample>& previous) {
    const double t=rclcpp::Time(stamp).seconds(),age=now().seconds()-t;
    return t>0&&age>=-.05&&age<=timeout_&&(!previous||t>previous->stamp);
  }
  bool fresh(const std::optional<Sample>& sample) {
    if(!sample)return false;
    const double age=now().seconds()-sample->stamp;
    return age>=-.05&&age<=timeout_&&std::chrono::duration<double>(Clock::now()-sample->arrival).count()<=timeout_;
  }
  std::array<double,3> rotate(const std::string& frame,const geometry_msgs::msg::Vector3& vector) {
    if(frame.empty())throw std::invalid_argument("Empty IMU frame");
    const auto t=buffer_.lookupTransform(base_frame_,frame,tf2::TimePointZero);
    const auto q=t.transform.rotation;
    tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
    if(!std::isfinite(quaternion.length2())||quaternion.length2()<1e-18||
        !std::isfinite(vector.x)||!std::isfinite(vector.y)||!std::isfinite(vector.z))
      throw std::invalid_argument("Invalid mounting quaternion or IMU vector");
    quaternion.normalize();
    const auto v=tf2::Matrix3x3(quaternion)*tf2::Vector3(vector.x,vector.y,vector.z);
    return {v.x(),v.y(),v.z()};
  }
  bool calibrate() {
    if(!fresh(accel_)||std::abs(wheel_->value[0])>.015||std::abs(gyro_->value[0])>.05) {
      bias_samples_.clear();return false;
    }
    const auto a=accel_->value;
    const double norm=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(norm<=8||norm>=11){bias_samples_.clear();return false;}
    const double t=gyro_->stamp;
    if(bias_samples_.empty()||t>bias_samples_.back()[0])bias_samples_.push_back({t,gyro_->value[0],a[0],a[1],a[2]});
    while(!bias_samples_.empty()&&t-bias_samples_.front()[0]>2.2)bias_samples_.pop_front();
    if(bias_samples_.size()<50||t-bias_samples_.front()[0]<2)return false;
    std::array<double,5> means{},variances{};
    for(const auto& row:bias_samples_)for(int j=1;j<5;j++)means[j]+=row[j]/bias_samples_.size();
    for(const auto& row:bias_samples_)for(int j=1;j<5;j++)variances[j]+=(row[j]-means[j])*(row[j]-means[j])/bias_samples_.size();
    if(std::sqrt(variances[1])>.01||std::sqrt(variances[2])>.15||std::sqrt(variances[3])>.15||std::sqrt(variances[4])>.15) {
      bias_samples_.clear();return false;
    }
    bias_=means[1];bias_ready_=true;bias_samples_.clear();
    RCLCPP_INFO(get_logger(),"Stationary RealSense body yaw bias: %.6f rad/s",bias_);return true;
  }
  void tick() {
    std::string reason;
    if(!fresh(wheel_))reason="Waiting for fresh measured VESC RPM; odometry paused";
    else if(!fresh(gyro_))reason="Waiting for fresh RealSense gyro and mounting TF; odometry paused";
    else if(!bias_ready_&&!calibrate())reason="Hold vehicle still for RealSense gyro bias initialization (2 seconds)";
    if(!reason.empty()) {
      motion_.pause();
      if(!fresh(wheel_)||!fresh(gyro_))bias_samples_.clear();
      if(reason!=last_reason_)RCLCPP_WARN(get_logger(),"%s",reason.c_str());
      last_reason_=reason;return;
    }
    if(!last_reason_.empty()){RCLCPP_INFO(get_logger(),"Measured odometry ready: RealSense gyro + VESC RPM");last_reason_.clear();}
    const double speed=wheel_->value[0],rate=gyro_->value[0]-bias_;
    if(!motion_.update(gyro_->stamp,speed,rate))return;
    nav_msgs::msg::Odometry m;
    m.header.stamp=rclcpp::Time(static_cast<int64_t>(gyro_->stamp*1e9));
    m.header.frame_id=odom_frame_;m.child_frame_id=base_frame_;
    m.pose.pose.position.x=motion_.x;m.pose.pose.position.y=motion_.y;
    m.pose.pose.orientation.z=std::sin(motion_.yaw/2);m.pose.pose.orientation.w=std::cos(motion_.yaw/2);
    m.twist.twist.linear.x=speed;m.twist.twist.angular.z=rate;
    for(int i:{2,3,4}){m.pose.covariance[i*6+i]=1e6;m.twist.covariance[i*6+i]=1e6;}
    const std::array<int,3> indices{0,1,5};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)m.pose.covariance[indices[i]*6+indices[j]]=motion_.covariance[i][j];
    m.twist.covariance[0]=motion_.speed_variance;m.twist.covariance[7]=.01;m.twist.covariance[35]=motion_.yaw_rate_variance;
    publisher_->publish(m);
  }
public:
  RealSenseOdometry():Node("realsense_wheel_odometry"),
    gain_(declare_parameter("speed_to_erpm_gain",4614.0)),offset_(declare_parameter("speed_to_erpm_offset",0.0)),
    timeout_(declare_parameter("feedback_timeout_sec",.25)),max_speed_(declare_parameter("max_measured_speed",6.0)),
    bias_(declare_parameter("gyro_bias_z",0.0)),bias_ready_(!declare_parameter("calibrate_gyro_bias",true)),
    base_frame_(declare_parameter("base_frame","base_link")),odom_frame_(declare_parameter("odom_frame","odom")),
    buffer_(get_clock()),listener_(buffer_,this,false) {
    if(!std::isfinite(gain_)||gain_==0||!std::isfinite(offset_)||!std::isfinite(timeout_)||timeout_<=0||
        !std::isfinite(max_speed_)||max_speed_<=0||!std::isfinite(bias_))throw std::invalid_argument("Invalid odometry parameters");
    motion_.max_gap=timeout_;
    const auto gyro_topic=declare_parameter("gyro_topic","/camera/camera/gyro/sample");
    const auto accel_topic=declare_parameter("accel_topic","/camera/camera/accel/sample");
    const auto wheel_topic=declare_parameter("wheel_topic","/sensors/core");
    publisher_=create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    wheel_sub_=create_subscription<Wheel>(wheel_topic,rclcpp::SensorDataQoS(),[this](Wheel::ConstSharedPtr m){
      const double speed=(m->state.speed-offset_)/gain_;
      if(m->state.fault_code!=0||!std::isfinite(speed)||std::abs(speed)>max_speed_){wheel_.reset();return;}
      if(valid_stamp(m->header.stamp,wheel_))wheel_=Sample{rclcpp::Time(m->header.stamp).seconds(),Clock::now(),{speed,0,0}};
    });
    gyro_sub_=create_subscription<Imu>(gyro_topic,rclcpp::SensorDataQoS(),[this](Imu::ConstSharedPtr m){
      if(!valid_stamp(m->header.stamp,gyro_))return;
      try {
        const auto v=rotate(m->header.frame_id,m->angular_velocity);
        if(std::abs(v[2])>20||m->angular_velocity_covariance[0]<0)throw std::invalid_argument("Invalid gyro");
        gyro_=Sample{rclcpp::Time(m->header.stamp).seconds(),Clock::now(),{v[2],0,0}};
      } catch(const tf2::TransformException&){gyro_.reset();}catch(const std::invalid_argument&){gyro_.reset();}
    });
    accel_sub_=create_subscription<Imu>(accel_topic,rclcpp::SensorDataQoS(),[this](Imu::ConstSharedPtr m){
      if(!valid_stamp(m->header.stamp,accel_))return;
      try {
        const auto v=rotate(m->header.frame_id,m->linear_acceleration);
        if(m->linear_acceleration_covariance[0]<0)throw std::invalid_argument("Invalid accel");
        accel_=Sample{rclcpp::Time(m->header.stamp).seconds(),Clock::now(),v};
      } catch(const tf2::TransformException&){accel_.reset();}catch(const std::invalid_argument&){accel_.reset();}
    });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Odometry uses measured VESC RPM and RealSense gyro; no command fallback");
  }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RealSenseOdometry>());rclcpp::shutdown();}
