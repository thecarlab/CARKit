#define CARKIT_CONTROL_CENTER_NO_MAIN
#include "../src/control_center_node.cpp"
#include <thread>
#include <limits>
#include "gtest/gtest.h"

using namespace std::chrono_literals;

class CommandSafety : public ::testing::Test
{
protected:
  virtual bool require_odometry() const {return false;}
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("publish_rate_hz", 100.0),
      rclcpp::Parameter("require_measured_odometry", require_odometry()),
      rclcpp::Parameter("odometry_timeout_sec", 0.15),
      rclcpp::Parameter("behavior_timeout_sec", 0.10),
      rclcpp::Parameter("nav2_timeout_sec", 0.10),
      rclcpp::Parameter("max_speed", 0.8), rclcpp::Parameter("max_reverse_speed", 0.3)});
    arbiter = std::make_shared<ControlCenterNode>(options);
    peer = std::make_shared<rclcpp::Node>("command_safety_test");
    drive = peer->create_publisher<Drive>("/drive", 10);
    behavior = peer->create_publisher<Drive>("/behavior/override_cmd", 10);
    active = peer->create_publisher<std_msgs::msg::Bool>("/behavior/override_active", 10);
    enable = peer->create_publisher<std_msgs::msg::Int8>("enable_autonomous_control", 10);
    joy = peer->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    output = peer->create_subscription<Drive>("/ackermann_cmd", 10,
      [this](Drive::SharedPtr msg) {last = *msg; ++received;});
    executor.add_node(arbiter);
    executor.add_node(peer);
    pump(300ms);
    ASSERT_GT(received, 0);
    ASSERT_GT(drive->get_subscription_count(), 0U);
    std_msgs::msg::Int8 mode;
    mode.data = 1;
    enable->publish(mode);
    pump(50ms);
  }

  void TearDown() override
  {
    executor.remove_node(peer);
    executor.remove_node(arbiter);
    output.reset(); joy.reset(); enable.reset(); active.reset(); behavior.reset(); drive.reset();
    peer.reset(); arbiter.reset();
  }

  void pump(std::chrono::milliseconds duration, const std::function<void()> & send = []() {})
  {
    const auto until = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < until) {
      send(); executor.spin_some(); std::this_thread::sleep_for(5ms);
    }
  }

  void send_drive(float speed = 0.6F)
  {
    Drive cmd; cmd.drive.speed = speed; drive->publish(cmd);
  }

  void activate_stop()
  {
    std_msgs::msg::Bool flag; flag.data = true;
    pump(50ms, [&]() {active->publish(flag); behavior->publish(Drive()); send_drive();});
    ASSERT_FLOAT_EQ(last.drive.speed, 0.0F);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ControlCenterNode> arbiter;
  rclcpp::Node::SharedPtr peer;
  rclcpp::Publisher<Drive>::SharedPtr drive, behavior;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr enable;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy;
  rclcpp::Subscription<Drive>::SharedPtr output;
  Drive last;
  int received{0};
};

class MeasuredMotionSafety : public CommandSafety
{
protected:
  bool require_odometry() const override {return true;}
};

TEST_F(MeasuredMotionSafety, MissingFrozenAndLostMeasurementsBlockAutonomy)
{
  auto odom = peer->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  pump(100ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  nav_msgs::msg::Odometry sample;
  sample.pose.pose.orientation.w = 1.0;
  pump(100ms, [&]() {sample.header.stamp = peer->now(); odom->publish(sample); send_drive();});
  EXPECT_GT(last.drive.speed, 0.0F);
  // Repeated receipt of an old measurement must not keep autonomous motion alive.
  pump(250ms, [&]() {odom->publish(sample); send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  pump(80ms, [&]() {sample.header.stamp = peer->now(); odom->publish(sample); send_drive();});
  EXPECT_GT(last.drive.speed, 0.0F);
  pump(250ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(MeasuredMotionSafety, ManualControlDoesNotRequireLocalization)
{
  auto teleop = peer->create_publisher<Drive>("/teleop", 10);
  std_msgs::msg::Int8 mode; mode.data = 0; enable->publish(mode);
  Drive command; command.drive.speed = 0.2F;
  pump(100ms, [&]() {teleop->publish(command);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.2F);
}

TEST_F(CommandSafety, LostBehaviorProcessHoldsStopUntilExplicitRelease)
{
  pump(80ms, [&]() {send_drive();});
  ASSERT_GT(last.drive.speed, 0.0F);
  activate_stop();
  pump(250ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  std_msgs::msg::Bool release; release.data = false;
  active->publish(release);
  pump(80ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.6F);
}

TEST_F(CommandSafety, ActiveHeartbeatWithoutCommandHoldsStop)
{
  activate_stop();
  std_msgs::msg::Bool flag; flag.data = true;
  pump(250ms, [&]() {active->publish(flag); send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, CommandWithoutActiveHeartbeatHoldsStop)
{
  activate_stop();
  Drive cmd; cmd.drive.speed = 0.2F;
  pump(250ms, [&]() {behavior->publish(cmd); send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, EmergencyStopWinsOverSimultaneousClear)
{
  sensor_msgs::msg::Joy buttons; buttons.buttons = {0, 0, 1, 1};
  joy->publish(buttons);
  pump(80ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, InvalidSteeringStopsSpeedAndReverseUsesOwnLimit)
{
  Drive invalid; invalid.drive.speed = 0.6F;
  invalid.drive.steering_angle = std::numeric_limits<float>::quiet_NaN();
  pump(80ms, [&]() {drive->publish(invalid);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  EXPECT_FLOAT_EQ(last.drive.steering_angle, 0.0F);
  pump(80ms, [&]() {send_drive(-2.0F);});
  EXPECT_FLOAT_EQ(last.drive.speed, -0.3F);
  pump(160ms);
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, ClearingEmergencyStopAlwaysReturnsToManual)
{
  pump(50ms, [&]() {send_drive();});
  ASSERT_GT(last.drive.speed, 0.0F);
  sensor_msgs::msg::Joy buttons; buttons.buttons = {0, 0, 1, 0};
  joy->publish(buttons);
  pump(50ms);
  // Autonomous requests received during the stop must not be queued for reset.
  std_msgs::msg::Int8 request; request.data = 1;
  enable->publish(request);
  pump(50ms);
  buttons.buttons = {0, 0, 0, 1};
  joy->publish(buttons);
  pump(80ms, [&]() {send_drive();});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  // Only a new request after reset may re-enable autonomous motion.
  enable->publish(request);
  pump(80ms, [&]() {send_drive();});
  EXPECT_GT(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, CalibrationOwnsCommandAndLostHeartbeatLatchesStop)
{
  auto cal = peer->create_publisher<Drive>("/calibration/command", 10);
  auto heartbeat = peer->create_publisher<std_msgs::msg::Bool>("/calibration/active", 10);
  auto manual = peer->create_publisher<Drive>("/teleop", 10);
  pump(150ms);
  std_msgs::msg::Int8 human; human.data = 0; enable->publish(human);
  pump(40ms, [&]() {cal->publish(Drive());});
  std_msgs::msg::Bool yes; yes.data = true; heartbeat->publish(yes);
  pump(40ms, [&]() {cal->publish(Drive());});
  Drive test; test.drive.speed = 2.0F; test.drive.steering_angle = 0.1F;
  Drive other; other.drive.speed = 0.8F; other.drive.steering_angle = -0.2F;
  std_msgs::msg::Int8 autonomous; autonomous.data = 1;
  pump(80ms, [&]() {
    cal->publish(test); heartbeat->publish(yes); manual->publish(other);
    drive->publish(other); enable->publish(autonomous);
  });
  EXPECT_FLOAT_EQ(last.drive.speed, 0.8F);  // Vehicle limit remains below the 1.0 m/s calibration cap.
  EXPECT_FLOAT_EQ(last.drive.steering_angle, 0.1F);
  // Keep every competing source fresh while the calibration worker disappears.
  pump(400ms, [&]() {manual->publish(other); drive->publish(other);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  EXPECT_FLOAT_EQ(last.drive.steering_angle, 0.0F);
  pump(80ms, [&]() {cal->publish(test); heartbeat->publish(yes);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}

TEST_F(CommandSafety, CalibrationRequiresStationaryArmAndCannotRearmAcrossEstop)
{
  auto cal = peer->create_publisher<Drive>("/calibration/command", 10);
  auto heartbeat = peer->create_publisher<std_msgs::msg::Bool>("/calibration/active", 10);
  auto stop = peer->create_publisher<std_msgs::msg::Bool>("/control_center/emergency_stop", 10);
  pump(150ms);
  std_msgs::msg::Int8 human; human.data = 0; enable->publish(human);
  Drive moving; moving.drive.speed = 0.2F;
  std_msgs::msg::Bool yes; yes.data = true;
  std_msgs::msg::Bool no; no.data = false;
  pump(40ms, [&]() {cal->publish(moving);});
  pump(40ms, [&]() {heartbeat->publish(yes); cal->publish(moving);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  heartbeat->publish(no);
  pump(40ms, [&]() {cal->publish(Drive());});
  heartbeat->publish(yes);
  pump(40ms, [&]() {cal->publish(Drive());});
  pump(80ms, [&]() {cal->publish(moving); heartbeat->publish(yes);});
  ASSERT_GT(last.drive.speed, 0.0F);
  stop->publish(yes);
  pump(40ms, [&]() {cal->publish(moving); heartbeat->publish(yes);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
  stop->publish(no);
  pump(80ms, [&]() {cal->publish(moving); heartbeat->publish(yes);});
  EXPECT_FLOAT_EQ(last.drive.speed, 0.0F);
}
