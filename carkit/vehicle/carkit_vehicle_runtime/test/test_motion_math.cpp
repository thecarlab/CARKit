#include <gtest/gtest.h>
#include "carkit_vehicle_runtime/motion_math.hpp"
using namespace carkit_vehicle_runtime;
TEST(SpeedFeedback,TrackingReverseAndOverspeed) {
  SpeedFeedback c(4,2);double speed=0,output=0;
  for(int i=0;i<1000;i++){output=c.update(1,speed,.02);speed+=.08*(.8*output-speed);}
  EXPECT_NEAR(speed,1,.01);EXPECT_NEAR(output,1.25,.02);
  EXPECT_LT(c.update(-1,-.5,.02),-1);
  c.reset();EXPECT_LT(c.update(1,1.3,.02),1);
}
TEST(SpeedFeedback,LimitsStopAndInvalid) {
  SpeedFeedback c(.8,.4);
  for(int i=0;i<1000;i++)EXPECT_LE(c.update(1,0,.02),.8);
  EXPECT_EQ(c.integral,0);EXPECT_GE(c.update(-1,0,.02),-.4);
  EXPECT_EQ(c.update(0,0,.02),0);EXPECT_EQ(c.integral,0);
  EXPECT_EQ(c.update(NAN,0,.02),0);EXPECT_EQ(c.update(1,0,.5),0);
}
TEST(MeasuredOdometry,StraightReverseAndGaps) {
  MeasuredOdometry m;m.update(1,.5,0);m.update(1.2,.5,0);
  EXPECT_NEAR(m.x,.1,1e-10);EXPECT_NEAR(m.y,0,1e-10);
  EXPECT_FALSE(m.update(1.2,100,100));
  m.update(2,.5,0);EXPECT_NEAR(m.x,.1,1e-10);EXPECT_GT(m.covariance[0][0],1);
  m.pause();m.update(3,-.5,0);m.update(3.2,-.5,0);EXPECT_NEAR(m.x,0,1e-10);
}
TEST(MeasuredOdometry,ArcAndRotation) {
  MeasuredOdometry m;m.update(1,1,.5);
  for(int i=1;i<=50;i++)m.update(1+i*.02,1,.5);
  EXPECT_NEAR(m.x,2*std::sin(.5),1e-9);EXPECT_NEAR(m.y,2*(1-std::cos(.5)),1e-9);
  EXPECT_NEAR(m.yaw,.5,1e-9);
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)EXPECT_NEAR(m.covariance[i][j],m.covariance[j][i],1e-12);
}
