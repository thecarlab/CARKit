#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace carkit_vehicle_runtime {
struct SpeedFeedback {
  double forward, reverse, kp, ki, limit, integral{0.0}, previous_target{0.0};
  SpeedFeedback(double f, double r, double p=.35, double i=.6, double l=.5)
  : forward(f), reverse(r), kp(p), ki(i), limit(l) {}
  void reset() {integral=0; previous_target=0;}
  double update(double target, double measured, double dt) {
    if (!std::isfinite(target) || !std::isfinite(measured) || !std::isfinite(dt) || dt<=0 || dt>.25) {
      reset(); return 0;
    }
    target=std::clamp(target,-reverse,forward);
    if(std::abs(target)<1e-6) {reset();return 0;}
    if(target*previous_target<=0 || std::abs(target)<std::abs(previous_target)-.01) integral=0;
    previous_target=target;
    const double error=target-measured;
    const double candidate=std::clamp(integral+ki*error*dt,-limit,limit);
    const double correction=std::clamp(kp*error+candidate,-limit,limit);
    const double raw=target+correction;
    const double output=target>0 ? std::clamp(raw,0.0,forward) : std::clamp(raw,-reverse,0.0);
    if((output==raw && std::abs(kp*error+candidate)<=limit) || error*candidate<0) integral=candidate;
    return output;
  }
};

using Matrix = std::array<std::array<double,3>,3>;
struct MeasuredOdometry {
  double max_gap{.25}, speed_variance{.01}, yaw_rate_variance{.0004};
  double x{0},y{0},yaw{0};
  Matrix covariance{{{{.001,0,0}},{{0,.001,0}},{{0,0,.001}}}};
  std::optional<std::array<double,3>> previous;
  void pause() {
    if(previous) {covariance[0][0]+=1;covariance[1][1]+=1;covariance[2][2]+=.1;}
    previous.reset();
  }
  bool update(double stamp,double speed,double rate) {
    if(!std::isfinite(stamp)||!std::isfinite(speed)||!std::isfinite(rate)) throw std::invalid_argument("Nonfinite motion");
    if(!previous) {previous=std::array<double,3>{stamp,speed,rate};return true;}
    const auto old=*previous;
    const double dt=stamp-old[0];
    if(dt<=0)return false;
    if(dt>max_gap) {pause();previous=std::array<double,3>{stamp,speed,rate};return true;}
    const double velocity=(speed+old[1])/2, angular=(rate+old[2])/2;
    const double angle=angular*dt;
    const double scale=std::abs(angle)>1e-8 ? std::sin(angle/2)/(angle/2):1;
    const double heading=yaw+angle/2;
    const double dx=velocity*dt*scale*std::cos(heading),dy=velocity*dt*scale*std::sin(heading);
    x+=dx;y+=dy;yaw=std::atan2(std::sin(yaw+angle),std::cos(yaw+angle));
    Matrix f{{{{1,0,-dy}},{{0,1,dx}},{{0,0,1}}}},fp{},p{};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)fp[i][j]+=f[i][k]*covariance[k][j];
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)p[i][j]+=fp[i][k]*f[j][k];
    const std::array<double,2> direction{std::cos(heading),std::sin(heading)};
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++)p[i][j]+=direction[i]*direction[j]*speed_variance*dt*dt;
      p[i][i]+=.0025*dt;
    }
    p[2][2]+=yaw_rate_variance*dt*dt+.0001*dt;
    covariance=p;previous=std::array<double,3>{stamp,speed,rate};return true;
  }
};
} // namespace carkit_vehicle_runtime
