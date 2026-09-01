// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#ifndef CARKIT_INTRO2AV_CPP__CONTROL_ALGORITHM_HPP_
#define CARKIT_INTRO2AV_CPP__CONTROL_ALGORITHM_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_intro2av_cpp
{

struct ControlConfig
{
  double wheelbase_m{0.325};
  double lookahead_m{0.55};
  double target_speed_mps{0.45};
  double maximum_speed_mps{1.0};
  double maximum_steering_rad{0.34};
  double goal_tolerance_m{0.15};
};

struct ControlCommand
{
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
};

ControlCommand compute_command(
  const nav_msgs::msg::Odometry & odometry,
  const nav_msgs::msg::Path & path,
  const ControlConfig & config);

}  // namespace carkit_intro2av_cpp

#endif  // CARKIT_INTRO2AV_CPP__CONTROL_ALGORITHM_HPP_
