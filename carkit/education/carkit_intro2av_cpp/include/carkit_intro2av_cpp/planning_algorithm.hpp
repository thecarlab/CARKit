// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#ifndef CARKIT_INTRO2AV_CPP__PLANNING_ALGORITHM_HPP_
#define CARKIT_INTRO2AV_CPP__PLANNING_ALGORITHM_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_intro2av_cpp
{

struct PlanningConfig
{
  int occupancy_threshold{65};
  bool allow_unknown{false};
  double inflation_radius_m{0.25};
  double waypoint_spacing_m{0.10};
  double goal_tolerance_m{0.15};
};

std::vector<geometry_msgs::msg::PoseStamped> compute_path(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const nav_msgs::msg::Odometry & odometry,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlanningConfig & config);

}  // namespace carkit_intro2av_cpp

#endif  // CARKIT_INTRO2AV_CPP__PLANNING_ALGORITHM_HPP_
