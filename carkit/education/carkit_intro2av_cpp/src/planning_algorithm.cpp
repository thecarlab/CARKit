// Copyright 2026 CARKit maintainers
// Licensed under the Apache License, Version 2.0.

#include "carkit_intro2av_cpp/planning_algorithm.hpp"

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace carkit_intro2av_cpp
{

std::vector<geometry_msgs::msg::PoseStamped> compute_path(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const nav_msgs::msg::Odometry & odometry,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlanningConfig & config)
{
  // TODO(Intro2AV C++): Implement grid conversion, obstacle inflation, path
  // search, world-coordinate conversion, orientation, and smoothing here.
  (void)occupancy_grid;
  (void)odometry;
  (void)goal;
  (void)config;
  return {};
}

}  // namespace carkit_intro2av_cpp
