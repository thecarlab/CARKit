// Copyright 2026 University of Delaware
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carkit_behavior/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// CARKit learning annotation: implements the behavior described by this file's package and module.
namespace carkit_behavior
{

/// Project a map point onto the closest path segment and return cumulative arc length.
/// Arc length, rather than Euclidean distance, keeps stop decisions correct on curved paths.
std::optional<double> distance_along_path(
  const Point2d & point, const std::vector<Point2d> & path_points)
{
  if (path_points.size() < 2) {
    return std::nullopt;
  }
  double best_distance_sq = std::numeric_limits<double>::infinity();
  std::optional<double> best_path_distance;
  double path_distance = 0.0;
  for (std::size_t index = 1; index < path_points.size(); ++index) {
    const auto & start = path_points[index - 1];
    const auto & end = path_points[index];
    const double dx = end.first - start.first;
    const double dy = end.second - start.second;
    const double segment_length_sq = dx * dx + dy * dy;
    if (segment_length_sq <= 1.0e-12) {
      continue;
    }
    const double segment_length = std::sqrt(segment_length_sq);
    const double fraction = std::clamp(
      ((point.first - start.first) * dx + (point.second - start.second) * dy) /
      segment_length_sq, 0.0, 1.0);
    const double projected_x = start.first + fraction * dx;
    const double projected_y = start.second + fraction * dy;
    const double error_x = point.first - projected_x;
    const double error_y = point.second - projected_y;
    const double distance_sq = error_x * error_x + error_y * error_y;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_path_distance = path_distance + fraction * segment_length;
    }
    path_distance += segment_length;
  }
  return best_path_distance;
}

/// Accept a small negative distance so a control cycle that crosses the stop line
/// cannot skip the stop state entirely.
bool should_stop_before_line(
  double remaining_distance_m, double stop_before_distance_m,
  double stop_line_tolerance_m)
{
  return remaining_distance_m >= -std::max(0.0, stop_line_tolerance_m) &&
         remaining_distance_m <= stop_before_distance_m;
}

}  // namespace carkit_behavior
