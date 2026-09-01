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

#ifndef CARKIT_BEHAVIOR__PATH_GEOMETRY_HPP_
#define CARKIT_BEHAVIOR__PATH_GEOMETRY_HPP_

#include <optional>
#include <utility>
#include <vector>

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_behavior
{

using Point2d = std::pair<double, double>;

std::optional<double> distance_along_path(
  const Point2d & point, const std::vector<Point2d> & path_points);

bool should_stop_before_line(
  double remaining_distance_m, double stop_before_distance_m,
  double stop_line_tolerance_m = 0.0);

}  // namespace carkit_behavior

#endif  // CARKIT_BEHAVIOR__PATH_GEOMETRY_HPP_
