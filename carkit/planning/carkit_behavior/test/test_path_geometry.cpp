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

#include <vector>

#include "carkit_behavior/path_geometry.hpp"
#include "gtest/gtest.h"

namespace cb = carkit_behavior;

TEST(PathGeometry, ProjectsOntoClosestSegment)
{
  const std::vector<cb::Point2d> path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  const auto distance = cb::distance_along_path({2.2, 1.0}, path);
  ASSERT_TRUE(distance);
  EXPECT_NEAR(*distance, 3.0, 1.0e-9);
}

TEST(PathGeometry, RejectsPathWithoutSegment)
{
  EXPECT_FALSE(cb::distance_along_path({0.0, 0.0}, {}).has_value());
}

TEST(PathGeometry, StopRegionIncludesToleranceBehindLine)
{
  EXPECT_TRUE(cb::should_stop_before_line(0.5, 1.0, 0.25));
  EXPECT_TRUE(cb::should_stop_before_line(-0.2, 1.0, 0.25));
  EXPECT_FALSE(cb::should_stop_before_line(-0.3, 1.0, 0.25));
}
