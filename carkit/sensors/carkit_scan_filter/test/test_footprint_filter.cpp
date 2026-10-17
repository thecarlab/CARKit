#include <cmath>
#include <limits>
#include <vector>

#include "carkit_scan_filter/footprint_filter.hpp"
#include "gtest/gtest.h"

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<float> filter_at_angle(double angle, float distance, double padding = 0.0)
{
  auto limits = carkit_scan_filter::footprint_range_limits(
    1, angle, 0.0, 0.50, 0.25, padding);
  std::vector<float> ranges{distance};
  carkit_scan_filter::apply_footprint_limits(ranges, limits);
  return ranges;
}

TEST(FootprintFilter, RemovesPointsInsideAllFourSides)
{
  EXPECT_TRUE(std::isinf(filter_at_angle(0.0, 0.24F)[0]));
  EXPECT_TRUE(std::isinf(filter_at_angle(kPi, 0.24F)[0]));
  EXPECT_TRUE(std::isinf(filter_at_angle(kPi / 2.0, 0.12F)[0]));
  EXPECT_TRUE(std::isinf(filter_at_angle(-kPi / 2.0, 0.12F)[0]));
}

TEST(FootprintFilter, KeepsPointsOutsideAllFourSides)
{
  EXPECT_FLOAT_EQ(filter_at_angle(0.0, 0.26F)[0], 0.26F);
  EXPECT_FLOAT_EQ(filter_at_angle(kPi, 0.26F)[0], 0.26F);
  EXPECT_FLOAT_EQ(filter_at_angle(kPi / 2.0, 0.13F)[0], 0.13F);
  EXPECT_FLOAT_EQ(filter_at_angle(-kPi / 2.0, 0.13F)[0], 0.13F);
}

TEST(FootprintFilter, UsesRectangleAtDiagonalAngles)
{
  const auto inside = static_cast<float>(std::hypot(0.20, 0.10));
  const auto outside = static_cast<float>(std::hypot(0.20, 0.13));
  EXPECT_TRUE(std::isinf(filter_at_angle(std::atan2(0.10, 0.20), inside)[0]));
  EXPECT_FLOAT_EQ(filter_at_angle(std::atan2(0.13, 0.20), outside)[0], outside);
}

TEST(FootprintFilter, PreservesInvalidReadings)
{
  auto limits = carkit_scan_filter::footprint_range_limits(3, 0.0, 0.1, 0.50, 0.25, 0.0);
  std::vector<float> ranges{
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(),
    -1.0F};
  EXPECT_EQ(carkit_scan_filter::apply_footprint_limits(ranges, limits), 0U);
  EXPECT_TRUE(std::isinf(ranges[0]));
  EXPECT_TRUE(std::isnan(ranges[1]));
  EXPECT_FLOAT_EQ(ranges[2], -1.0F);
}

TEST(FootprintFilter, OptionalPaddingExpandsFootprint)
{
  EXPECT_TRUE(std::isinf(filter_at_angle(0.0, 0.26F, 0.02)[0]));
}

TEST(FootprintFilter, OffsetLaserPreservesObstacleBehindCarAndRemovesBodyAhead)
{
  carkit_scan_filter::ScanProjection transform;
  transform.x = -0.082558;
  transform.y = -0.017229;
  // A rear return at 0.20 m is outside the car, despite being near the laser.
  std::vector<float> rear{0.20F};
  carkit_scan_filter::filter_in_base_frame(rear, kPi, 0, transform, .5, .25, 0, 0, 0);
  EXPECT_FLOAT_EQ(rear[0], 0.20F);
  // A front return at 0.30 m is still inside the car in base coordinates.
  std::vector<float> front{0.30F};
  carkit_scan_filter::filter_in_base_frame(front, 0, 0, transform, .5, .25, 0, 0, 0);
  EXPECT_TRUE(std::isinf(front[0]));
}

TEST(FootprintFilter, RotatedAndInvertedMountUsesBaseFootprint)
{
  // Laser +X points along base +Y; +Y points along base +X (upside-down mount).
  carkit_scan_filter::ScanProjection transform{0, 0, 0, 1, 1, 0};
  std::vector<float> side{.20F};
  carkit_scan_filter::filter_in_base_frame(side, 0, 0, transform, .5, .25, 0, 0, 0);
  EXPECT_FLOAT_EQ(side[0], .20F);
  std::vector<float> front{.20F};
  carkit_scan_filter::filter_in_base_frame(front, kPi/2, 0, transform, .5, .25, 0, 0, 0);
  EXPECT_TRUE(std::isinf(front[0]));
}

TEST(FootprintFilter, RearAxleOriginAndOutsideSensorOnlyRemovePointsInsideBody)
{
  carkit_scan_filter::ScanProjection transform{.6, 0, -1, 0, 0, -1};
  std::vector<float> ranges{.1F, .3F};
  carkit_scan_filter::filter_in_base_frame(ranges, 0, 0, transform, .485, .27, .1925, 0, 0);
  EXPECT_FLOAT_EQ(ranges[0], .1F);  // x=0.5: ahead of bumper, preserve.
  EXPECT_TRUE(std::isinf(ranges[1]));  // x=0.3: within body.
}

}  // namespace
