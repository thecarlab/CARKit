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

}  // namespace
