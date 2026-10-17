#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

// CARKit learning annotation: declares interfaces implemented by the corresponding source.
namespace carkit_scan_filter
{

/// XY projection of the complete laser-to-base rotation and translation.
/// Keeping the four matrix entries also handles inverted/tilted laser mounts.
struct ScanProjection
{
  double x{0.0}, y{0.0};
  double xx{1.0}, xy{0.0}, yx{0.0}, yy{1.0};
};

inline std::size_t filter_in_base_frame(
  std::vector<float> & ranges, double angle_min, double angle_increment,
  const ScanProjection & transform, double length, double width,
  double center_x, double center_y, double padding)
{
  std::size_t removed = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double distance = ranges[i];
    if (!std::isfinite(distance) || distance < 0.0) {continue;}
    const double angle = angle_min + i * angle_increment;
    const double lx = distance * std::cos(angle), ly = distance * std::sin(angle);
    const double x = transform.x + transform.xx * lx + transform.xy * ly;
    const double y = transform.y + transform.yx * lx + transform.yy * ly;
    if (std::abs(x - center_x) <= length * 0.5 + padding &&
      std::abs(y - center_y) <= width * 0.5 + padding)
    {
      ranges[i] = std::numeric_limits<float>::infinity();
      ++removed;
    }
  }
  return removed;
}

/// Compute the ray-to-rectangle intersection distance for every laser bearing.
/// A return value is the distance at which that ray exits the padded vehicle footprint.
inline std::vector<float> footprint_range_limits(
  std::size_t count,
  double angle_min,
  double angle_increment,
  double vehicle_length,
  double vehicle_width,
  double padding)
{
  const double half_length = vehicle_length * 0.5 + padding;
  const double half_width = vehicle_width * 0.5 + padding;
  std::vector<float> limits;
  limits.reserve(count);

  for (std::size_t index = 0; index < count; ++index) {
    const double angle = angle_min + index * angle_increment;
    const double cosine = std::abs(std::cos(angle));
    const double sine = std::abs(std::sin(angle));
    const double x_limit = cosine > 1e-12 ?
      half_length / cosine : std::numeric_limits<double>::infinity();
    const double y_limit = sine > 1e-12 ?
      half_width / sine : std::numeric_limits<double>::infinity();
    limits.push_back(static_cast<float>(std::min(x_limit, y_limit)));
  }
  return limits;
}

/// Replace finite returns inside the vehicle footprint with infinity so downstream
/// localization and planning never interpret the chassis itself as an obstacle.
inline std::size_t apply_footprint_limits(
  std::vector<float> & ranges, const std::vector<float> & limits)
{
  std::size_t removed = 0;
  const std::size_t count = std::min(ranges.size(), limits.size());
  for (std::size_t index = 0; index < count; ++index) {
    const float distance = ranges[index];
    if (std::isfinite(distance) && distance >= 0.0F && distance <= limits[index]) {
      ranges[index] = std::numeric_limits<float>::infinity();
      ++removed;
    }
  }
  return removed;
}

}  // namespace carkit_scan_filter
