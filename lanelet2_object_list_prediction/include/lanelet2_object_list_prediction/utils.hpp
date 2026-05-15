#pragma once

#include <algorithm>
#include <cmath>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace lanelet2_object_list_prediction {

/**
 * @brief Wraps an angle into a configurable interval
 *
 * @param angle_rad input angle in radians
 * @param min_val lower interval bound in radians
 * @param max_val upper interval bound in radians
 * @return wrapped angle in radians
 */
inline double wrap_angle_rad(double angle_rad, double min_val = -M_PI, double max_val = M_PI) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > max_val) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < min_val) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

/**
 * @brief Computes the centerline heading of a lanelet at an arc length
 *
 * @param lanelet lanelet whose centerline is sampled
 * @param arc_length position along the centerline in meters
 * @return yaw angle of the local centerline tangent in radians
 */
inline double computeLaneletYawAtArcLength(const lanelet::ConstLanelet& lanelet, double arc_length) {
  const lanelet::ConstLineString2d centerline = lanelet.centerline2d();
  const double lanelet_length = lanelet::geometry::length(centerline);
  const double sample_distance = std::min(0.5, std::max(0.01, lanelet_length * 0.1));
  const double before_arc_length = std::max(0.0, arc_length - sample_distance);
  const double after_arc_length = std::min(lanelet_length, arc_length + sample_distance);
  const lanelet::BasicPoint2d before_point = lanelet::geometry::interpolatedPointAtDistance(centerline, before_arc_length);
  const lanelet::BasicPoint2d after_point = lanelet::geometry::interpolatedPointAtDistance(centerline, after_arc_length);
  return std::atan2(after_point.y() - before_point.y(), after_point.x() - before_point.x());
}

}  // namespace lanelet2_object_list_prediction
