// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2/exceptions.h>
#include <lanelet2_object_list_prediction/lanelet2_object_list_prediction.hpp>
#include <lanelet2_object_list_prediction/utils.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <tf2/time.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

namespace lanelet2_object_list_prediction {

namespace {

constexpr double kRouteProfileResolutionM = 0.25;
constexpr double kCurvatureSampleDistanceM = 1.0;
constexpr double kCurvatureHalfBaselineM = 4.0;
constexpr int kHeadingMedianFilterHalfWidth = 6;
constexpr double kHeadingMedianFilterSpacingM = 1.0;
constexpr double kKinematicEpsilon = 1e-6;
constexpr double kMotionDirectionMinSpeedMps = 0.3;
constexpr double kMinAlongLaneSpeedFraction = 0.25;
constexpr double kMotionTangentSampleDistanceM = 0.05;
constexpr double kRoundaboutLookaheadM = 5.0;

bool isBicycleClass(uint8_t type) {
  return type == perception_msgs::msg::ObjectClassification::BICYCLE ||
         type == perception_msgs::msg::ObjectClassification::BIKE_UNION ||
         type == perception_msgs::msg::ObjectClassification::MICRO;
}

bool isPedestrianClass(uint8_t type) {
  return type == perception_msgs::msg::ObjectClassification::PEDESTRIAN ||
         type == perception_msgs::msg::ObjectClassification::VRU;
}

bool isMotorcycleClass(uint8_t type) { return type == perception_msgs::msg::ObjectClassification::MOTORCYCLE; }

bool isPedestrianLane(const lanelet::ConstLanelet& lanelet) {
  const auto subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{});
  return subtype == static_cast<const char*>(lanelet::AttributeValueString::Walkway) ||
         subtype == static_cast<const char*>(lanelet::AttributeValueString::SharedWalkway) ||
         subtype == static_cast<const char*>(lanelet::AttributeValueString::Crosswalk);
}

bool isBicycleLane(const lanelet::ConstLanelet& lanelet) {
  return lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{}) ==
         static_cast<const char*>(lanelet::AttributeValueString::BicycleLane);
}

bool isRoundabout(const lanelet::ConstLanelet& lanelet) {
  return lanelet.attributeOr(static_cast<const char*>("intersection_type"), std::string{}) ==
         static_cast<const char*>("roundabout");
}

struct LaneletVelocity {
  double longitudinal{0.0};
  double lateral{0.0};
};

LaneletVelocity velocityAlongLanelet(const geometry_msgs::msg::Vector3& velocity, double yaw) {
  return {velocity.x * std::cos(yaw) + velocity.y * std::sin(yaw), -velocity.x * std::sin(yaw) + velocity.y * std::cos(yaw)};
}

bool initialMotionFeasible(const geometry_msgs::msg::Point& observed_position,
                           const geometry_msgs::msg::Vector3& observed_velocity,
                           const geometry_msgs::msg::Point& predicted_position,
                           double sample_interval,
                           double max_lateral_acceleration,
                           double max_longitudinal_acceleration,
                           double max_longitudinal_deceleration,
                           double alignment_tolerance = 0.0) {
  // A constant acceleration reaching the first position would need this
  // change in velocity. Check it in the observed direction of travel.
  const double scale = 2.0 / (sample_interval * sample_interval);
  const double acceleration_x = (predicted_position.x - observed_position.x - observed_velocity.x * sample_interval) * scale;
  const double acceleration_y = (predicted_position.y - observed_position.y - observed_velocity.y * sample_interval) * scale;
  const double speed = std::hypot(observed_velocity.x, observed_velocity.y);
  if (speed <= kMotionDirectionMinSpeedMps) {
    const double excess =
        std::max(0.0, std::hypot(acceleration_x, acceleration_y) -
                          std::max({max_lateral_acceleration, max_longitudinal_acceleration, max_longitudinal_deceleration}));
    return excess / scale <= alignment_tolerance + kKinematicEpsilon;
  }
  const double longitudinal = (acceleration_x * observed_velocity.x + acceleration_y * observed_velocity.y) / speed;
  const double lateral = (acceleration_y * observed_velocity.x - acceleration_x * observed_velocity.y) / speed;
  const double longitudinal_excess =
      std::max({0.0, longitudinal - max_longitudinal_acceleration, -longitudinal - max_longitudinal_deceleration});
  const double lateral_excess = std::max(0.0, std::abs(lateral) - max_lateral_acceleration);
  return std::hypot(longitudinal_excess, lateral_excess) / scale <= alignment_tolerance + kKinematicEpsilon;
}

struct YieldConstraint {
  double stop_distance{0.0};
  double release_time{0.0};
};

struct TimeInterval {
  double entry{0.0};
  double exit{0.0};
};

// The start position can lie beyond the first lanelet, e.g. with a preceding lanelet added for its shape.
double remainingRouteLength(const lanelet::routing::LaneletPath& route, double start_arc_length) {
  double length = -start_arc_length;
  for (const lanelet::ConstLanelet& lanelet : route) {
    length += static_cast<double>(lanelet::geometry::length(lanelet.centerline2d()));
  }
  return std::max(0.0, length);
}

/** Routes against the legal direction of one-way bicycle lanes, following their
 * legal predecessors backwards. `start` is the inverted lanelet the bicycle is on. */
lanelet::routing::LaneletPaths wrongWayBicyclePaths(const lanelet::ConstLanelet& start,
                                                    double distance_after_start,
                                                    const lanelet::routing::RoutingGraph& routing_graph) {
  constexpr std::size_t kMaxPaths = 8;
  lanelet::routing::LaneletPaths paths;
  std::function<void(lanelet::ConstLanelets&, double)> extend = [&](lanelet::ConstLanelets& path, double remaining) {
    bool extended = false;
    if (remaining > 0.0) {
      for (const lanelet::ConstLanelet& previous : routing_graph.previous(path.back().invert(), false)) {
        if (paths.size() >= kMaxPaths) break;
        const bool visited =
            std::any_of(path.begin(), path.end(), [&](const auto& lanelet) { return lanelet.id() == previous.id(); });
        if (!isBicycleLane(previous) || visited) continue;
        extended = true;
        path.push_back(previous.invert());
        extend(path, remaining - static_cast<double>(lanelet::geometry::length(previous.centerline2d())));
        path.pop_back();
      }
    }
    if (!extended && paths.size() < kMaxPaths) paths.emplace_back(path);
  };
  lanelet::ConstLanelets path{start};
  extend(path, distance_after_start);
  return paths;
}

/** Route geometry including the lanelet preceding the route, for shape only.
 * Heading filtering and curvature also sample behind the object; without the
 * preceding lanelet, an object entering a turning lanelet gets the heading of
 * the turn ahead instead of its approach. Distances along the route stay
 * relative to the object. */
std::pair<lanelet::routing::LaneletPath, double> routeGeometryWithPredecessor(
    const lanelet::routing::LaneletPath& route, double start_arc_length, const lanelet::routing::RoutingGraph& routing_graph) {
  const double lookbehind = kHeadingMedianFilterHalfWidth * kHeadingMedianFilterSpacingM + kCurvatureHalfBaselineM;
  if (route.empty() || start_arc_length >= lookbehind) return {route, start_arc_length};
  const lanelet::ConstLanelets predecessors = routing_graph.previous(route.front(), false);
  if (predecessors.empty()) return {route, start_arc_length};
  const double start_yaw = computeLaneletYawAtArcLength(route.front(), 0.0);
  const auto yaw_difference = [&](const lanelet::ConstLanelet& predecessor) {
    const double length = static_cast<double>(lanelet::geometry::length(predecessor.centerline2d()));
    return std::abs(wrap_angle_rad(computeLaneletYawAtArcLength(predecessor, length) - start_yaw));
  };
  const lanelet::ConstLanelet& predecessor =
      *std::min_element(predecessors.begin(), predecessors.end(),
                        [&](const auto& lhs, const auto& rhs) { return yaw_difference(lhs) < yaw_difference(rhs); });
  lanelet::ConstLanelets lanelets{predecessor};
  lanelets.insert(lanelets.end(), route.begin(), route.end());
  return {lanelet::routing::LaneletPath(lanelets),
          start_arc_length + static_cast<double>(lanelet::geometry::length(predecessor.centerline2d()))};
}

bool startsNearRoundabout(const lanelet::routing::LaneletPath& route, double start_arc_length) {
  double distance_to_lanelet = 0.0;
  for (std::size_t index = 0; index < route.size() && distance_to_lanelet <= kRoundaboutLookaheadM; ++index) {
    if (isRoundabout(route[index])) return true;
    const double lanelet_length = static_cast<double>(lanelet::geometry::length(route[index].centerline2d()));
    distance_to_lanelet += index == 0 ? std::max(0.0, lanelet_length - start_arc_length) : lanelet_length;
  }
  return false;
}

// Negative travel distances reach back on the first lanelet, behind the object.
lanelet::BasicPoint2d pointOnRoute(const lanelet::routing::LaneletPath& route, double start_arc_length, double travel_distance) {
  double distance_on_route = start_arc_length + std::max(-start_arc_length, travel_distance);
  for (std::size_t route_index = 0; route_index < route.size(); ++route_index) {
    const lanelet::ConstLineString2d centerline = route[route_index].centerline2d();
    const double lanelet_length = static_cast<double>(lanelet::geometry::length(centerline));
    if (distance_on_route > lanelet_length && route_index + 1 < route.size()) {
      distance_on_route -= lanelet_length;
      continue;
    }
    return lanelet::geometry::interpolatedPointAtDistance(centerline, std::clamp(distance_on_route, 0.0, lanelet_length));
  }

  const lanelet::ConstLineString2d centerline = route.back().centerline2d();
  const double centerline_length = static_cast<double>(lanelet::geometry::length(centerline));
  return lanelet::geometry::interpolatedPointAtDistance(centerline, centerline_length);
}

double convergingLateralOffset(double initial_offset,
                               double travel_distance,
                               double convergence_distance,
                               double initial_slope = 0.0) {
  if (convergence_distance <= kKinematicEpsilon) return 0.0;
  if (!std::isfinite(convergence_distance)) return initial_offset;
  const double progress = std::clamp(travel_distance / convergence_distance, 0.0, 1.0);
  const double smoothstep = progress * progress * (3.0 - 2.0 * progress);
  return initial_offset * (1.0 - smoothstep) +
         convergence_distance * initial_slope * progress * (1.0 - progress) * (1.0 - progress);
}

double lateralConvergenceDistance(double initial_offset,
                                  double longitudinal_speed,
                                  double max_lateral_acceleration,
                                  double lateral_speed = 0.0) {
  if (std::abs(initial_offset) <= kKinematicEpsilon && std::abs(lateral_speed) <= kKinematicEpsilon) return 0.0;
  if (longitudinal_speed <= kKinematicEpsilon) return std::numeric_limits<double>::infinity();
  const double initial_slope = lateral_speed / longitudinal_speed;
  double distance = std::max(kRouteProfileResolutionM,
                             longitudinal_speed * std::sqrt(6.0 * std::abs(initial_offset) / max_lateral_acceleration));
  // This cubic reaches its greatest lateral acceleration at one of its endpoints.
  while (true) {
    const double first = -6.0 * initial_offset / (distance * distance) - 4.0 * initial_slope / distance;
    const double last = 6.0 * initial_offset / (distance * distance) + 2.0 * initial_slope / distance;
    if (std::max(std::abs(first), std::abs(last)) * longitudinal_speed * longitudinal_speed <=
        max_lateral_acceleration + kKinematicEpsilon) {
      return distance;
    }
    distance *= 2.0;
  }
}

RouteMotionProfile buildRouteMotionProfile(const lanelet::routing::LaneletPath& route,
                                           double start_arc_length,
                                           double initial_speed,
                                           double max_lateral_acceleration,
                                           double max_longitudinal_acceleration,
                                           double max_longitudinal_deceleration,
                                           bool stop_at_route_end,
                                           const std::optional<YieldConstraint>& yield_constraint = std::nullopt,
                                           bool enable_kinematic_limitations = true) {
  const SmoothedRoute smoothed_route(route, start_arc_length);
  const double route_length = smoothed_route.length();
  std::vector<double> distances;
  const std::size_t regular_segment_count = static_cast<std::size_t>(std::ceil(route_length / kRouteProfileResolutionM));
  distances.reserve(regular_segment_count + 2);
  for (std::size_t index = 0; index < regular_segment_count; ++index) {
    distances.push_back(std::min(route_length, static_cast<double>(index) * kRouteProfileResolutionM));
  }
  distances.push_back(route_length);
  if (yield_constraint.has_value()) {
    distances.push_back(std::clamp(yield_constraint->stop_distance, 0.0, route_length));
  }
  std::sort(distances.begin(), distances.end());
  distances.erase(std::unique(distances.begin(), distances.end(),
                              [](double lhs, double rhs) { return std::abs(lhs - rhs) < kKinematicEpsilon; }),
                  distances.end());

  const std::size_t segment_count = distances.size() - 1;
  RouteMotionProfile profile;
  profile.samples.resize(segment_count + 1);
  std::vector<double> curve_speed_limits(segment_count + 1, initial_speed);
  std::optional<std::size_t> stop_index;

  for (std::size_t index = 0; index <= segment_count; ++index) {
    const double distance = distances[index];
    profile.samples[index].distance = distance;
    const double curvature = smoothed_route.curvature(distance);
    if (enable_kinematic_limitations && curvature > kKinematicEpsilon) {
      curve_speed_limits[index] = std::min(initial_speed, std::sqrt(max_lateral_acceleration / curvature));
    }
  }
  std::vector<double> speed_limits = curve_speed_limits;
  if (stop_at_route_end) speed_limits.back() = 0.0;
  if (yield_constraint.has_value()) {
    const double stop_distance = std::clamp(yield_constraint->stop_distance, 0.0, route_length);
    stop_index = static_cast<std::size_t>(
        std::distance(distances.begin(), std::find_if(distances.begin(), distances.end(), [stop_distance](double distance) {
                        return std::abs(distance - stop_distance) < kKinematicEpsilon;
                      })));
    speed_limits[*stop_index] = 0.0;
  }

  const double effective_deceleration =
      enable_kinematic_limitations ? max_longitudinal_deceleration : std::numeric_limits<double>::infinity();
  const double effective_acceleration =
      enable_kinematic_limitations ? max_longitudinal_acceleration : std::numeric_limits<double>::infinity();

  // Propagate curve speed limits backwards so braking starts early enough.
  for (std::size_t index = segment_count; index > 0; --index) {
    const double distance = profile.samples[index].distance - profile.samples[index - 1].distance;
    const double reachable_speed = std::sqrt(speed_limits[index] * speed_limits[index] + 2.0 * effective_deceleration * distance);
    speed_limits[index - 1] = std::min(speed_limits[index - 1], reachable_speed);
  }

  profile.samples.front().speed = initial_speed;
  if (enable_kinematic_limitations && initial_speed > speed_limits.front() + kKinematicEpsilon) profile.feasible = false;
  for (std::size_t index = 1; index <= segment_count; ++index) {
    const double distance = profile.samples[index].distance - profile.samples[index - 1].distance;
    const double previous_speed = profile.samples[index - 1].speed;
    const double minimum_reachable_speed =
        std::sqrt(std::max(0.0, previous_speed * previous_speed - 2.0 * effective_deceleration * distance));
    const double maximum_reachable_speed = std::sqrt(previous_speed * previous_speed + 2.0 * effective_acceleration * distance);
    const double desired_speed = std::min(initial_speed, speed_limits[index]);
    profile.samples[index].speed = std::clamp(desired_speed, minimum_reachable_speed, maximum_reachable_speed);
    if (enable_kinematic_limitations && profile.samples[index].speed > speed_limits[index] + kKinematicEpsilon) {
      profile.feasible = false;
    }

    const double average_speed = 0.5 * (previous_speed + profile.samples[index].speed);
    profile.samples[index].time = average_speed > kKinematicEpsilon ? profile.samples[index - 1].time + distance / average_speed
                                                                    : std::numeric_limits<double>::infinity();
  }

  if (stop_index.has_value() && profile.samples[*stop_index].speed <= kKinematicEpsilon) {
    const double arrival_time = profile.samples[*stop_index].time;
    const double release_time = std::max(arrival_time, yield_constraint->release_time);
    if (release_time > arrival_time + kKinematicEpsilon) {
      RouteMotionSample wait_sample = profile.samples[*stop_index];
      wait_sample.time = release_time;
      profile.samples.insert(profile.samples.begin() + static_cast<std::ptrdiff_t>(*stop_index + 1), wait_sample);
      for (std::size_t index = *stop_index + 2; index < profile.samples.size(); ++index) {
        const double distance = profile.samples[index].distance - profile.samples[index - 1].distance;
        const double average_speed = 0.5 * (profile.samples[index - 1].speed + profile.samples[index].speed);
        profile.samples[index].time = average_speed > kKinematicEpsilon
                                          ? profile.samples[index - 1].time + distance / average_speed
                                          : std::numeric_limits<double>::infinity();
      }
    }
  }
  return profile;
}

/** Describes the curve that requires the hardest braking from the initial speed, for debugging infeasible profiles. */
std::string criticalCurveDescription(const lanelet::routing::LaneletPath& route,
                                     double start_arc_length,
                                     double initial_speed,
                                     double max_lateral_acceleration) {
  const SmoothedRoute smoothed_route(route, start_arc_length);
  double worst_deceleration = 0.0;
  std::ostringstream description;
  const auto sample_count = static_cast<std::size_t>(smoothed_route.length() / kRouteProfileResolutionM);
  for (std::size_t index = 0; index <= sample_count; ++index) {
    const double distance = static_cast<double>(index) * kRouteProfileResolutionM;
    const double curvature = smoothed_route.curvature(distance);
    if (curvature <= kKinematicEpsilon) continue;
    const double speed_limit = std::sqrt(max_lateral_acceleration / curvature);
    const double deceleration =
        (initial_speed * initial_speed - speed_limit * speed_limit) / (2.0 * std::max(distance, kRouteProfileResolutionM));
    if (deceleration <= worst_deceleration) continue;
    worst_deceleration = deceleration;
    description.str("");
    description << "radius " << 1.0 / curvature << " m at " << distance << " m allows " << speed_limit << " m/s, needs "
                << deceleration << " m/s^2 braking";
  }
  return description.str();
}

double timeAtRouteDistance(const std::vector<RouteMotionSample>& profile, double target_distance) {
  if (profile.empty()) return std::numeric_limits<double>::infinity();
  const auto upper =
      std::lower_bound(profile.begin(), profile.end(), target_distance,
                       [](const RouteMotionSample& sample, double distance) { return sample.distance < distance; });
  if (upper == profile.begin()) return upper->time;
  if (upper == profile.end()) return profile.back().time;
  const RouteMotionSample& previous = *(upper - 1);
  const double distance = upper->distance - previous.distance;
  if (distance < kKinematicEpsilon) return upper->time;
  const double ratio = std::clamp((target_distance - previous.distance) / distance, 0.0, 1.0);
  return previous.time + ratio * (upper->time - previous.time);
}

double routeDistanceAtLaneletStart(const lanelet::routing::LaneletPath& route,
                                   double start_arc_length,
                                   std::size_t lanelet_index) {
  double distance = -start_arc_length;
  for (std::size_t index = 0; index < lanelet_index; ++index) {
    distance += static_cast<double>(lanelet::geometry::length(route[index].centerline2d()));
  }
  return distance;
}

struct YieldLineProjection {
  double distance{0.0};
  std::size_t lanelet_index{0};
};

std::optional<YieldLineProjection> projectYieldLineOnRoute(const lanelet::routing::LaneletPath& route,
                                                           double start_arc_length,
                                                           std::size_t first_lanelet_index,
                                                           const lanelet::ConstLineString3d& stop_line) {
  if (stop_line.empty()) return std::nullopt;
  lanelet::BasicPoint2d line_center(0.0, 0.0);
  for (const lanelet::ConstPoint3d& point : stop_line) {
    line_center += lanelet::BasicPoint2d(point.x(), point.y());
  }
  line_center /= static_cast<double>(stop_line.size());

  double nearest_distance = std::numeric_limits<double>::infinity();
  std::optional<YieldLineProjection> nearest;
  for (std::size_t index = first_lanelet_index; index < route.size(); ++index) {
    const lanelet::ConstLineString2d centerline = route[index].centerline2d();
    const double length = static_cast<double>(lanelet::geometry::length(centerline));
    const double arc = std::clamp(lanelet::geometry::toArcCoordinates(centerline, line_center).length, 0.0, length);
    const lanelet::BasicPoint2d route_point = lanelet::geometry::interpolatedPointAtDistance(centerline, arc);
    const double distance = (route_point - line_center).norm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = YieldLineProjection{routeDistanceAtLaneletStart(route, start_arc_length, index) + arc, index};
    }
  }
  // A reference line beyond this route belongs to a different branch or is
  // not reached by the hypothesis. A line behind the object has been passed.
  if (!nearest.has_value() || nearest_distance > 5.0 || nearest->distance < -kKinematicEpsilon) return std::nullopt;
  return nearest;
}

bool laneletConflictsWithPriorityContinuation(const lanelet::ConstLanelet& route_lanelet,
                                              const lanelet::RightOfWay& right_of_way,
                                              const lanelet::routing::RoutingGraph& routing_graph) {
  std::unordered_set<lanelet::Id> priority_path_ids;
  for (const lanelet::ConstLanelet& priority_lanelet : right_of_way.rightOfWayLanelets()) {
    priority_path_ids.insert(priority_lanelet.id());
    for (const lanelet::ConstLanelet& following : routing_graph.following(priority_lanelet)) {
      priority_path_ids.insert(following.id());
    }
  }
  const auto conflicts = routing_graph.conflicting(route_lanelet);
  return std::any_of(conflicts.begin(), conflicts.end(), [&](const lanelet::ConstLaneletOrArea& conflict) {
    return conflict.isLanelet() && priority_path_ids.count(conflict.id()) > 0;
  });
}

std::optional<std::size_t> conflictLaneletAtYieldLine(const lanelet::routing::LaneletPath& route,
                                                      std::size_t regulating_lanelet_index,
                                                      std::size_t line_lanelet_index,
                                                      const lanelet::RightOfWay& right_of_way,
                                                      const lanelet::routing::RoutingGraph& routing_graph) {
  if (laneletConflictsWithPriorityContinuation(route[line_lanelet_index], right_of_way, routing_graph)) {
    return line_lanelet_index;
  }
  // The line can lie on a shared approach, before the route branches.
  if (line_lanelet_index == regulating_lanelet_index && line_lanelet_index + 1 < route.size() &&
      laneletConflictsWithPriorityContinuation(route[line_lanelet_index + 1], right_of_way, routing_graph)) {
    return line_lanelet_index + 1;
  }
  return std::nullopt;
}

bool intervalsOverlap(const TimeInterval& lhs, const TimeInterval& rhs) {
  return lhs.entry <= rhs.exit + kKinematicEpsilon && rhs.entry <= lhs.exit + kKinematicEpsilon;
}

// The yielding participant needs the clearance gap on both sides: leaving the
// conflict just before priority traffic arrives is no acceptable gap either.
bool conflictsDuringClearance(const TimeInterval& yielding, const TimeInterval& priority, double clearance_time) {
  return intervalsOverlap(yielding, TimeInterval{priority.entry - clearance_time, priority.exit + clearance_time});
}

std::optional<TimeInterval> egoRouteOccupancy(const perception_msgs::msg::EgoData& ego_data,
                                              const rclcpp::Time& base_stamp,
                                              double horizon,
                                              const std::vector<lanelet::ConstLanelet>& priority_lanelets,
                                              const lanelet::ConstLanelet& conflict_lanelet) {
  struct TimedPosition {
    double time;
    lanelet::BasicPoint2d position;
  };
  std::vector<TimedPosition> positions;
  auto append = [&](const perception_msgs::msg::ObjectState& state) {
    try {
      const geometry_msgs::msg::Point point = perception_msgs::object_access::getPosition(state);
      const double time = (rclcpp::Time(state.header.stamp) - base_stamp).seconds();
      if (std::isfinite(time) && std::isfinite(point.x) && std::isfinite(point.y)) {
        positions.push_back({time, lanelet::BasicPoint2d(point.x, point.y)});
      }
    } catch (const std::exception&) {
    }
  };
  append(ego_data.state);
  for (const auto& state : ego_data.trajectory_planned) append(state);
  std::sort(positions.begin(), positions.end(), [](const auto& lhs, const auto& rhs) { return lhs.time < rhs.time; });

  // A short or braking plan can end before the priority lanelets although ego
  // keeps its right of way. Continue it straight at no less than the current
  // speed, so a plan braking for a crossing object cannot hide ego from it.
  if (positions.size() >= 2 && positions.back().time < horizon - kKinematicEpsilon) {
    const TimedPosition& last = positions.back();
    const TimedPosition& previous = positions[positions.size() - 2];
    const lanelet::BasicPoint2d motion = last.position - previous.position;
    const double dt = last.time - previous.time;
    double current_speed = 0.0;
    try {
      const auto velocity = perception_msgs::object_access::getVelocityXYZ(ego_data.state);
      current_speed = std::hypot(velocity.x, velocity.y);
    } catch (const std::exception&) {
    }
    const double final_speed = dt > kKinematicEpsilon ? motion.norm() / dt : 0.0;
    double yaw = std::atan2(motion.y(), motion.x());
    if (motion.norm() <= kKinematicEpsilon) {
      try {
        yaw = perception_msgs::object_access::getYaw(ego_data.state);
      } catch (const std::exception&) {
      }
    }
    const double extrapolation = (horizon - last.time) * std::max(current_speed, final_speed);
    positions.push_back({horizon, last.position + extrapolation * lanelet::BasicPoint2d(std::cos(yaw), std::sin(yaw))});
  }

  std::optional<TimeInterval> priority_interval;
  std::optional<TimeInterval> conflict_interval;
  auto include = [](std::optional<TimeInterval>& interval, double time) {
    if (!interval) interval = TimeInterval{time, time};
    interval->entry = std::min(interval->entry, time);
    interval->exit = std::max(interval->exit, time);
  };
  auto evaluate = [&](double time, const lanelet::BasicPoint2d& point) {
    if (time < -kKinematicEpsilon || time > horizon + kKinematicEpsilon) return;
    if (std::any_of(priority_lanelets.begin(), priority_lanelets.end(),
                    [&](const auto& lanelet) { return lanelet::geometry::inside(lanelet, point); })) {
      include(priority_interval, time);
    }
    if (lanelet::geometry::inside(conflict_lanelet, point)) include(conflict_interval, time);
  };
  for (std::size_t index = 0; index < positions.size(); ++index) {
    evaluate(positions[index].time, positions[index].position);
    if (index + 1 == positions.size()) continue;
    const auto& from = positions[index];
    const auto& to = positions[index + 1];
    const double distance = (to.position - from.position).norm();
    const std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(distance / kRouteProfileResolutionM)));
    for (std::size_t step = 1; step < steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      evaluate(from.time + fraction * (to.time - from.time), from.position + fraction * (to.position - from.position));
    }
  }
  if (!priority_interval) return std::nullopt;
  if (conflict_interval && conflict_interval->exit >= priority_interval->entry) {
    priority_interval->exit = std::max(priority_interval->exit, conflict_interval->exit);
  }
  return priority_interval;
}

struct TimedRoutePose {
  double time{0.0};
  lanelet::BasicPoint2d position{0.0, 0.0};
  double yaw{0.0};
};

std::optional<TimedRoutePose> interpolatePose(const std::vector<TimedRoutePose>& poses, double time) {
  if (poses.empty() || time < poses.front().time - kKinematicEpsilon) return std::nullopt;
  if (time <= poses.front().time) return poses.front();
  if (time >= poses.back().time) return poses.back();
  const auto upper = std::upper_bound(poses.begin(), poses.end(), time,
                                      [](double value, const TimedRoutePose& pose) { return value < pose.time; });
  const TimedRoutePose& before = *(upper - 1);
  const TimedRoutePose& after = *upper;
  const double fraction = (time - before.time) / (after.time - before.time);
  return TimedRoutePose{time, before.position + fraction * (after.position - before.position),
                        before.yaw + fraction * std::remainder(after.yaw - before.yaw, 2.0 * M_PI)};
}

std::optional<double> projectLeaderOnRoute(const lanelet::routing::LaneletPath& route,
                                           double start_arc_length,
                                           const lanelet::BasicPoint2d& position,
                                           double yaw,
                                           const std::optional<lanelet::Id>& required_lanelet_id) {
  std::optional<double> distance;
  for (std::size_t index = 0; index < route.size(); ++index) {
    const lanelet::ConstLanelet& lanelet = route[index];
    if (required_lanelet_id && lanelet.id() != *required_lanelet_id) continue;
    if (!lanelet::geometry::inside(lanelet, position)) continue;
    const auto arc = lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), position);
    const double lane_yaw = computeLaneletYawAtArcLength(lanelet, arc.length);
    if (std::abs(wrap_angle_rad(yaw - lane_yaw)) > M_PI / 4.0) continue;
    const double projected = routeDistanceAtLaneletStart(route, start_arc_length, index) + arc.length;
    if (!distance || projected < *distance) distance = projected;
  }
  return distance;
}

std::pair<std::size_t, double> laneletAtRouteDistance(const lanelet::routing::LaneletPath& route,
                                                      double start_arc_length,
                                                      double travel_distance) {
  double distance = start_arc_length + travel_distance;
  for (std::size_t index = 0; index < route.size(); ++index) {
    const double length = static_cast<double>(lanelet::geometry::length(route[index].centerline2d()));
    if (distance <= length || index + 1 == route.size()) return {index, std::clamp(distance, 0.0, length)};
    distance -= length;
  }
  return {0, 0.0};
}

double profileSpeedAtDistance(const std::vector<RouteMotionSample>& profile, double distance) {
  const auto upper = std::lower_bound(profile.begin(), profile.end(), distance,
                                      [](const RouteMotionSample& sample, double value) { return sample.distance < value; });
  if (upper == profile.begin()) return upper->speed;
  if (upper == profile.end()) return profile.back().speed;
  const RouteMotionSample& before = *(upper - 1);
  const double fraction = (distance - before.distance) / (upper->distance - before.distance);
  return before.speed + fraction * (upper->speed - before.speed);
}

struct FollowingStepResult {
  RouteMotionSample sample;
  bool feasible{true};
};

FollowingStepResult advanceFollowingStep(const RouteMotionSample& previous,
                                         double next_time,
                                         double nominal_distance,
                                         double speed_limit,
                                         double leader_distance_limit,
                                         double headway,
                                         double initial_speed,
                                         double max_acceleration,
                                         double max_deceleration,
                                         double route_length) {
  const double dt = next_time - previous.time;
  const double minimum_speed = std::max(0.0, previous.speed - max_deceleration * dt);
  const double maximum_speed = std::min(initial_speed, previous.speed + max_acceleration * dt);
  const double speed = std::clamp(speed_limit, minimum_speed, maximum_speed);
  const double distance = std::min(route_length, previous.distance + 0.5 * (previous.speed + speed) * dt);
  return {{distance, speed, next_time},
          distance + headway * speed <= leader_distance_limit + 1e-3 && distance <= nominal_distance + 1e-3};
}

bool isEgoDataTimestampUsable(double ego_age, double timeout) { return std::abs(ego_age) <= timeout + kKinematicEpsilon; }

RouteMotionSample sampleRouteMotionAtTime(const std::vector<RouteMotionSample>& profile, double target_time) {
  const auto upper = std::lower_bound(profile.begin(), profile.end(), target_time,
                                      [](const RouteMotionSample& sample, double time) { return sample.time < time; });
  if (upper == profile.begin()) return *upper;
  if (upper == profile.end()) return profile.back();

  const RouteMotionSample& previous = *(upper - 1);
  const double segment_duration = upper->time - previous.time;
  if (!std::isfinite(segment_duration) || segment_duration < kKinematicEpsilon) return previous;

  const double elapsed = std::clamp(target_time - previous.time, 0.0, segment_duration);
  const double acceleration = (upper->speed - previous.speed) / segment_duration;
  RouteMotionSample sample;
  sample.time = target_time;
  sample.speed = std::max(0.0, previous.speed + acceleration * elapsed);
  sample.distance = previous.distance + previous.speed * elapsed + 0.5 * acceleration * elapsed * elapsed;
  return sample;
}

}  // namespace

SmoothedRoute::SmoothedRoute(lanelet::routing::LaneletPath route, double start_arc_length)
    : route_(std::move(route)), start_arc_length_(start_arc_length), length_(remainingRouteLength(route_, start_arc_length)) {
  grid_points_.push_back(route_.empty() ? lanelet::BasicPoint2d(0.0, 0.0) : pointOnRoute(route_, start_arc_length_, 0.0));
}

double SmoothedRoute::rawYaw(double travel_distance) const {
  if (route_.empty()) return 0.0;
  const auto [cached, inserted] = raw_yaws_.try_emplace(travel_distance, 0.0);
  if (!inserted) return cached->second;
  const double before_distance = std::max(-start_arc_length_, travel_distance - kCurvatureSampleDistanceM);
  const double after_distance = std::min(length_, travel_distance + kCurvatureSampleDistanceM);
  const lanelet::BasicPoint2d before = pointOnRoute(route_, start_arc_length_, before_distance);
  const lanelet::BasicPoint2d after = pointOnRoute(route_, start_arc_length_, after_distance);
  cached->second = std::atan2(after.y() - before.y(), after.x() - before.x());
  return cached->second;
}

double SmoothedRoute::yaw(double travel_distance) const {
  const auto [cached, inserted] = yaws_.try_emplace(travel_distance, 0.0);
  if (!inserted) return cached->second;
  const double reference_yaw = rawYaw(travel_distance);
  std::vector<double> yaw_samples;
  yaw_samples.reserve(2 * kHeadingMedianFilterHalfWidth + 1);
  // The window also covers the first lanelet behind the object. Looking only
  // ahead, an object already in a turn gets the heading of the turn's rest.
  for (int offset_index = -kHeadingMedianFilterHalfWidth; offset_index <= kHeadingMedianFilterHalfWidth; ++offset_index) {
    const double sample_distance = travel_distance + static_cast<double>(offset_index) * kHeadingMedianFilterSpacingM;
    if (sample_distance < -start_arc_length_ || sample_distance > length_) continue;
    yaw_samples.push_back(reference_yaw + std::remainder(rawYaw(sample_distance) - reference_yaw, 2.0 * M_PI));
  }
  if (yaw_samples.empty()) {
    cached->second = reference_yaw;
    return reference_yaw;
  }

  std::sort(yaw_samples.begin(), yaw_samples.end());
  const std::size_t middle = yaw_samples.size() / 2;
  cached->second = yaw_samples.size() % 2 == 0 ? 0.5 * (yaw_samples[middle - 1] + yaw_samples[middle]) : yaw_samples[middle];
  return cached->second;
}

double SmoothedRoute::curvature(double travel_distance) const {
  const double before_distance = std::max(-start_arc_length_, travel_distance - kCurvatureHalfBaselineM);
  const double after_distance = std::min(length_, travel_distance + kCurvatureHalfBaselineM);
  if (after_distance - before_distance < kKinematicEpsilon) return 0.0;
  return std::abs(std::remainder(yaw(after_distance) - yaw(before_distance), 2.0 * M_PI)) / (after_distance - before_distance);
}

lanelet::BasicPoint2d SmoothedRoute::point(double travel_distance) const {
  const double distance = std::clamp(travel_distance, 0.0, length_);
  const auto grid_index = static_cast<std::size_t>(distance / kRouteProfileResolutionM);
  const auto direction = [](double heading) { return lanelet::BasicPoint2d(std::cos(heading), std::sin(heading)); };
  while (grid_points_.size() <= grid_index) {
    const double segment_middle = (static_cast<double>(grid_points_.size()) - 0.5) * kRouteProfileResolutionM;
    grid_points_.push_back(grid_points_.back() + kRouteProfileResolutionM * direction(yaw(segment_middle)));
  }
  const double grid_distance = static_cast<double>(grid_index) * kRouteProfileResolutionM;
  const double remainder = distance - grid_distance;
  if (remainder < kKinematicEpsilon) return grid_points_[grid_index];
  return grid_points_[grid_index] + remainder * direction(yaw(grid_distance + 0.5 * remainder));
}

lanelet::BasicPoint2d SmoothedRoute::convergingPoint(double travel_distance,
                                                     double initial_lateral_offset,
                                                     double convergence_distance,
                                                     double initial_slope) const {
  const double lateral_offset =
      convergingLateralOffset(initial_lateral_offset, travel_distance, convergence_distance, initial_slope);
  const double heading = yaw(travel_distance);
  return point(travel_distance) + lateral_offset * lanelet::BasicPoint2d(-std::sin(heading), std::cos(heading));
}

Lanelet2ObjectListPrediction::Lanelet2ObjectListPrediction() : Node("lanelet2_object_list_prediction") {
  this->declareAndLoadParameter("input.ego_data_timeout", input_ego_data_timeout_, "timeout for considering ego vehicle data [s]",
                                true, false, false, 0.0, 60.0, 0.1);
  this->declareAndLoadParameter("processing.map_matching.ll2_map_server_name", processing_map_matching_ll2_map_server_name_,
                                "name of lanelet2_map_server node", false, false, true);
  this->declareAndLoadParameter("processing.map_matching.max_distance", processing_map_matching_max_distance_,
                                "max distance from a lanelet to consider it a match [m]", true, false, false, 0.0, 100.0, 0.1);
  this->declareAndLoadParameter("processing.map_matching.bicycle_max_distance", processing_map_matching_bicycle_max_distance_,
                                "max distance from a lanelet to consider it a match for bicycles, riding at lane edges [m]", true,
                                false, false, 0.0, 100.0, 0.1);
  this->declareAndLoadParameter("processing.map_matching.max_delta_yaw_deg", processing_map_matching_max_delta_yaw_deg_,
                                "max yaw difference from a lanelet direction to consider it a match [deg]", true, false, false,
                                0.0, 180.0);
  this->declareAndLoadParameter("processing.map_matching.fallback_mode", processing_map_matching_fallback_mode_,
                                "fallback mode for objects not matched to map [kinematic|static]", true, false, false,
                                std::nullopt, std::nullopt, std::nullopt, "Allowed values: static, kinematic");
  this->declareAndLoadParameter("processing.kinematic_limitations.enable", processing_kinematic_limitations_enable_,
                                "enable kinematic limitations");
  this->declareAndLoadParameter("processing.kinematic_limitations.max_lateral_acceleration",
                                processing_kinematic_limitations_max_lateral_acceleration_,
                                "max lateral acceleration for predictions [m/s^2]", true, false, false, 0.01, 20.0, 0.01);
  this->declareAndLoadParameter("processing.kinematic_limitations.max_longitudinal_deceleration",
                                processing_kinematic_limitations_max_longitudinal_deceleration_,
                                "max longitudinal deceleration for predictions [m/s^2]", true, false, false, 0.01, 20.0, 0.01);
  this->declareAndLoadParameter("processing.kinematic_limitations.max_longitudinal_acceleration",
                                processing_kinematic_limitations_max_longitudinal_acceleration_,
                                "max longitudinal acceleration for predictions [m/s^2]", true, false, false, 0.01, 20.0, 0.01);
  this->declareAndLoadParameter("processing.yielding.enable", processing_yielding_enable_, "enable yielding");
  this->declareAndLoadParameter("processing.yielding.clearance_distance", processing_yielding_clearance_distance_,
                                "clearance between front and yield line [m]", true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter("processing.yielding.clearance_time", processing_yielding_clearance_time_,
                                "time gap required before and after priority traffic [s]", true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter("processing.yielding.ego_lookahead_time", processing_yielding_ego_lookahead_time_,
                                "time up to which ego's right of way is considered, extrapolating its planned trajectory [s]",
                                true, false, false, 0.0, 30.0, 0.1);
  this->declareAndLoadParameter("processing.following.enable", processing_following_enable_,
                                "enable following, avoiding collisions with leading objects");
  this->declareAndLoadParameter("processing.following.headway_distance", processing_following_headway_distance_,
                                "min distance to the leading object [m]", true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter("processing.following.headway_time", processing_following_headway_time_,
                                "min time headway to the leading object [s]", true, false, false, 0.0, 5.0, 0.1);
  this->declareAndLoadParameter("processing.roundabout.enable", processing_roundabout_enable_,
                                "enable special roundabout handling");
  this->declareAndLoadParameter(
      "processing.roundabout.initial_alignment_tolerance", processing_roundabout_initial_alignment_tolerance_,
      "tolerance for initial alignment with roundabout centerline, not respecting kinematic limitations [m]", true, false, false,
      0.0, 10.0, 0.1);
  this->declareAndLoadParameter("output.prediction_horizon", output_prediction_horizon_, "prediction time horizon [s]", true,
                                false, false, 0.1, 60.0, 0.1);
  this->declareAndLoadParameter("output.sample_interval", output_sample_interval_, "time interval between prediction samples [s]",
                                true, false, false, 0.01, 10.0, 0.01);
  this->declareAndLoadParameter("output.infeasible_hypothesis_probability", output_infeasible_hypothesis_probability_,
                                "probability for infeasible hypotheses", true, false, false, 0.0, 1.0, 0.01);
  this->setup();
}

template <typename T>
void Lanelet2ObjectListPrediction::declareAndLoadParameter(const std::string& name,
                                                           T& param,
                                                           const std::string& description,
                                                           const bool add_to_auto_reconfigurable_params,
                                                           const bool is_required,
                                                           const bool read_only,
                                                           const std::optional<double>& from_value,
                                                           const std::optional<double>& to_value,
                                                           const std::optional<double>& step_value,
                                                           const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter type of parameter '%s' does not support "
                  "specifying a range",
                  name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult Lanelet2ObjectListPrediction::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(),
                    param.value_to_string().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void Lanelet2ObjectListPrediction::setup() {
  // TF listener for transforming incoming object lists into the map frame
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // map interface
  ll2_interface_ = std::make_unique<Lanelet2MapInterface>(*this, processing_map_matching_ll2_map_server_name_);

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&Lanelet2ObjectListPrediction::parametersCallback, this, std::placeholders::_1));

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      "~/tracked_object_list", 1, std::bind(&Lanelet2ObjectListPrediction::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  ego_data_subscriber_ = this->create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&Lanelet2ObjectListPrediction::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<perception_msgs::msg::ObjectList>("~/object_list", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
}

void Lanelet2ObjectListPrediction::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr& msg) {
  latest_ego_data_ = msg;
}

void Lanelet2ObjectListPrediction::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr& msg) {
  const auto processing_start = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "Message received with stamp: '%d'", msg->header.stamp.sec);

  if (!checkMap(true)) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map is not loaded yet, skipping object list");
    return;
  }

  perception_msgs::msg::ObjectList object_list_map_frame;
  if (msg->header.frame_id != ll2_interface_->map_frame_id_) {
    try {
      object_list_map_frame = tf_buffer_->transform(*msg, ll2_interface_->map_frame_id_, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not transform object list from frame '%s' to frame "
                   "'%s': %s. Skipping object list.",
                   msg->header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
      return;
    }
  } else {
    object_list_map_frame = *msg;
  }

  std::vector<PredictionObject> prediction_objects = matchObjectListToMap(object_list_map_frame);
  const auto matching_end = std::chrono::steady_clock::now();
  std::size_t matched_object_count = 0;
  for (PredictionObject& prediction_object : prediction_objects) {
    if (!prediction_object.lanelet_matches.empty()) {
      ++matched_object_count;
      createMapBasedPredictions(prediction_object, object_list_map_frame.header.stamp);
    }
  }
  const auto routes_end = std::chrono::steady_clock::now();

  std::optional<perception_msgs::msg::EgoData> ego_data_map_frame;
  if ((processing_yielding_enable_ || processing_following_enable_) && latest_ego_data_ == nullptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No ego data received on '%s', ignoring ego interaction",
                         ego_data_subscriber_->get_topic_name());
  } else if ((processing_yielding_enable_ || processing_following_enable_) && latest_ego_data_->trajectory_planned.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Ego data has no planned trajectory, ignoring ego interaction");
  } else if (processing_yielding_enable_ || processing_following_enable_) {
    const double ego_age =
        (rclcpp::Time(object_list_map_frame.header.stamp) - rclcpp::Time(latest_ego_data_->header.stamp)).seconds();
    if (!isEgoDataTimestampUsable(ego_age, input_ego_data_timeout_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Ego data is %.2f s older than the object list, ignoring ego interaction", ego_age);
    } else {
      try {
        if (latest_ego_data_->state.header.frame_id == ll2_interface_->map_frame_id_) {
          ego_data_map_frame = *latest_ego_data_;
        } else {
          ego_data_map_frame = tf_buffer_->transform(*latest_ego_data_, ll2_interface_->map_frame_id_, tf2::durationFromSec(0.1));
          for (std::size_t index = 0; index < ego_data_map_frame->trajectory_planned.size(); ++index) {
            ego_data_map_frame->trajectory_planned[index].header.stamp = latest_ego_data_->trajectory_planned[index].header.stamp;
          }
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform ego data to map frame, ignoring ego "
                    "interaction: %s",
                    ex.what());
      }
    }
  }

  const auto yield_start = std::chrono::steady_clock::now();
  if (processing_yielding_enable_) {
    applyYieldInteractions(prediction_objects, object_list_map_frame.header.stamp, ego_data_map_frame);
  }
  const auto following_start = std::chrono::steady_clock::now();
  if (processing_following_enable_) {
    applyFollowingInteractions(prediction_objects, object_list_map_frame.header.stamp, ego_data_map_frame);
  }
  const auto sampling_start = std::chrono::steady_clock::now();
  for (PredictionObject& prediction_object : prediction_objects) {
    prediction_object.object.state_predictions =
        createPredictionsForMatchedObject(prediction_object, object_list_map_frame.header.stamp);
  }
  const auto sampling_end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "Matched %zu/%zu objects to at least one lanelet", matched_object_count,
               object_list_map_frame.objects.size());

  perception_msgs::msg::ObjectList out_msg = object_list_map_frame;
  out_msg.objects.clear();
  out_msg.objects.reserve(prediction_objects.size());
  for (const PredictionObject& prediction_object : prediction_objects) {
    out_msg.objects.push_back(prediction_object.object);
  }

  publisher_->publish(out_msg);
  const auto milliseconds = [](auto from, auto to) { return std::chrono::duration<double, std::milli>(to - from).count(); };
  RCLCPP_DEBUG(this->get_logger(),
               "Message published with stamp: '%d' after %.1f ms (matching %.1f, routes %.1f, yield %.1f, following %.1f, "
               "sampling %.1f)",
               out_msg.header.stamp.sec, milliseconds(processing_start, std::chrono::steady_clock::now()),
               milliseconds(processing_start, matching_end), milliseconds(matching_end, routes_end),
               milliseconds(yield_start, following_start), milliseconds(following_start, sampling_start),
               milliseconds(sampling_start, sampling_end));
}

std::vector<Lanelet2ObjectListPrediction::PredictionObject> Lanelet2ObjectListPrediction::matchObjectListToMap(
    const perception_msgs::msg::ObjectList& object_list) const {
  std::vector<PredictionObject> prediction_objects;
  prediction_objects.reserve(object_list.objects.size());

  const auto map = ll2_interface_->getMapPtr();
  if (map == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Lanelet2 map pointer is null, cannot match objects to lanelets");
    return prediction_objects;
  }

  for (std::size_t object_index = 0; object_index < object_list.objects.size(); ++object_index) {
    PredictionObject prediction_object;
    prediction_object.object = object_list.objects[object_index];

    const auto classification = perception_msgs::object_access::getClassWithHighestProbability(prediction_object.object);
    const bool bicycle = isBicycleClass(classification.type);
    const bool pedestrian = isPedestrianClass(classification.type);
    const bool motorcycle = isMotorcycleClass(classification.type);

    geometry_msgs::msg::Point position;
    double object_yaw = 0.0;
    geometry_msgs::msg::Vector3 velocity;
    try {
      position = perception_msgs::object_access::getPosition(prediction_object.object);
      object_yaw = perception_msgs::object_access::getYaw(prediction_object.object);
      velocity = perception_msgs::object_access::getVelocityXYZ(prediction_object.object);
    } catch (const std::exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not read position, yaw, or velocity of object %zu: %s", object_index, ex.what());
      prediction_objects.push_back(prediction_object);
      continue;
    }

    const double planar_speed = std::hypot(velocity.x, velocity.y);
    const lanelet::BasicPoint2d position_2d(position.x, position.y);
    // Cyclists often ride at the edge of or between mapped lanes.
    const double max_distance =
        bicycle ? std::max(processing_map_matching_max_distance_, processing_map_matching_bicycle_max_distance_)
                : processing_map_matching_max_distance_;
    const auto candidate_lanelets = lanelet::geometry::findWithin2d(map->laneletLayer, position_2d, max_distance);

    const rclcpp::Logger matching_logger = this->get_logger().get_child("matching");
    for (const auto& candidate_lanelet : candidate_lanelets) {
      lanelet::ConstLanelet lanelet = candidate_lanelet.second;
      const auto reject = [&](const std::string& reason) {
        RCLCPP_DEBUG_STREAM(matching_logger, "object " << prediction_object.object.id << ": lanelet " << lanelet.id() << " ("
                                                       << candidate_lanelet.first << " m) rejected, " << reason);
      };
      lanelet::ConstLanelet matched_lanelet = lanelet;
      double start_arc_length = lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), position_2d).length;
      const double lanelet_length = static_cast<double>(lanelet::geometry::length(lanelet.centerline2d()));
      start_arc_length = std::clamp(start_arc_length, 0.0, lanelet_length);
      double orientation_difference = 0.0;

      const double lanelet_yaw = computeLaneletYawAtArcLength(lanelet, start_arc_length);
      const double inverted_arc_length = lanelet_length - start_arc_length;
      const double inverted_yaw = computeLaneletYawAtArcLength(lanelet.invert(), inverted_arc_length);
      const double lanelet_difference = std::abs(wrap_angle_rad(object_yaw - lanelet_yaw));
      const double inverted_difference = std::abs(wrap_angle_rad(object_yaw - inverted_yaw));

      if (inverted_difference < lanelet_difference) {
        matched_lanelet = lanelet.invert();
        start_arc_length = inverted_arc_length;
        orientation_difference = inverted_difference;
      } else {
        orientation_difference = lanelet_difference;
      }

      if (orientation_difference > processing_map_matching_max_delta_yaw_deg_ * M_PI / 180.0) {
        reject("yaw difference " + std::to_string(orientation_difference * 180.0 / M_PI) + " deg");
        continue;
      }

      // The German rules provide vehicle, bicycle and pedestrian participants.
      // Motorcycles may be observed on either road or bicycle lanes; use the
      // bicycle graph only for dedicated bicycle lanes, avoiding duplicate road matches.
      PredictionParticipant participant = PredictionParticipant::Vehicle;
      if (bicycle || (motorcycle && isBicycleLane(matched_lanelet))) {
        participant = PredictionParticipant::Bicycle;
      } else if (pedestrian) {
        participant = PredictionParticipant::Pedestrian;
      }
      const auto* matching_rules = traffic_rules_.get();
      if (participant == PredictionParticipant::Bicycle) {
        matching_rules = bicycle_traffic_rules_.get();
      } else if (participant == PredictionParticipant::Pedestrian) {
        matching_rules = pedestrian_traffic_rules_.get();
      }
      // Cyclists often ride against the direction of one-way bicycle lanes.
      const bool wrong_way = participant == PredictionParticipant::Bicycle && matching_rules != nullptr &&
                             matched_lanelet.inverted() && isBicycleLane(matched_lanelet) &&
                             !matching_rules->canPass(matched_lanelet) && matching_rules->canPass(matched_lanelet.invert());
      if (matching_rules == nullptr || (!matching_rules->canPass(matched_lanelet) && !wrong_way) ||
          (classification.type == perception_msgs::msg::ObjectClassification::VRU &&
           matched_lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{}) ==
               static_cast<const char*>(lanelet::AttributeValueString::Stairs))) {
        reject(std::string("traffic rules forbid ") + (matched_lanelet.inverted() ? "inverted " : "") +
               matched_lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{"lanelet"}) +
               (matching_rules != nullptr && matching_rules->canPass(matched_lanelet.invert()) ? "" : " (in both directions)"));
        continue;
      }

      const double matched_yaw = computeLaneletYawAtArcLength(matched_lanelet, start_arc_length);
      const LaneletVelocity lane_velocity = velocityAlongLanelet(velocity, matched_yaw);
      if (planar_speed >= kMotionDirectionMinSpeedMps &&
          std::abs(lane_velocity.longitudinal) < kMinAlongLaneSpeedFraction * planar_speed) {
        reject("moving sideways");
        continue;  // Nearly sideways motion is better represented by the Cartesian fallback.
      }
      // Below this speed the tracked velocity is noise: the object stands, keeping its position and heading
      const bool standing = planar_speed < kMotionDirectionMinSpeedMps;
      const bool reversing = !standing && lane_velocity.longitudinal < -kKinematicEpsilon;
      if (reversing && wrong_way) {
        reject("wrong-way heading but moving along the legal direction");
        continue;
      }
      if (reversing) {
        matched_lanelet = matched_lanelet.invert();
        start_arc_length = lanelet_length - start_arc_length;
      }
      double lateral_speed = reversing ? -lane_velocity.lateral : lane_velocity.lateral;
      if (standing) lateral_speed = 0.0;
      const double centerline_distance =
          std::abs(lanelet::geometry::toArcCoordinates(matched_lanelet.centerline2d(), position_2d).distance);
      prediction_object.lanelet_matches.push_back(LaneletMatch{
          matched_lanelet, participant, candidate_lanelet.first, centerline_distance, start_arc_length, orientation_difference,
          reversing, standing ? 0.0 : std::abs(lane_velocity.longitudinal), lateral_speed, wrong_way});
    }

    // Wrong-way riding is only assumed when no lanelet is ridden legally.
    auto& matches = prediction_object.lanelet_matches;
    if (std::any_of(matches.begin(), matches.end(), [](const LaneletMatch& match) { return !match.wrong_way; })) {
      matches.erase(std::remove_if(matches.begin(), matches.end(), [](const LaneletMatch& match) { return match.wrong_way; }),
                    matches.end());
    }

    // Bicycle and pedestrian rules also permit some shared or road lanelets.
    // Prefer dedicated space when it is a valid nearby match.
    if (bicycle || pedestrian) {
      const auto preferred = [&](const LaneletMatch& match) {
        return bicycle ? isBicycleLane(match.lanelet) : isPedestrianLane(match.lanelet);
      };
      if (std::any_of(matches.begin(), matches.end(), preferred)) {
        matches.erase(
            std::remove_if(matches.begin(), matches.end(), [&](const LaneletMatch& match) { return !preferred(match); }),
            matches.end());
      }
    }

    if (prediction_object.lanelet_matches.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Object %zu did not match any lanelet within %.2f m", object_index, max_distance);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Object %zu matched to %zu lanelet candidate(s)", object_index,
                   prediction_object.lanelet_matches.size());
    }
    prediction_objects.push_back(prediction_object);
  }

  return prediction_objects;
}

std::vector<perception_msgs::msg::ObjectStatePrediction> Lanelet2ObjectListPrediction::createPredictionsForMatchedObject(
    PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const {
  std::vector<perception_msgs::msg::ObjectStatePrediction> predictions;
  if (!prediction_object.hypotheses.empty()) {
    finalizeMapBasedPredictions(prediction_object, base_time);
    predictions.reserve(prediction_object.hypotheses.size());
    for (const PredictionObject::Hypothesis& hypothesis : prediction_object.hypotheses) {
      predictions.push_back(hypothesis.prediction);
    }
  }

  if (predictions.empty()) {
    RCLCPP_DEBUG(this->get_logger().get_child("hypotheses"),
                 "object %lu: no map-based hypothesis (%zu lanelet matches), %s fallback",
                 static_cast<unsigned long>(prediction_object.object.id), prediction_object.lanelet_matches.size(),
                 processing_map_matching_fallback_mode_.c_str());
    if (processing_map_matching_fallback_mode_ == "static") {
      predictions.push_back(createStationaryPrediction(prediction_object.object, base_time));
    } else {
      predictions.push_back(createConstantVelocityPrediction(prediction_object.object, base_time));
    }
  }

  const double probability_sum = std::accumulate(
      predictions.begin(), predictions.end(), 0.0,
      [](double sum, const perception_msgs::msg::ObjectStatePrediction& prediction) { return sum + prediction.probability; });
  if (probability_sum <= kKinematicEpsilon) {
    const double probability = 1.0 / static_cast<double>(predictions.size());
    for (perception_msgs::msg::ObjectStatePrediction& prediction : predictions) {
      prediction.probability = probability;
    }
  }
  return predictions;
}

void Lanelet2ObjectListPrediction::createMapBasedPredictions(PredictionObject& prediction_object,
                                                             const builtin_interfaces::msg::Time& base_time) const {
  prediction_object.hypotheses.clear();
  const auto classification = perception_msgs::object_access::getClassWithHighestProbability(prediction_object.object);
  for (const LaneletMatch& match : prediction_object.lanelet_matches) {
    const auto* routing_graph = routing_graph_.get();
    if (match.participant == PredictionParticipant::Bicycle) {
      routing_graph = bicycle_routing_graph_.get();
    } else if (match.participant == PredictionParticipant::Pedestrian) {
      routing_graph = pedestrian_routing_graph_.get();
    }
    if (routing_graph == nullptr) continue;
    const double speed = match.longitudinal_speed;
    const double max_travel_distance = speed * output_prediction_horizon_;
    lanelet::routing::LaneletPaths routes;
    if (match.reversing || max_travel_distance <= std::numeric_limits<double>::epsilon()) {
      routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
    } else if (match.wrong_way) {
      const double remaining_on_start =
          static_cast<double>(lanelet::geometry::length(match.lanelet.centerline2d())) - match.start_arc_length;
      routes = wrongWayBicyclePaths(match.lanelet, max_travel_distance - remaining_on_start, *routing_graph);
    } else {
      lanelet::routing::PossiblePathsParams params;
      params.routingCostLimit = max_travel_distance + match.start_arc_length;
      params.includeShorterPaths = true;
      params.includeLaneChanges = false;
      try {
        routes = routing_graph->possiblePaths(match.lanelet, params);
      } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not create lanelet routes from matched lanelet: %s", ex.what());
        continue;
      }
      if (routes.empty()) {
        routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
      }
    }

    for (const lanelet::routing::LaneletPath& lanelet_route : routes) {
      const double route_length = remainingRouteLength(lanelet_route, match.start_arc_length);
      const bool stop_at_route_end = route_length + kKinematicEpsilon < max_travel_distance;
      const auto [geometry_route, geometry_start_arc_length] =
          routeGeometryWithPredecessor(lanelet_route, match.start_arc_length, *routing_graph);
      const RouteMotionProfile motion_profile = buildRouteMotionProfile(
          geometry_route, geometry_start_arc_length, speed, processing_kinematic_limitations_max_lateral_acceleration_,
          processing_kinematic_limitations_max_longitudinal_acceleration_,
          processing_kinematic_limitations_max_longitudinal_deceleration_, stop_at_route_end, std::nullopt,
          processing_kinematic_limitations_enable_);
      bool feasible = motion_profile.feasible;
      const char* infeasibility = feasible ? "" : "curve speed not reachable";
      if (!feasible) {
        RCLCPP_DEBUG_STREAM(this->get_logger().get_child("hypotheses"),
                            "object " << prediction_object.object.id << " critical curve: "
                                      << criticalCurveDescription(geometry_route, geometry_start_arc_length, speed,
                                                                  processing_kinematic_limitations_max_lateral_acceleration_));
      }
      if (processing_kinematic_limitations_enable_ && feasible) {
        const SmoothedRoute smoothed_route(geometry_route, geometry_start_arc_length);
        const double route_lateral_acceleration = speed * speed * smoothed_route.curvature(0.0);
        const RouteMotionSample first_motion = sampleRouteMotionAtTime(motion_profile.samples, output_sample_interval_);
        const perception_msgs::msg::ObjectState first_state =
            sampleStateOnLaneletRoute(prediction_object.object.state, smoothed_route, first_motion.distance, first_motion.speed,
                                      speed, match.lateral_speed, match.reversing, base_time, 0);
        feasible =
            initialMotionFeasible(perception_msgs::object_access::getPosition(prediction_object.object),
                                  perception_msgs::object_access::getVelocityXYZ(prediction_object.object),
                                  perception_msgs::object_access::getPosition(first_state), output_sample_interval_,
                                  processing_kinematic_limitations_max_lateral_acceleration_ + route_lateral_acceleration,
                                  processing_kinematic_limitations_max_longitudinal_acceleration_,
                                  processing_kinematic_limitations_max_longitudinal_deceleration_,
                                  processing_roundabout_enable_ && startsNearRoundabout(lanelet_route, match.start_arc_length)
                                      ? processing_roundabout_initial_alignment_tolerance_
                                      : 0.0);
        if (!feasible) {
          infeasibility = "initial motion not reachable";
          RCLCPP_DEBUG_STREAM(this->get_logger().get_child("hypotheses"), [&] {
            // Acceleration needed to reach the first predicted position, split along and across the motion.
            const auto observed = perception_msgs::object_access::getPosition(prediction_object.object);
            const auto velocity = perception_msgs::object_access::getVelocityXYZ(prediction_object.object);
            const auto predicted = perception_msgs::object_access::getPosition(first_state);
            const double scale = 2.0 / (output_sample_interval_ * output_sample_interval_);
            const double ax = (predicted.x - observed.x - velocity.x * output_sample_interval_) * scale;
            const double ay = (predicted.y - observed.y - velocity.y * output_sample_interval_) * scale;
            const double v = std::max(std::hypot(velocity.x, velocity.y), kKinematicEpsilon);
            std::ostringstream detail;
            detail << "object " << prediction_object.object.id << " first step needs " << (ax * velocity.x + ay * velocity.y) / v
                   << " m/s^2 along, " << (ay * velocity.x - ax * velocity.y) / v << " m/s^2 across (route curve allows "
                   << route_lateral_acceleration << " m/s^2 extra)";
            return detail.str();
          }());
        }
      }
      std::ostringstream route_ids;
      for (const lanelet::ConstLanelet& route_lanelet : lanelet_route) route_ids << " " << route_lanelet.id();
      RCLCPP_DEBUG_STREAM(this->get_logger().get_child("hypotheses"),
                          "object " << prediction_object.object.id << (match.wrong_way ? " wrong-way" : "") << " route"
                                    << route_ids.str() << " (v " << speed << " m/s, lateral v " << match.lateral_speed
                                    << " m/s): " << (feasible ? "feasible" : infeasibility));
      if (!feasible && output_infeasible_hypothesis_probability_ == 0.0) continue;
      PredictionObject::Hypothesis hypothesis;
      hypothesis.route = lanelet_route;
      hypothesis.participant = match.participant;
      if (isMotorcycleClass(classification.type)) {
        hypothesis.match_weight = 1.0 / ((1.0 + match.centerline_distance) * static_cast<double>(routes.size()));
      }
      hypothesis.start_arc_length = match.start_arc_length;
      hypothesis.geometry_route = geometry_route;
      hypothesis.geometry_start_arc_length = geometry_start_arc_length;
      hypothesis.initial_speed = speed;
      hypothesis.initial_lateral_speed = match.lateral_speed;
      hypothesis.reversing = match.reversing;
      hypothesis.wrong_way = match.wrong_way;
      hypothesis.stop_at_route_end = stop_at_route_end;
      hypothesis.feasible = feasible;
      hypothesis.motion_profile = motion_profile.samples;
      prediction_object.hypotheses.push_back(std::move(hypothesis));
    }
  }
}

void Lanelet2ObjectListPrediction::finalizeMapBasedPredictions(PredictionObject& prediction_object,
                                                               const builtin_interfaces::msg::Time& base_time) const {
  if (output_infeasible_hypothesis_probability_ == 0.0) {
    prediction_object.hypotheses.erase(
        std::remove_if(prediction_object.hypotheses.begin(), prediction_object.hypotheses.end(),
                       [](const PredictionObject::Hypothesis& hypothesis) { return !hypothesis.feasible; }),
        prediction_object.hypotheses.end());
  }

  const std::size_t sample_count = getPredictionSampleCount();
  for (PredictionObject::Hypothesis& hypothesis : prediction_object.hypotheses) {
    hypothesis.prediction.states.clear();
    hypothesis.prediction.states.reserve(sample_count);
    const SmoothedRoute route(hypothesis.geometry_route, hypothesis.geometry_start_arc_length);
    for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
      const double sample_time = output_sample_interval_ * static_cast<double>(sample_index + 1);
      const RouteMotionSample motion = sampleRouteMotionAtTime(hypothesis.motion_profile, sample_time);
      hypothesis.prediction.states.push_back(sampleStateOnLaneletRoute(
          prediction_object.object.state, route, motion.distance, motion.speed, hypothesis.initial_speed,
          hypothesis.initial_lateral_speed, hypothesis.reversing, base_time, sample_index));
    }
  }

  const std::size_t feasible_count =
      static_cast<std::size_t>(std::count_if(prediction_object.hypotheses.begin(), prediction_object.hypotheses.end(),
                                             [](const PredictionObject::Hypothesis& hypothesis) { return hypothesis.feasible; }));
  const std::size_t infeasible_count = prediction_object.hypotheses.size() - feasible_count;
  if (feasible_count > 0) {
    const double infeasible_probability = infeasible_count > 0 ? std::min(output_infeasible_hypothesis_probability_,
                                                                          1.0 / static_cast<double>(infeasible_count + 1))
                                                               : 0.0;
    const double feasible_weight = std::accumulate(prediction_object.hypotheses.begin(), prediction_object.hypotheses.end(), 0.0,
                                                   [](double weight, const PredictionObject::Hypothesis& hypothesis) {
                                                     return weight + (hypothesis.feasible ? hypothesis.match_weight : 0.0);
                                                   });
    const double feasible_probability_mass = 1.0 - infeasible_probability * static_cast<double>(infeasible_count);
    for (PredictionObject::Hypothesis& hypothesis : prediction_object.hypotheses) {
      hypothesis.prediction.probability =
          hypothesis.feasible ? feasible_probability_mass * hypothesis.match_weight / feasible_weight : infeasible_probability;
    }
  } else if (!prediction_object.hypotheses.empty()) {
    const double probability = 1.0 / static_cast<double>(prediction_object.hypotheses.size());
    for (PredictionObject::Hypothesis& hypothesis : prediction_object.hypotheses) {
      hypothesis.prediction.probability = probability;
    }
  }
}

void Lanelet2ObjectListPrediction::applyYieldInteractions(std::vector<PredictionObject>& prediction_objects,
                                                          const builtin_interfaces::msg::Time& base_time,
                                                          const std::optional<perception_msgs::msg::EgoData>& ego_data) const {
  const lanelet::LaneletMapConstPtr map = ll2_interface_->getMapPtr();
  if (map == nullptr) return;

  const rclcpp::Time base_stamp(base_time);
  const rclcpp::Logger yield_logger = this->get_logger().get_child("yield");

  auto routeOccupancy = [](const PredictionObject::Hypothesis& hypothesis, const std::unordered_set<lanelet::Id>& lanelet_ids,
                           const lanelet::ConstLanelet& conflict_lanelet) -> std::optional<TimeInterval> {
    double entry_distance = std::numeric_limits<double>::infinity();
    double exit_distance = -std::numeric_limits<double>::infinity();
    for (std::size_t route_index = 0; route_index < hypothesis.route.size(); ++route_index) {
      const lanelet::ConstLanelet& route_lanelet = hypothesis.route[route_index];
      const double lanelet_start = routeDistanceAtLaneletStart(hypothesis.route, hypothesis.start_arc_length, route_index);
      const double lanelet_length = static_cast<double>(lanelet::geometry::length(route_lanelet.centerline2d()));
      if (lanelet_ids.count(route_lanelet.id()) > 0) {
        entry_distance = std::min(entry_distance, std::max(0.0, lanelet_start));
        exit_distance = std::max(exit_distance, lanelet_start + lanelet_length);
      }
    }
    if (!std::isfinite(entry_distance) || exit_distance < 0.0) return std::nullopt;
    // A priority lanelet can be only the approach to the crossing. Keep the
    // participant present until its nominal route leaves the conflict lanelet.
    for (std::size_t route_index = 0; route_index < hypothesis.route.size(); ++route_index) {
      const lanelet::ConstLanelet& route_lanelet = hypothesis.route[route_index];
      const double lanelet_start = routeDistanceAtLaneletStart(hypothesis.route, hypothesis.start_arc_length, route_index);
      if (lanelet_start + kKinematicEpsilon < entry_distance) continue;
      // Polygon overlap is symmetric; the two route roles have no argument order.
      // NOLINTNEXTLINE(readability-suspicious-call-argument)
      if (lanelet::geometry::overlaps2d(route_lanelet, conflict_lanelet)) {
        exit_distance =
            std::max(exit_distance, lanelet_start + static_cast<double>(lanelet::geometry::length(route_lanelet.centerline2d())));
      }
    }
    const TimeInterval interval{timeAtRouteDistance(hypothesis.motion_profile, entry_distance),
                                timeAtRouteDistance(hypothesis.motion_profile, exit_distance)};
    if (!std::isfinite(interval.entry)) return std::nullopt;

    return interval;
  };

  std::vector<std::vector<std::optional<YieldConstraint>>> yield_constraints;
  yield_constraints.reserve(prediction_objects.size());
  for (const PredictionObject& prediction_object : prediction_objects) {
    yield_constraints.emplace_back(prediction_object.hypotheses.size());
  }

  // Determine all constraints from nominal profiles before changing any of
  // them, so interactions are independent of object processing order.
  for (std::size_t object_index = 0; object_index < prediction_objects.size(); ++object_index) {
    const PredictionObject& prediction_object = prediction_objects[object_index];
    double front_offset = processing_yielding_clearance_distance_;
    try {
      const double length = perception_msgs::object_access::getLength(prediction_object.object);
      const double reference_to_center = prediction_object.object.state.reference_point.translation_to_geometric_center.x;
      front_offset += std::max(0.0, reference_to_center + 0.5 * length);
    } catch (const std::exception& ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not determine object length for yield-line clearance: %s", ex.what());
    }

    for (std::size_t hypothesis_index = 0; hypothesis_index < prediction_object.hypotheses.size(); ++hypothesis_index) {
      const PredictionObject::Hypothesis& hypothesis = prediction_object.hypotheses[hypothesis_index];
      const auto* routing_graph = routing_graph_.get();
      if (hypothesis.participant == PredictionParticipant::Bicycle) {
        routing_graph = bicycle_routing_graph_.get();
      } else if (hypothesis.participant == PredictionParticipant::Pedestrian) {
        routing_graph = pedestrian_routing_graph_.get();
      }
      // Right-of-way rules are mapped for the legal direction of travel only.
      if (hypothesis.reversing || hypothesis.wrong_way) continue;
      std::optional<YieldConstraint> selected_constraint;
      std::ostringstream route_ids;
      for (const lanelet::ConstLanelet& route_lanelet : hypothesis.route) route_ids << " " << route_lanelet.id();
      const std::string log_prefix = "object " + std::to_string(prediction_object.object.id) + " hypothesis " +
                                     std::to_string(hypothesis_index) + " (route" + route_ids.str() + ", v " +
                                     std::to_string(hypothesis.initial_speed) + " m/s)";
      std::size_t evaluated_rules = 0;
      auto evaluateRule = [&](const lanelet::ConstLanelet& regulating_lanelet, std::size_t route_index,
                              bool regulating_lanelet_on_route) {
        for (const auto& right_of_way : regulating_lanelet.regulatoryElementsAs<lanelet::RightOfWay>()) {
          ++evaluated_rules;
          const std::string rule_prefix = log_prefix + " rule " + std::to_string(right_of_way->id()) + " on " +
                                          std::to_string(regulating_lanelet.id()) + ": ";
          if (right_of_way->getManeuver(regulating_lanelet) != lanelet::ManeuverType::Yield) {
            RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "lanelet has right of way");
            continue;
          }

          std::unordered_set<lanelet::Id> priority_lanelet_ids;
          for (const lanelet::ConstLanelet& priority_lanelet : right_of_way->rightOfWayLanelets()) {
            priority_lanelet_ids.insert(priority_lanelet.id());
          }
          if (priority_lanelet_ids.empty() || routing_graph == nullptr) {
            RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "no priority lanelets");
            continue;
          }

          double yield_line_distance = 0.0;
          std::size_t interval_end_index = route_index;
          const auto stop_line = right_of_way->stopLine();
          if (stop_line.has_value() && !stop_line->empty()) {
            const auto projection =
                projectYieldLineOnRoute(hypothesis.route, hypothesis.start_arc_length, route_index, *stop_line);
            if (!projection.has_value()) {
              RCLCPP_DEBUG_STREAM(
                  yield_logger, rule_prefix << "stop line " << stop_line->id() << " not on route (>5 m away, or already passed)");
              continue;
            }
            yield_line_distance = projection->distance;
            interval_end_index = projection->lanelet_index;
          } else {
            // Without a mapped line, the end of the regulating lanelet is the
            // stop position. A predecessor already lies behind this route.
            if (!regulating_lanelet_on_route) {
              RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "no stop line and regulating lanelet already passed");
              continue;
            }
            const double lanelet_start = routeDistanceAtLaneletStart(hypothesis.route, hypothesis.start_arc_length, route_index);
            const double lanelet_length = static_cast<double>(lanelet::geometry::length(regulating_lanelet.centerline2d()));
            yield_line_distance = lanelet_start + lanelet_length;
          }

          if (yield_line_distance < -kKinematicEpsilon) {
            RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "yield line passed (" << yield_line_distance << " m)");
            continue;
          }
          const std::optional<std::size_t> conflict_index =
              conflictLaneletAtYieldLine(hypothesis.route, route_index, interval_end_index, *right_of_way, *routing_graph);
          if (!conflict_index) {
            RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "route does not conflict with priority lanelets");
            continue;
          }
          interval_end_index = std::max(interval_end_index, *conflict_index);

          const double stop_distance = std::max(0.0, yield_line_distance - front_offset);
          const double interval_end =
              routeDistanceAtLaneletStart(hypothesis.route, hypothesis.start_arc_length, interval_end_index) +
              static_cast<double>(lanelet::geometry::length(hypothesis.route[interval_end_index].centerline2d()));
          const TimeInterval yielding_interval{timeAtRouteDistance(hypothesis.motion_profile, stop_distance),
                                               timeAtRouteDistance(hypothesis.motion_profile, interval_end)};
          if (!std::isfinite(yielding_interval.entry)) {
            RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "never reaches the stop position " << stop_distance << " m");
            continue;
          }

          double latest_clearance = -std::numeric_limits<double>::infinity();
          std::vector<lanelet::ConstLanelet> priority_lanelets;
          priority_lanelets.reserve(priority_lanelet_ids.size());
          for (const lanelet::ConstLanelet& priority_lanelet : right_of_way->rightOfWayLanelets()) {
            priority_lanelets.push_back(priority_lanelet);
          }
          const lanelet::ConstLanelet& conflict_lanelet = hypothesis.route[*conflict_index];
          const std::optional<TimeInterval> ego_interval =
              ego_data ? egoRouteOccupancy(*ego_data, base_stamp, processing_yielding_ego_lookahead_time_, priority_lanelets,
                                           conflict_lanelet)
                       : std::nullopt;
          std::ostringstream ego_log;
          if (!ego_data) {
            ego_log << "no ego data";
          } else if (!ego_interval) {
            ego_log << "ego not on priority lanelets within horizon";
          } else {
            ego_log << "ego on priority lanelets [" << ego_interval->entry << ", " << ego_interval->exit << "] s";
          }
          if (ego_interval.has_value() &&
              conflictsDuringClearance(yielding_interval, *ego_interval, processing_yielding_clearance_time_)) {
            latest_clearance =
                std::max(latest_clearance, std::isfinite(ego_interval->exit) ? ego_interval->exit : output_prediction_horizon_);
            ego_log << " -> conflict";
          }

          std::size_t conflicting_priority_objects = 0;
          for (std::size_t priority_object_index = 0; priority_object_index < prediction_objects.size();
               ++priority_object_index) {
            if (priority_object_index == object_index) continue;
            for (const PredictionObject::Hypothesis& priority_hypothesis : prediction_objects[priority_object_index].hypotheses) {
              const std::optional<TimeInterval> priority_interval =
                  routeOccupancy(priority_hypothesis, priority_lanelet_ids, conflict_lanelet);
              if (priority_interval.has_value() &&
                  conflictsDuringClearance(yielding_interval, *priority_interval, processing_yielding_clearance_time_)) {
                latest_clearance =
                    std::max(latest_clearance,
                             std::isfinite(priority_interval->exit) ? priority_interval->exit : output_prediction_horizon_);
                ++conflicting_priority_objects;
              }
            }
          }
          RCLCPP_DEBUG_STREAM(yield_logger, rule_prefix << "stop at " << stop_distance << " m, occupies conflict ["
                                                        << yielding_interval.entry << ", " << yielding_interval.exit << "] s, "
                                                        << ego_log.str() << ", " << conflicting_priority_objects
                                                        << " conflicting priority hypotheses -> "
                                                        << (std::isfinite(latest_clearance) ? "yield" : "no yield"));

          if (std::isfinite(latest_clearance)) {
            const YieldConstraint constraint{stop_distance, latest_clearance + processing_yielding_clearance_time_};
            if (!selected_constraint.has_value() ||
                constraint.stop_distance < selected_constraint->stop_distance - kKinematicEpsilon) {
              selected_constraint = constraint;
            } else if (std::abs(constraint.stop_distance - selected_constraint->stop_distance) < kKinematicEpsilon) {
              selected_constraint->release_time = std::max(selected_constraint->release_time, constraint.release_time);
            }
          }
        }
      };

      for (std::size_t route_index = 0; route_index < hypothesis.route.size(); ++route_index) {
        evaluateRule(hypothesis.route[route_index], route_index, true);
      }
      // The vehicle may already be on the first successor of a regulated
      // lanelet while the reference line is still ahead of it.
      if (routing_graph != nullptr && !hypothesis.route.empty()) {
        for (const lanelet::ConstLanelet& predecessor : routing_graph->previous(hypothesis.route.front())) {
          evaluateRule(predecessor, 0, false);
        }
      }
      if (evaluated_rules == 0) {
        RCLCPP_DEBUG_STREAM(yield_logger, log_prefix << ": no right-of-way rule on route or predecessors");
      }

      yield_constraints[object_index][hypothesis_index] = selected_constraint;
    }
  }

  for (std::size_t object_index = 0; object_index < prediction_objects.size(); ++object_index) {
    PredictionObject& prediction_object = prediction_objects[object_index];
    for (std::size_t hypothesis_index = 0; hypothesis_index < prediction_object.hypotheses.size(); ++hypothesis_index) {
      const std::optional<YieldConstraint>& selected_constraint = yield_constraints[object_index][hypothesis_index];
      if (selected_constraint.has_value()) {
        PredictionObject::Hypothesis& hypothesis = prediction_object.hypotheses[hypothesis_index];
        const RouteMotionProfile yielded_profile =
            buildRouteMotionProfile(hypothesis.geometry_route, hypothesis.geometry_start_arc_length, hypothesis.initial_speed,
                                    processing_kinematic_limitations_max_lateral_acceleration_,
                                    processing_kinematic_limitations_max_longitudinal_acceleration_,
                                    processing_kinematic_limitations_max_longitudinal_deceleration_, hypothesis.stop_at_route_end,
                                    selected_constraint, processing_kinematic_limitations_enable_);
        hypothesis.motion_profile = yielded_profile.samples;
        hypothesis.feasible = hypothesis.feasible && yielded_profile.feasible;
        RCLCPP_DEBUG(yield_logger, "object %lu hypothesis %zu: yielding at %.2f m until %.2f s, %s",
                     static_cast<unsigned long>(prediction_object.object.id), hypothesis_index,
                     selected_constraint->stop_distance, selected_constraint->release_time,
                     yielded_profile.feasible ? "feasible" : "INFEASIBLE (hypothesis dropped)");
      }
    }
  }
}

void Lanelet2ObjectListPrediction::applyFollowingInteractions(
    std::vector<PredictionObject>& prediction_objects,
    const builtin_interfaces::msg::Time& base_time,
    const std::optional<perception_msgs::msg::EgoData>& ego_data) const {
  const rclcpp::Time base_stamp(base_time);
  std::vector<TimedRoutePose> ego_poses;
  if (ego_data) {
    auto append = [&](const perception_msgs::msg::ObjectState& state) {
      try {
        const auto position = perception_msgs::object_access::getPosition(state);
        const double time = (rclcpp::Time(state.header.stamp) - base_stamp).seconds();
        if (std::isfinite(time) && std::isfinite(position.x) && std::isfinite(position.y)) {
          ego_poses.push_back(
              {time, lanelet::BasicPoint2d(position.x, position.y), perception_msgs::object_access::getYaw(state)});
        }
      } catch (const std::exception&) {
      }
    };
    append(ego_data->state);
    for (const auto& state : ego_data->trajectory_planned) append(state);
    std::sort(ego_poses.begin(), ego_poses.end(),
              [](const TimedRoutePose& lhs, const TimedRoutePose& rhs) { return lhs.time < rhs.time; });
  }

  auto offsets = [](const perception_msgs::msg::Object& object) {
    try {
      const double center = object.state.reference_point.translation_to_geometric_center.x;
      const double half_length = 0.5 * perception_msgs::object_access::getLength(object);
      return std::pair<double, double>{center + half_length, center - half_length};
    } catch (const std::exception&) {
      return std::pair<double, double>{0.0, 0.0};
    }
  };
  std::vector<std::pair<double, double>> object_offsets;
  object_offsets.reserve(prediction_objects.size());
  for (const PredictionObject& object : prediction_objects) object_offsets.push_back(offsets(object.object));
  const double ego_rear =
      ego_data ? ego_data->state.reference_point.translation_to_geometric_center.x - 0.5 * ego_data->length : 0.0;
  const double step_size = std::min(0.1, output_sample_interval_);
  std::vector<std::vector<std::optional<RouteMotionProfile>>> results(prediction_objects.size());
  for (std::size_t object_index = 0; object_index < prediction_objects.size(); ++object_index) {
    const PredictionObject& object = prediction_objects[object_index];
    results[object_index].resize(object.hypotheses.size());
    const double follower_front = object_offsets[object_index].first;
    for (std::size_t hypothesis_index = 0; hypothesis_index < object.hypotheses.size(); ++hypothesis_index) {
      const PredictionObject::Hypothesis& follower = object.hypotheses[hypothesis_index];
      if (follower.reversing || follower.motion_profile.empty() || follower.route.empty()) continue;
      // Traffic behind the follower cannot lead it
      std::optional<lanelet::BasicPoint2d> follower_position;
      lanelet::BasicPoint2d follower_direction(0.0, 0.0);
      try {
        const auto position = perception_msgs::object_access::getPosition(object.object);
        const double yaw = perception_msgs::object_access::getYaw(object.object);
        follower_position = lanelet::BasicPoint2d(position.x, position.y);
        follower_direction = lanelet::BasicPoint2d(std::cos(yaw), std::sin(yaw));
      } catch (const std::exception&) {
      }
      const auto behind_follower = [&](const lanelet::BasicPoint2d& position) {
        return follower_position.has_value() && (position - *follower_position).dot(follower_direction) < 0.0;
      };
      const bool ego_behind = !ego_poses.empty() && behind_follower(ego_poses.front().position);
      std::vector<bool> leader_behind(prediction_objects.size(), false);
      for (std::size_t leader_index = 0; leader_index < prediction_objects.size(); ++leader_index) {
        try {
          const auto position = perception_msgs::object_access::getPosition(prediction_objects[leader_index].object);
          leader_behind[leader_index] = behind_follower(lanelet::BasicPoint2d(position.x, position.y));
        } catch (const std::exception&) {
        }
      }
      RouteMotionProfile profile;
      profile.feasible = follower.feasible;
      profile.samples.push_back(follower.motion_profile.front());
      bool following_active = false;
      std::optional<RouteMotionProfile> unconstrained_profile;
      const double route_length = remainingRouteLength(follower.route, follower.start_arc_length);

      for (double time = 0.0; time < output_prediction_horizon_ - kKinematicEpsilon;) {
        const double next_time = std::min(output_prediction_horizon_, time + step_size);
        const double dt = next_time - time;
        const RouteMotionSample& previous = profile.samples.back();
        RouteMotionSample nominal = sampleRouteMotionAtTime(follower.motion_profile, next_time);
        nominal.time = next_time;
        double speed_limit = nominal.speed;
        double distance_limit = std::numeric_limits<double>::infinity();
        double braking_speed_limit = std::numeric_limits<double>::infinity();
        std::string limiting_leader;

        auto consider = [&](const lanelet::BasicPoint2d& position_now, const lanelet::BasicPoint2d& position_next,
                            double yaw_next, double leader_speed, double leader_rear,
                            const std::optional<lanelet::Id>& lanelet_id, const std::string& leader_label) {
          const auto projected_next =
              projectLeaderOnRoute(follower.route, follower.start_arc_length, position_next, yaw_next, lanelet_id);
          if (!projected_next || *projected_next <= previous.distance + kKinematicEpsilon) return;
          const auto projected_now =
              projectLeaderOnRoute(follower.route, follower.start_arc_length, position_now, yaw_next, lanelet_id);
          const double gap =
              (projected_now ? *projected_now : *projected_next) + leader_rear - previous.distance - follower_front;
          const double clearance = *projected_next + leader_rear - follower_front - processing_following_headway_distance_;
          if (clearance < distance_limit) limiting_leader = leader_label;
          distance_limit = std::min(distance_limit, clearance);
          const double available_braking_distance =
              std::max(0.0, gap - processing_following_headway_distance_ - processing_following_headway_time_ * previous.speed);
          braking_speed_limit = std::min(
              braking_speed_limit,
              std::sqrt(leader_speed * leader_speed +
                        2.0 * processing_kinematic_limitations_max_longitudinal_deceleration_ * available_braking_distance));
        };

        if (!ego_poses.empty() && !ego_behind) {
          const auto now = interpolatePose(ego_poses, time);
          const auto next = interpolatePose(ego_poses, next_time);
          if (now && next) {
            const lanelet::BasicPoint2d motion = next->position - now->position;
            const double speed = motion.norm() / dt;
            const double yaw = speed > kMotionDirectionMinSpeedMps ? std::atan2(motion.y(), motion.x()) : next->yaw;
            consider(now->position, next->position, yaw, speed, ego_rear, std::nullopt, "ego");
          }
        }
        for (std::size_t leader_index = 0; leader_index < prediction_objects.size(); ++leader_index) {
          if (leader_index == object_index || leader_behind[leader_index]) continue;
          const PredictionObject& leader_object = prediction_objects[leader_index];
          const double leader_rear = object_offsets[leader_index].second;
          if (leader_object.hypotheses.empty()) {
            try {
              const auto position = perception_msgs::object_access::getPosition(leader_object.object);
              const auto velocity = perception_msgs::object_access::getVelocityXYZ(leader_object.object);
              const double speed = processing_map_matching_fallback_mode_ == "static" ? 0.0 : std::hypot(velocity.x, velocity.y);
              const double yaw = speed > kMotionDirectionMinSpeedMps
                                     ? std::atan2(velocity.y, velocity.x)
                                     : perception_msgs::object_access::getYaw(leader_object.object);
              const lanelet::BasicPoint2d start(position.x, position.y);
              const lanelet::BasicPoint2d step =
                  speed > 0.0 ? lanelet::BasicPoint2d(velocity.x, velocity.y) : lanelet::BasicPoint2d(0.0, 0.0);
              consider(start + time * step, start + next_time * step, yaw, speed, leader_rear, std::nullopt,
                       "object " + std::to_string(leader_object.object.id) + " (fallback)");
            } catch (const std::exception&) {
            }
            continue;
          }
          for (const PredictionObject::Hypothesis& leader : leader_object.hypotheses) {
            if (leader.reversing || leader.route.empty() || leader.motion_profile.empty() || !leader.feasible) continue;
            const RouteMotionSample now = sampleRouteMotionAtTime(leader.motion_profile, time);
            const RouteMotionSample next = sampleRouteMotionAtTime(leader.motion_profile, next_time);
            const auto [lanelet_index, arc_length] = laneletAtRouteDistance(leader.route, leader.start_arc_length, next.distance);
            const lanelet::ConstLanelet& lanelet = leader.route[lanelet_index];
            const double yaw = computeLaneletYawAtArcLength(lanelet, arc_length);
            consider(pointOnRoute(leader.route, leader.start_arc_length, now.distance),
                     lanelet::geometry::interpolatedPointAtDistance(lanelet.centerline2d(), arc_length), yaw, next.speed,
                     leader_rear, lanelet.id(), "object " + std::to_string(leader_object.object.id));
          }
        }

        if (std::isfinite(distance_limit)) {
          const double positional_speed_limit =
              (distance_limit - previous.distance - 0.5 * previous.speed * dt) / (processing_following_headway_time_ + 0.5 * dt);
          speed_limit = std::min({speed_limit, std::max(0.0, positional_speed_limit), braking_speed_limit});
        }
        if (!following_active && speed_limit >= nominal.speed - kKinematicEpsilon &&
            distance_limit >= nominal.distance + processing_following_headway_time_ * nominal.speed - kKinematicEpsilon) {
          profile.samples.push_back(nominal);
          time = next_time;
          continue;
        }
        following_active = true;
        if (!unconstrained_profile) {
          unconstrained_profile =
              buildRouteMotionProfile(follower.geometry_route, follower.geometry_start_arc_length, follower.initial_speed,
                                      processing_kinematic_limitations_max_lateral_acceleration_,
                                      processing_kinematic_limitations_max_longitudinal_acceleration_,
                                      processing_kinematic_limitations_max_longitudinal_deceleration_, follower.stop_at_route_end,
                                      std::nullopt, processing_kinematic_limitations_enable_);
        }
        speed_limit = std::min(speed_limit, profileSpeedAtDistance(unconstrained_profile->samples, previous.distance));
        const FollowingStepResult step = advanceFollowingStep(
            previous, next_time, nominal.distance, speed_limit, distance_limit, processing_following_headway_time_,
            follower.initial_speed, processing_kinematic_limitations_max_longitudinal_acceleration_,
            processing_kinematic_limitations_max_longitudinal_deceleration_, route_length);
        if (profile.feasible && !step.feasible) {
          RCLCPP_DEBUG_STREAM(this->get_logger().get_child("following"),
                              "object " << object.object.id << " hypothesis " << hypothesis_index << ": infeasible at "
                                        << next_time << " s, cannot keep the headway to " << limiting_leader << " (at "
                                        << previous.distance << " m, " << previous.speed << " m/s, allowed up to "
                                        << distance_limit << " m)");
        }
        profile.feasible = profile.feasible && step.feasible;
        profile.samples.push_back(step.sample);
        time = next_time;
      }
      if (following_active) results[object_index][hypothesis_index] = std::move(profile);
    }
  }

  // Commit only after every follower has read the same yielded leader profiles.
  for (std::size_t object_index = 0; object_index < prediction_objects.size(); ++object_index) {
    for (std::size_t hypothesis_index = 0; hypothesis_index < results[object_index].size(); ++hypothesis_index) {
      if (!results[object_index][hypothesis_index]) continue;
      PredictionObject::Hypothesis& hypothesis = prediction_objects[object_index].hypotheses[hypothesis_index];
      hypothesis.motion_profile = std::move(results[object_index][hypothesis_index]->samples);
      hypothesis.feasible = results[object_index][hypothesis_index]->feasible;
    }
  }
}

perception_msgs::msg::ObjectStatePrediction Lanelet2ObjectListPrediction::createStationaryPrediction(
    const perception_msgs::msg::Object& object, const builtin_interfaces::msg::Time& base_time) const {
  perception_msgs::msg::ObjectStatePrediction prediction;
  const std::size_t sample_count = getPredictionSampleCount();
  prediction.states.reserve(sample_count);

  geometry_msgs::msg::Point position = perception_msgs::object_access::getPosition(object);
  const double yaw = perception_msgs::object_access::getYaw(object);
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = 0.0;
  velocity.y = 0.0;
  velocity.z = 0.0;

  for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    perception_msgs::msg::ObjectState state = object.state;
    setPredictedStateKinematics(state, position.x, position.y, position.z, yaw, velocity, base_time, sample_index);
    prediction.states.push_back(state);
  }
  return prediction;
}

perception_msgs::msg::ObjectStatePrediction Lanelet2ObjectListPrediction::createConstantVelocityPrediction(
    const perception_msgs::msg::Object& object, const builtin_interfaces::msg::Time& base_time) const {
  perception_msgs::msg::ObjectStatePrediction prediction;
  const std::size_t sample_count = getPredictionSampleCount();
  prediction.states.reserve(sample_count);

  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Vector3 velocity;
  double yaw = 0.0;
  try {
    position = perception_msgs::object_access::getPosition(object);
    velocity = perception_msgs::object_access::getVelocityXYZ(object);
    yaw = perception_msgs::object_access::getYaw(object);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not read velocity for kinematic prediction, using "
                "static fallback: %s",
                ex.what());
    return createStationaryPrediction(object, base_time);
  }

  for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double time_offset = output_sample_interval_ * static_cast<double>(sample_index + 1);
    perception_msgs::msg::ObjectState state = object.state;
    setPredictedStateKinematics(state, position.x + velocity.x * time_offset, position.y + velocity.y * time_offset,
                                position.z + velocity.z * time_offset, yaw, velocity, base_time, sample_index);
    prediction.states.push_back(state);
  }
  return prediction;
}

perception_msgs::msg::ObjectState Lanelet2ObjectListPrediction::sampleStateOnLaneletRoute(
    const perception_msgs::msg::ObjectState& base_state,
    const SmoothedRoute& route,
    double travel_distance,
    double speed,
    double initial_speed,
    double initial_lateral_speed,
    bool reversing,
    const builtin_interfaces::msg::Time& base_time,
    std::size_t sample_index) const {
  perception_msgs::msg::ObjectState state = base_state;
  if (route.empty()) {
    RCLCPP_WARN(this->get_logger(), "Lanelet route is empty, cannot sample lanelet prediction");
    return state;
  }

  geometry_msgs::msg::Point fallback_position;
  try {
    fallback_position = perception_msgs::object_access::getPosition(base_state);
  } catch (const std::exception&) {
    fallback_position.x = 0.0;
    fallback_position.y = 0.0;
    fallback_position.z = 0.0;
  }
  const double route_length = route.length();
  const lanelet::BasicPoint2d initial_centerline_point = route.point(0.0);
  const double initial_yaw = route.yaw(0.0);
  const double lateral_offset = -std::sin(initial_yaw) * (fallback_position.x - initial_centerline_point.x()) +
                                std::cos(initial_yaw) * (fallback_position.y - initial_centerline_point.y());
  // Split the observed velocity along the smoothed route direction the path starts in, not along the lanelet's local direction
  double longitudinal_speed = initial_speed;
  double lateral_speed = initial_lateral_speed;
  if (!reversing && initial_speed > kKinematicEpsilon) {
    try {
      const LaneletVelocity route_velocity =
          velocityAlongLanelet(perception_msgs::object_access::getVelocityXYZ(base_state), initial_yaw);
      if (route_velocity.longitudinal > kKinematicEpsilon) {
        longitudinal_speed = route_velocity.longitudinal;
        lateral_speed = route_velocity.lateral;
      }
    } catch (const std::exception&) {
    }
  }
  const double clamped_travel_distance = std::clamp(travel_distance, 0.0, route_length);
  const double convergence_distance =
      processing_kinematic_limitations_enable_
          ? lateralConvergenceDistance(lateral_offset, longitudinal_speed,
                                       processing_kinematic_limitations_max_lateral_acceleration_, lateral_speed)
          : kRouteProfileResolutionM;
  const double initial_slope = longitudinal_speed > kKinematicEpsilon ? lateral_speed / longitudinal_speed : 0.0;
  const lanelet::BasicPoint2d point =
      route.convergingPoint(clamped_travel_distance, lateral_offset, convergence_distance, initial_slope);
  const double before_distance = std::max(0.0, clamped_travel_distance - kMotionTangentSampleDistanceM);
  const double after_distance = std::min(route_length, clamped_travel_distance + kMotionTangentSampleDistanceM);
  geometry_msgs::msg::Vector3 velocity;
  double path_yaw = route.yaw(clamped_travel_distance);
  if (after_distance - before_distance > kKinematicEpsilon) {
    const lanelet::BasicPoint2d before =
        route.convergingPoint(before_distance, lateral_offset, convergence_distance, initial_slope);
    const lanelet::BasicPoint2d after =
        route.convergingPoint(after_distance, lateral_offset, convergence_distance, initial_slope);
    velocity.x = speed * (after.x() - before.x()) / (after_distance - before_distance);
    velocity.y = speed * (after.y() - before.y()) / (after_distance - before_distance);
    path_yaw = std::atan2(after.y() - before.y(), after.x() - before.x());
  } else {
    velocity.x = speed * std::cos(path_yaw);
    velocity.y = speed * std::sin(path_yaw);
  }
  velocity.z = 0.0;

  // The body can face opposite its motion when reversing. Preserve its observed
  // heading initially and converge to its final lane alignment with the path.
  double body_yaw = path_yaw + (reversing ? M_PI : 0.0);
  try {
    const double observed_yaw = perception_msgs::object_access::getYaw(base_state);
    const double progress =
        convergence_distance <= kKinematicEpsilon ? 1.0 : std::clamp(clamped_travel_distance / convergence_distance, 0.0, 1.0);
    body_yaw = observed_yaw + progress * std::remainder(body_yaw - observed_yaw, 2.0 * M_PI);
  } catch (const std::exception&) {
  }
  setPredictedStateKinematics(state, point.x(), point.y(), fallback_position.z, body_yaw, velocity, base_time, sample_index);
  return state;
}

std::size_t Lanelet2ObjectListPrediction::getPredictionSampleCount() const {
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(output_prediction_horizon_ / output_sample_interval_)));
}

void Lanelet2ObjectListPrediction::setPredictedStateKinematics(perception_msgs::msg::ObjectState& state,
                                                               double x,
                                                               double y,
                                                               double z,
                                                               double yaw,
                                                               const geometry_msgs::msg::Vector3& velocity,
                                                               const builtin_interfaces::msg::Time& base_time,
                                                               std::size_t sample_index) const {
  state.header.frame_id = ll2_interface_->map_frame_id_;
  const rclcpp::Time stamp(base_time);
  const double time_offset = output_sample_interval_ * static_cast<double>(sample_index + 1);
  const rclcpp::Time future_stamp = stamp + rclcpp::Duration::from_seconds(time_offset);
  const int64_t future_nanoseconds = future_stamp.nanoseconds();
  state.header.stamp.sec = static_cast<int32_t>(future_nanoseconds / 1000000000);
  state.header.stamp.nanosec = static_cast<uint32_t>(future_nanoseconds % 1000000000);

  geometry_msgs::msg::Point position;
  position.x = x;
  position.y = y;
  position.z = z;
  perception_msgs::object_access::setPosition(state, position, false);

  try {
    perception_msgs::object_access::setVelocityXYZYaw(state, velocity, yaw, false);
  } catch (const std::exception& ex) {
    RCLCPP_DEBUG(this->get_logger(), "Could not set predicted velocity, setting yaw only: %s", ex.what());
    perception_msgs::object_access::setYaw(state, yaw, false);
  }
}

void Lanelet2ObjectListPrediction::rebuildRoutingGraphFromMap() {
  routing_graph_.reset();
  bicycle_routing_graph_.reset();
  pedestrian_routing_graph_.reset();
  traffic_rules_.reset();
  bicycle_traffic_rules_.reset();
  pedestrian_traffic_rules_.reset();
  routing_graph_map_ = ll2_interface_->getMapPtr();
  if (routing_graph_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map pointer is null, cannot build routing graph");
    return;
  }

  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(static_cast<const char*>(lanelet::Locations::Germany),
                                                                       static_cast<const char*>(lanelet::Participants::Vehicle));
  routing_graph_ = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *traffic_rules_);
  bicycle_traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Bicycle));
  bicycle_routing_graph_ = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *bicycle_traffic_rules_);
  pedestrian_traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Pedestrian));
  pedestrian_routing_graph_ = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *pedestrian_traffic_rules_);

  RCLCPP_INFO(this->get_logger(), "Built lanelet2 routing graph");
}

bool Lanelet2ObjectListPrediction::checkMap(bool handle_update) {
  bool map_status = ll2_interface_->map_loaded_;
  // update routing graph on map update
  if (handle_update && ll2_interface_->update_pending_ && ll2_interface_->map_loaded_) {
    ll2_interface_->update_pending_ = false;
    map_status = map_status && !ll2_interface_->update_pending_;
  }
  if (map_status && handle_update && routing_graph_map_ != ll2_interface_->getMapPtr()) {
    rebuildRoutingGraphFromMap();
  }
  return map_status;
}

}  // namespace lanelet2_object_list_prediction

/**
 * @brief Initializes ROS, spins the prediction node, and shuts down on exit
 *
 * @param[in] argc number of command-line arguments
 * @param[in] argv command-line arguments
 * @return process exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lanelet2_object_list_prediction::Lanelet2ObjectListPrediction>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
