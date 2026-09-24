// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

#define main lanelet2_object_list_prediction_main
#include "../src/lanelet2_object_list_prediction.cpp"
#undef main

namespace lanelet2_object_list_prediction {
namespace {

lanelet::Lanelet makeLaneletWithCenterline(lanelet::Id id, const std::vector<lanelet::BasicPoint2d>& center_points) {
  lanelet::Points3d left_points;
  lanelet::Points3d right_points;
  lanelet::Points3d centerline_points;
  for (const auto& point : center_points) {
    left_points.emplace_back(lanelet::utils::getId(), point.x(), point.y() + 1.0, 0.0);
    right_points.emplace_back(lanelet::utils::getId(), point.x(), point.y() - 1.0, 0.0);
    centerline_points.emplace_back(lanelet::utils::getId(), point.x(), point.y(), 0.0);
  }

  lanelet::Lanelet lanelet(id, lanelet::LineString3d(lanelet::utils::getId(), left_points),
                           lanelet::LineString3d(lanelet::utils::getId(), right_points));
  lanelet.setCenterline(lanelet::LineString3d(lanelet::utils::getId(), centerline_points));
  return lanelet;
}

lanelet::routing::LaneletPath makeQuarterTurnRoute(bool left) {
  lanelet::Lanelet first = makeLaneletWithCenterline(lanelet::utils::getId(), {{0.0, 0.0}, {15.0, 0.0}});
  std::vector<lanelet::BasicPoint2d> curve_points;
  if (left) {
    for (int degrees = -90; degrees <= 0; degrees += 5) {
      const double angle = static_cast<double>(degrees) * M_PI / 180.0;
      curve_points.emplace_back(15.0 + 5.0 * std::cos(angle), 5.0 + 5.0 * std::sin(angle));
    }
  } else {
    for (int degrees = 90; degrees >= 0; degrees -= 5) {
      const double angle = static_cast<double>(degrees) * M_PI / 180.0;
      curve_points.emplace_back(15.0 + 5.0 * std::cos(angle), -5.0 + 5.0 * std::sin(angle));
    }
  }
  lanelet::Lanelet curve = makeLaneletWithCenterline(lanelet::utils::getId(), curve_points);
  const double final_y = left ? 20.0 : -20.0;
  lanelet::Lanelet last = makeLaneletWithCenterline(lanelet::utils::getId(), {{20.0, left ? 5.0 : -5.0}, {20.0, final_y}});
  return lanelet::routing::LaneletPath({first, curve, last});
}

double maximumRouteCurvature(const lanelet::routing::LaneletPath& route) {
  const double route_length = remainingRouteLength(route, 0.0);
  double maximum_curvature = 0.0;
  for (double distance = 0.0; distance <= route_length; distance += 0.25) {
    maximum_curvature = std::max(maximum_curvature, routeCurvature(route, 0.0, route_length, distance));
  }
  return maximum_curvature;
}

TEST(RouteCurvature, SuppressesIsolatedLaneletBoundaryKink) {
  constexpr double kink_angle = 1.3 * M_PI / 180.0;
  lanelet::Lanelet first = makeLaneletWithCenterline(1, {{0.0, 0.0}, {10.0, 0.0}});
  lanelet::Lanelet second = makeLaneletWithCenterline(2, {{10.0, 0.0}, {20.0, 10.0 * std::tan(kink_angle)}});
  const lanelet::routing::LaneletPath route({first, second});
  const double route_length = remainingRouteLength(route, 0.0);

  EXPECT_LT(routeCurvature(route, 0.0, route_length, 10.0), 2.5 / (13.0 * 13.0));
  EXPECT_TRUE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
}

TEST(RouteCurvature, SuppressesShortLateralCenterlineStep) {
  lanelet::Lanelet first = makeLaneletWithCenterline(5, {{0.0, 0.0}, {10.0, 0.0}});
  lanelet::Lanelet step = makeLaneletWithCenterline(6, {{10.0, 0.0}, {10.2, 1.0}, {12.5, 1.2}});
  lanelet::Lanelet last = makeLaneletWithCenterline(7, {{12.5, 1.2}, {22.5, 1.2}});
  const lanelet::routing::LaneletPath route({first, step, last});
  const double route_length = remainingRouteLength(route, 0.0);

  EXPECT_TRUE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
  EXPECT_NEAR(smoothedPointOnRoute(route, 0.0, route_length, route_length).y(), 0.0, 0.2);
}

TEST(RouteCurvature, PreservesSustainedCurve) {
  constexpr double radius = 10.0;
  std::vector<lanelet::BasicPoint2d> center_points;
  for (int degrees = -90; degrees <= 0; degrees += 5) {
    const double angle = static_cast<double>(degrees) * M_PI / 180.0;
    center_points.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
  }
  lanelet::Lanelet curve = makeLaneletWithCenterline(3, center_points);
  const lanelet::routing::LaneletPath route({curve});
  const double route_length = remainingRouteLength(route, 0.0);

  EXPECT_NEAR(routeCurvature(route, 0.0, route_length, route_length / 2.0), 1.0 / radius, 0.005);
}

TEST(RouteCurvature, PreservesShortLeftAndRightTurns) {
  for (const bool left : {true, false}) {
    const lanelet::routing::LaneletPath route = makeQuarterTurnRoute(left);
    const double route_length = remainingRouteLength(route, 0.0);
    const lanelet::BasicPoint2d raw_end = pointOnRoute(route, 0.0, route_length);
    const lanelet::BasicPoint2d smoothed_end = smoothedPointOnRoute(route, 0.0, route_length, route_length);

    EXPECT_GT(maximumRouteCurvature(route), 0.05);
    EXPECT_FALSE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
    EXPECT_LT((smoothed_end - raw_end).norm(), 2.0);
    EXPECT_NEAR(routeYaw(route, 0.0, route_length, route_length), left ? M_PI_2 : -M_PI_2, 0.1);
  }
}

TEST(RouteSampling, SmoothlyConvergesInitialLateralOffset) {
  lanelet::Lanelet straight = makeLaneletWithCenterline(4, {{0.0, 0.0}, {20.0, 0.0}});
  const lanelet::routing::LaneletPath route({straight});
  const double route_length = remainingRouteLength(route, 0.0);

  const double convergence_distance = lateralConvergenceDistance(1.5, 10.0, 9.0);
  const lanelet::BasicPoint2d initial = pointOnConvergingRoute(route, 0.0, route_length, 0.0, 1.5, convergence_distance);
  const lanelet::BasicPoint2d halfway = pointOnConvergingRoute(route, 0.0, route_length, 5.0, 1.5, convergence_distance);
  const lanelet::BasicPoint2d converged = pointOnConvergingRoute(route, 0.0, route_length, 10.0, 1.5, convergence_distance);
  EXPECT_NEAR(convergence_distance, 10.0, 1e-6);
  EXPECT_NEAR(initial.y(), 1.5, 1e-6);
  EXPECT_NEAR(halfway.y(), 0.75, 1e-6);
  EXPECT_NEAR(converged.y(), 0.0, 1e-6);
  const lanelet::BasicPoint2d before = pointOnConvergingRoute(route, 0.0, route_length, 4.99, 1.5, convergence_distance);
  const lanelet::BasicPoint2d after = pointOnConvergingRoute(route, 0.0, route_length, 5.01, 1.5, convergence_distance);
  EXPECT_LT(after.y() - before.y(), 0.0);
}

TEST(RouteSampling, MoreLateralAccelerationConvergesSooner) {
  EXPECT_LT(lateralConvergenceDistance(1.5, 10.0, 9.0), lateralConvergenceDistance(1.5, 10.0, 2.25));
}

TEST(RouteSampling, StartsAlongMeasuredLateralVelocity) {
  lanelet::Lanelet straight = makeLaneletWithCenterline(9, {{0.0, 0.0}, {100.0, 0.0}});
  const lanelet::routing::LaneletPath route({straight});
  const double route_length = remainingRouteLength(route, 0.0);
  const double distance = lateralConvergenceDistance(1.0, 10.0, 2.5, 2.0);
  const double slope = 2.0 / 10.0;
  const lanelet::BasicPoint2d initial = pointOnConvergingRoute(route, 0.0, route_length, 0.0, 1.0, distance, slope);
  const lanelet::BasicPoint2d next = pointOnConvergingRoute(route, 0.0, route_length, 0.001, 1.0, distance, slope);
  const lanelet::BasicPoint2d end = pointOnConvergingRoute(route, 0.0, route_length, distance, 1.0, distance, slope);

  EXPECT_NEAR((next.y() - initial.y()) / (next.x() - initial.x()), slope, 1e-3);
  EXPECT_NEAR(end.y(), 0.0, 1e-6);
  const double first_acceleration = 10.0 * 10.0 * std::abs(-6.0 / (distance * distance) - 4.0 * slope / distance);
  const double last_acceleration = 10.0 * 10.0 * std::abs(6.0 / (distance * distance) + 2.0 * slope / distance);
  EXPECT_LE(std::max(first_acceleration, last_acceleration), 2.5 + 1e-6);
}

TEST(RouteSampling, ReversingVelocityPointsAgainstBodyHeading) {
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = -5.0;
  velocity.y = 1.0;
  const LaneletVelocity legal_lane_velocity = velocityAlongLanelet(velocity, 0.0);
  const LaneletVelocity reverse_route_velocity = velocityAlongLanelet(velocity, M_PI);

  EXPECT_LT(legal_lane_velocity.longitudinal, 0.0);
  EXPECT_NEAR(reverse_route_velocity.longitudinal, 5.0, 1e-6);
  EXPECT_NEAR(reverse_route_velocity.lateral, -1.0, 1e-6);

  lanelet::Lanelet straight = makeLaneletWithCenterline(10, {{0.0, 0.0}, {20.0, 0.0}});
  const lanelet::routing::LaneletPath reverse_route({straight.invert()});
  const lanelet::BasicPoint2d next = pointOnRoute(reverse_route, 10.0, 1.0);
  EXPECT_NEAR(next.x(), 9.0, 1e-6);
  EXPECT_NEAR(std::abs(routeYaw(reverse_route, 10.0, 10.0, 0.0)), M_PI, 1e-6);
}

TEST(RouteSampling, StationaryObjectRetainsItsLateralOffset) {
  lanelet::Lanelet straight = makeLaneletWithCenterline(8, {{0.0, 0.0}, {20.0, 0.0}});
  const lanelet::routing::LaneletPath route({straight});
  const double route_length = remainingRouteLength(route, 0.0);
  const double convergence_distance = lateralConvergenceDistance(1.5, 0.0, 2.5);

  const lanelet::BasicPoint2d point = pointOnConvergingRoute(route, 0.0, route_length, 0.0, 1.5, convergence_distance);
  EXPECT_TRUE(std::isinf(convergence_distance));
  EXPECT_NEAR(point.y(), 1.5, 1e-6);
}

}  // namespace
}  // namespace lanelet2_object_list_prediction
