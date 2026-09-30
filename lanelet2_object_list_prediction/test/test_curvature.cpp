// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
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

TEST(RouteSampling, RejectsObservedTransientSidewaysJump) {
  geometry_msgs::msg::Point observed;
  observed.x = 2293.33;
  observed.y = 1548.35;
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = -0.29;
  velocity.y = 4.10;
  geometry_msgs::msg::Point predicted;
  predicted.x = 2291.17;
  predicted.y = 1550.44;

  EXPECT_FALSE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0));
}

TEST(RouteSampling, AcceptsReachableInitialDisplacement) {
  geometry_msgs::msg::Point observed;
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = 0.0;
  velocity.y = 10.0;
  geometry_msgs::msg::Point predicted;
  predicted.x = -0.25;  // 2 m/s² lateral acceleration over 0.5 s.
  predicted.y = 4.875;  // 1 m/s² longitudinal deceleration over 0.5 s.

  EXPECT_TRUE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0));
}

TEST(RouteSampling, AllowsSmallRoundaboutAlignmentErrorOnly) {
  geometry_msgs::msg::Point observed;
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = 6.0;
  geometry_msgs::msg::Point predicted;
  predicted.x = 2.1875;  // Apparent 6.5 m/s² braking over the first 0.5 s.
  predicted.y = 0.375;   // Apparent 3 m/s² lateral acceleration.

  EXPECT_FALSE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0));
  EXPECT_TRUE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0, 1.0));

  predicted.y = 2.0;  // A sharp sideways turn still exceeds the alignment allowance.
  EXPECT_FALSE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0, 1.0));
}

TEST(RouteSampling, RoundaboutAllowanceAppliesOnlyNearTaggedLanelets) {
  lanelet::Lanelet approach = makeLaneletWithCenterline(30, {{0.0, 0.0}, {20.0, 0.0}});
  lanelet::Lanelet roundabout = makeLaneletWithCenterline(31, {{20.0, 0.0}, {30.0, 0.0}});
  roundabout.setAttribute("intersection_type", "roundabout");
  const lanelet::routing::LaneletPath route({approach, roundabout});

  EXPECT_FALSE(startsNearRoundabout(route, 0.0));
  EXPECT_TRUE(startsNearRoundabout(route, 15.0));
  EXPECT_TRUE(startsNearRoundabout(route, 20.0));
  EXPECT_TRUE(startsNearRoundabout(lanelet::routing::LaneletPath({roundabout}), 0.0));
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

TEST(BicycleMatching, UsesBicycleRulesForDedicatedLane) {
  lanelet::Lanelet bicycle_lane = makeLaneletWithCenterline(20, {{0.0, 0.0}, {20.0, 0.0}});
  bicycle_lane.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::BicycleLane);
  const auto bicycle_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Bicycle));
  const auto vehicle_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));

  EXPECT_TRUE(isBicycleClass(perception_msgs::msg::ObjectClassification::BICYCLE));
  EXPECT_TRUE(isBicycleClass(perception_msgs::msg::ObjectClassification::MICRO));
  EXPECT_FALSE(isBicycleClass(perception_msgs::msg::ObjectClassification::MOTORCYCLE));
  EXPECT_TRUE(isMotorcycleClass(perception_msgs::msg::ObjectClassification::MOTORCYCLE));
  EXPECT_TRUE(isBicycleLane(bicycle_lane));
  EXPECT_TRUE(bicycle_rules->canPass(bicycle_lane));
  EXPECT_FALSE(vehicle_rules->canPass(bicycle_lane));
}

TEST(BicycleMatching, RoutesWrongWayAlongBicycleLanePredecessors) {
  auto point = [](double x, double y) { return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0); };
  const lanelet::Point3d left_start = point(0.0, 1.0), left_middle = point(20.0, 1.0), left_end = point(40.0, 1.0);
  const lanelet::Point3d right_start = point(0.0, -1.0), right_middle = point(20.0, -1.0), right_end = point(40.0, -1.0);
  lanelet::Lanelet first(lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), {left_start, left_middle}),
                         lanelet::LineString3d(lanelet::utils::getId(), {right_start, right_middle}));
  lanelet::Lanelet second(lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), {left_middle, left_end}),
                          lanelet::LineString3d(lanelet::utils::getId(), {right_middle, right_end}));
  for (lanelet::Lanelet* lanelet : {&first, &second}) {
    lanelet->setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::BicycleLane);
    lanelet->setAttribute(lanelet::AttributeName::OneWay, true);
  }
  const auto map = lanelet::utils::createMap({first, second});
  const auto bicycle_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Bicycle));
  const auto routing_graph = lanelet::routing::RoutingGraph::build(*map, *bicycle_rules);

  // A cyclist on the second lanelet rides back towards the first one.
  const lanelet::ConstLanelet start = lanelet::ConstLanelet(second).invert();
  EXPECT_FALSE(bicycle_rules->canPass(start));
  EXPECT_TRUE(bicycle_rules->canPass(start.invert()));
  const lanelet::routing::LaneletPaths paths = wrongWayBicyclePaths(start, 10.0, *routing_graph);
  ASSERT_EQ(paths.size(), 1U);
  ASSERT_EQ(paths.front().size(), 2U);
  EXPECT_EQ(paths.front()[0].id(), second.id());
  EXPECT_EQ(paths.front()[1].id(), first.id());
  EXPECT_TRUE(paths.front()[1].inverted());
  EXPECT_NEAR(remainingRouteLength(paths.front(), 5.0), 35.0, 1e-6);
}

TEST(ParticipantMatching, UsesPedestrianRulesForSidewalkUsers) {
  lanelet::Lanelet walkway = makeLaneletWithCenterline(21, {{0.0, 0.0}, {20.0, 0.0}});
  walkway.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Walkway);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Pedestrian));
  const auto vehicle_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));

  EXPECT_TRUE(isPedestrianClass(perception_msgs::msg::ObjectClassification::PEDESTRIAN));
  EXPECT_TRUE(isPedestrianClass(perception_msgs::msg::ObjectClassification::VRU));
  EXPECT_TRUE(isPedestrianLane(walkway));
  EXPECT_TRUE(pedestrian_rules->canPass(walkway));
  EXPECT_FALSE(vehicle_rules->canPass(walkway));
}

}  // namespace
}  // namespace lanelet2_object_list_prediction
