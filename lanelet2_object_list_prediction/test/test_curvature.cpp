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
  const SmoothedRoute smoothed_route(route, 0.0);
  double maximum_curvature = 0.0;
  for (double distance = 0.0; distance <= smoothed_route.length(); distance += 0.25) {
    maximum_curvature = std::max(maximum_curvature, smoothed_route.curvature(distance));
  }
  return maximum_curvature;
}

TEST(RouteCurvature, SuppressesIsolatedLaneletBoundaryKink) {
  constexpr double kink_angle = 1.3 * M_PI / 180.0;
  lanelet::Lanelet first = makeLaneletWithCenterline(1, {{0.0, 0.0}, {10.0, 0.0}});
  lanelet::Lanelet second = makeLaneletWithCenterline(2, {{10.0, 0.0}, {20.0, 10.0 * std::tan(kink_angle)}});
  const lanelet::routing::LaneletPath route({first, second});

  EXPECT_LT(SmoothedRoute(route, 0.0).curvature(10.0), 2.5 / (13.0 * 13.0));
  EXPECT_TRUE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
}

TEST(RouteCurvature, SuppressesLaneletBoundaryKinkAtUrbanSpeed) {
  // Bag 06: a car at 15 m/s on a nearly straight road with a 3.6 deg heading step between lanelets.
  constexpr double kink_angle = 3.6 * M_PI / 180.0;
  lanelet::Lanelet first = makeLaneletWithCenterline(8, {{0.0, 0.0}, {20.0, 0.0}});
  lanelet::Lanelet second = makeLaneletWithCenterline(9, {{20.0, 0.0}, {40.0, 20.0 * std::tan(kink_angle)}});
  const lanelet::routing::LaneletPath route({first, second});

  EXPECT_LT(SmoothedRoute(route, 0.0).curvature(20.0), 3.0 / (15.0 * 15.0));
  EXPECT_TRUE(buildRouteMotionProfile(route, 0.0, 15.0, 3.0, 1.0, 2.0, false).feasible);
}

TEST(RouteCurvature, SuppressesShortLateralCenterlineStep) {
  lanelet::Lanelet first = makeLaneletWithCenterline(5, {{0.0, 0.0}, {10.0, 0.0}});
  lanelet::Lanelet step = makeLaneletWithCenterline(6, {{10.0, 0.0}, {10.2, 1.0}, {12.5, 1.2}});
  lanelet::Lanelet last = makeLaneletWithCenterline(7, {{12.5, 1.2}, {22.5, 1.2}});
  const lanelet::routing::LaneletPath route({first, step, last});
  const double route_length = remainingRouteLength(route, 0.0);

  EXPECT_TRUE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
  EXPECT_NEAR(SmoothedRoute(route, 0.0).point(route_length).y(), 0.0, 0.2);
}

TEST(RouteCurvature, PreservesSustainedCurve) {
  constexpr double radius = 10.0;
  std::vector<lanelet::BasicPoint2d> center_points;
  for (int degrees = -90; degrees <= 0; degrees += 5) {
    const double angle = static_cast<double>(degrees) * M_PI / 180.0;
    center_points.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
  }
  lanelet::Lanelet curve = makeLaneletWithCenterline(3, center_points);
  // Curvature is measured across several metres, so the curve is embedded in its approach and exit as in a map.
  lanelet::Lanelet approach = makeLaneletWithCenterline(lanelet::utils::getId(), {{-20.0, -radius}, {0.0, -radius}});
  lanelet::Lanelet exit = makeLaneletWithCenterline(lanelet::utils::getId(), {{radius, 0.0}, {radius, 20.0}});
  const lanelet::routing::LaneletPath route({approach, curve, exit});
  const double curve_middle = 20.0 + 0.25 * M_PI * radius;

  EXPECT_NEAR(SmoothedRoute(route, 0.0).curvature(curve_middle), 1.0 / radius, 0.005);
}

TEST(RouteCurvature, PreservesShortLeftAndRightTurns) {
  for (const bool left : {true, false}) {
    const lanelet::routing::LaneletPath route = makeQuarterTurnRoute(left);
    const double route_length = remainingRouteLength(route, 0.0);
    const lanelet::BasicPoint2d raw_end = pointOnRoute(route, 0.0, route_length);
    const SmoothedRoute smoothed_route(route, 0.0);
    const lanelet::BasicPoint2d smoothed_end = smoothed_route.point(route_length);

    EXPECT_GT(maximumRouteCurvature(route), 0.05);
    EXPECT_FALSE(buildRouteMotionProfile(route, 0.0, 13.0, 2.5, 1.0, 2.0, false).feasible);
    EXPECT_LT((smoothed_end - raw_end).norm(), 2.0);
    EXPECT_NEAR(smoothed_route.yaw(route_length), left ? M_PI_2 : -M_PI_2, 0.1);
  }
}

TEST(RouteSampling, EnforcedLaneFollowingCentersDetectionAndRouteGeometry) {
  lanelet::Lanelet lane = makeLaneletWithCenterline(42, {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}});
  perception_msgs::msg::ObjectState state;
  perception_msgs::object_access::initializeState(state, perception_msgs::EGO::MODEL_ID);
  geometry_msgs::msg::Point observed;
  observed.x = 3.0;
  observed.y = 0.7;
  observed.z = 1.5;
  perception_msgs::object_access::setPosition(state, observed);
  geometry_msgs::msg::Vector3 observed_velocity;
  observed_velocity.x = 4.0;
  observed_velocity.y = 1.0;
  perception_msgs::object_access::setVelocityXYZYaw(state, observed_velocity, 0.2);

  centerStateOnLanelet(state, lane, 3.0, 4.0, false);
  const auto position = perception_msgs::object_access::getPosition(state);
  const auto velocity = perception_msgs::object_access::getVelocityXYZ(state);
  EXPECT_NEAR(position.x, 3.0, 1e-6);
  EXPECT_NEAR(position.y, 0.0, 1e-6);
  EXPECT_NEAR(position.z, 1.5, 1e-6);
  EXPECT_NEAR(perception_msgs::object_access::getYaw(state), 0.0, 1e-6);
  EXPECT_NEAR(velocity.x, 4.0, 1e-6);
  EXPECT_NEAR(velocity.y, 0.0, 1e-6);

  const lanelet::routing::LaneletPath path({lane});
  const SmoothedRoute route(path, 3.0);
  for (double distance : {0.0, 2.0, 8.0, 12.0}) {
    const auto expected = pointOnRoute(path, 3.0, distance);
    const auto actual = route.centerlinePoint(distance);
    EXPECT_NEAR(actual.x(), expected.x(), 1e-6);
    EXPECT_NEAR(actual.y(), expected.y(), 1e-6);
  }
  EXPECT_NEAR(route.centerlineYaw(12.0), M_PI_2, 1e-6);
}

TEST(RouteSampling, FollowsTurnForObjectAlreadyInIt) {
  // A car halfway through a right turn, moving exactly along the lane.
  constexpr double radius = 8.0;
  std::vector<lanelet::BasicPoint2d> curve_points;
  for (int degrees = 90; degrees >= 0; degrees -= 5) {
    const double angle = static_cast<double>(degrees) * M_PI / 180.0;
    curve_points.emplace_back(radius * std::cos(angle), -radius + radius * std::sin(angle));
  }
  lanelet::Lanelet curve = makeLaneletWithCenterline(lanelet::utils::getId(), curve_points);
  lanelet::Lanelet last = makeLaneletWithCenterline(lanelet::utils::getId(), {{radius, -radius}, {radius, -radius - 25.0}});
  const lanelet::routing::LaneletPath route({curve, last});
  const double start_arc_length = 0.25 * M_PI * radius;
  const double route_length = remainingRouteLength(route, start_arc_length);

  const SmoothedRoute smoothed(route, start_arc_length);
  EXPECT_NEAR(smoothed.yaw(0.0), -M_PI_4, 0.05);
  for (double distance = 0.0; distance <= route_length; distance += 0.5) {
    EXPECT_LT((smoothed.point(distance) - pointOnRoute(route, start_arc_length, distance)).norm(), 0.2);
  }
  const lanelet::BasicPoint2d position = pointOnRoute(route, start_arc_length, 0.0);
  const lanelet::BasicPoint2d first_step = smoothed.point(1.25);
  geometry_msgs::msg::Point observed;
  observed.x = position.x();
  observed.y = position.y();
  geometry_msgs::msg::Point predicted;
  predicted.x = first_step.x();
  predicted.y = first_step.y();
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = 2.5 * std::cos(-M_PI_4);
  velocity.y = 2.5 * std::sin(-M_PI_4);
  EXPECT_TRUE(initialMotionFeasible(observed, velocity, predicted, 0.5, 2.5, 1.0, 2.0));
}

TEST(RouteSampling, SmoothedPointsDoNotDependOnQueryOrder) {
  // The smoothed centerline is integrated once and extended as far as a query needs it.
  const lanelet::routing::LaneletPath route = makeQuarterTurnRoute(true);
  const double route_length = remainingRouteLength(route, 3.3);
  const SmoothedRoute forward(route, 3.3);
  const SmoothedRoute backward(route, 3.3);
  std::vector<double> distances;
  for (int step = 0; 0.37 * step <= route_length + 1.0; ++step) distances.push_back(0.37 * step);
  std::vector<lanelet::BasicPoint2d> forward_points;
  for (const double distance : distances) forward_points.push_back(forward.point(distance));
  for (std::size_t index = distances.size(); index-- > 0;) {
    EXPECT_LT((backward.point(distances[index]) - forward_points[index]).norm(), 1e-9);
  }
  EXPECT_LT((forward.point(route_length + 1.0) - forward.point(route_length)).norm(), 1e-9);
}

TEST(RouteSampling, UsesPrecedingLaneletAtTurnEntry) {
  // A car has just entered a right-turn lanelet from a straight approach.
  constexpr double radius = 8.0;
  auto point = [](double x, double y) { return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0); };
  const lanelet::Point3d left_join = point(20.0, 1.5);
  const lanelet::Point3d right_join = point(20.0, -1.5);
  lanelet::Lanelet approach(lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), {point(0.0, 1.5), left_join}),
                            lanelet::LineString3d(lanelet::utils::getId(), {point(0.0, -1.5), right_join}));
  lanelet::Points3d left_points{left_join};
  lanelet::Points3d right_points{right_join};
  for (int degrees = 85; degrees >= 0; degrees -= 5) {
    const double angle = static_cast<double>(degrees) * M_PI / 180.0;
    left_points.push_back(point(20.0 + (radius + 1.5) * std::cos(angle), -radius + (radius + 1.5) * std::sin(angle)));
    right_points.push_back(point(20.0 + (radius - 1.5) * std::cos(angle), -radius + (radius - 1.5) * std::sin(angle)));
  }
  lanelet::Lanelet turn(lanelet::utils::getId(), lanelet::LineString3d(lanelet::utils::getId(), left_points),
                        lanelet::LineString3d(lanelet::utils::getId(), right_points));
  const auto map = lanelet::utils::createMap({approach, turn});
  const auto vehicle_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));
  const auto routing_graph = lanelet::routing::RoutingGraph::build(*map, *vehicle_rules);

  const lanelet::routing::LaneletPath route({turn});
  constexpr double start_arc_length = 0.5;
  const auto [geometry_route, geometry_start_arc_length] = routeGeometryWithPredecessor(route, start_arc_length, *routing_graph);
  ASSERT_EQ(geometry_route.size(), 2U);
  EXPECT_EQ(geometry_route.front().id(), approach.id());
  EXPECT_NEAR(geometry_start_arc_length, 20.0 + start_arc_length, 1e-6);

  // Looking only ahead, the heading at the car is taken from the turn.
  const double route_length = remainingRouteLength(route, start_arc_length);
  EXPECT_LT(SmoothedRoute(route, start_arc_length).yaw(0.0), -0.4);
  // With the approach, it matches the car's heading at the turn entry.
  EXPECT_NEAR(SmoothedRoute(geometry_route, geometry_start_arc_length).yaw(0.0), 0.0, 0.15);
  EXPECT_NEAR(remainingRouteLength(geometry_route, geometry_start_arc_length), route_length, 1e-6);
}

TEST(RouteSampling, SmoothlyConvergesInitialLateralOffset) {
  lanelet::Lanelet straight = makeLaneletWithCenterline(4, {{0.0, 0.0}, {20.0, 0.0}});
  const SmoothedRoute route(lanelet::routing::LaneletPath({straight}), 0.0);

  const double convergence_distance = lateralConvergenceDistance(1.5, 10.0, 9.0);
  const lanelet::BasicPoint2d initial = route.convergingPoint(0.0, 1.5, convergence_distance);
  const lanelet::BasicPoint2d halfway = route.convergingPoint(5.0, 1.5, convergence_distance);
  const lanelet::BasicPoint2d converged = route.convergingPoint(10.0, 1.5, convergence_distance);
  EXPECT_NEAR(convergence_distance, 10.0, 1e-6);
  EXPECT_NEAR(initial.y(), 1.5, 1e-6);
  EXPECT_NEAR(halfway.y(), 0.75, 1e-6);
  EXPECT_NEAR(converged.y(), 0.0, 1e-6);
  const lanelet::BasicPoint2d before = route.convergingPoint(4.99, 1.5, convergence_distance);
  const lanelet::BasicPoint2d after = route.convergingPoint(5.01, 1.5, convergence_distance);
  EXPECT_LT(after.y() - before.y(), 0.0);
}

TEST(RouteSampling, MoreLateralAccelerationConvergesSooner) {
  EXPECT_LT(lateralConvergenceDistance(1.5, 10.0, 9.0), lateralConvergenceDistance(1.5, 10.0, 2.25));
}

TEST(RouteSampling, StartsAlongMeasuredLateralVelocity) {
  lanelet::Lanelet straight = makeLaneletWithCenterline(9, {{0.0, 0.0}, {100.0, 0.0}});
  const SmoothedRoute route(lanelet::routing::LaneletPath({straight}), 0.0);
  const double distance = lateralConvergenceDistance(1.0, 10.0, 2.5, 2.0);
  const double slope = 2.0 / 10.0;
  const lanelet::BasicPoint2d initial = route.convergingPoint(0.0, 1.0, distance, slope);
  const lanelet::BasicPoint2d next = route.convergingPoint(0.001, 1.0, distance, slope);
  const lanelet::BasicPoint2d end = route.convergingPoint(distance, 1.0, distance, slope);

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
  EXPECT_NEAR(std::abs(SmoothedRoute(reverse_route, 10.0).yaw(0.0)), M_PI, 1e-6);
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
  const SmoothedRoute route(lanelet::routing::LaneletPath({straight}), 0.0);
  const double convergence_distance = lateralConvergenceDistance(1.5, 0.0, 2.5);

  const lanelet::BasicPoint2d point = route.convergingPoint(0.0, 1.5, convergence_distance);
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
