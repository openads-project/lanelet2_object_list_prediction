// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#define main lanelet2_object_list_prediction_main
#include "../src/lanelet2_object_list_prediction.cpp"
#undef main

namespace lanelet2_object_list_prediction {
namespace {

lanelet::Lanelet makeStraightLanelet(lanelet::Id id, double y_offset = 0.0) {
  lanelet::LineString3d left(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0.0, y_offset + 1.0, 0.0),
                                                       lanelet::Point3d(lanelet::utils::getId(), 20.0, y_offset + 1.0, 0.0)});
  lanelet::LineString3d right(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0.0, y_offset - 1.0, 0.0),
                                                        lanelet::Point3d(lanelet::utils::getId(), 20.0, y_offset - 1.0, 0.0)});
  return lanelet::Lanelet(id, left, right);
}

TEST(YieldMotionProfile, StopsWaitsAndAcceleratesAfterRelease) {
  const lanelet::routing::LaneletPath route({makeStraightLanelet(1)});
  const RouteMotionProfile profile = buildRouteMotionProfile(route, 0.0, 4.0, 2.5, 1.0, 2.0, false, YieldConstraint{6.0, 5.0});

  ASSERT_TRUE(profile.feasible);
  const RouteMotionSample waiting = sampleRouteMotionAtTime(profile.samples, 4.5);
  EXPECT_NEAR(waiting.distance, 6.0, 1e-6);
  EXPECT_NEAR(waiting.speed, 0.0, 1e-6);

  const RouteMotionSample resumed = sampleRouteMotionAtTime(profile.samples, 6.0);
  EXPECT_GT(resumed.distance, 6.0);
  EXPECT_GT(resumed.speed, 0.0);
  EXPECT_LE(resumed.speed, 1.0 + 1e-6);
}

TEST(YieldMotionProfile, MarksUnavoidableStopInfeasibleWithoutPositionJump) {
  const lanelet::routing::LaneletPath route({makeStraightLanelet(2)});
  const RouteMotionProfile profile = buildRouteMotionProfile(route, 0.0, 5.0, 2.5, 1.0, 2.0, false, YieldConstraint{0.5, 3.0});

  EXPECT_FALSE(profile.feasible);
  const RouteMotionSample before = sampleRouteMotionAtTime(profile.samples, 0.05);
  const RouteMotionSample after = sampleRouteMotionAtTime(profile.samples, 0.15);
  EXPECT_GT(after.distance, before.distance);
  EXPECT_GT(after.speed, 0.0);
}

TEST(YieldMotionProfile, NominalProfileDoesNotStopWithoutConstraint) {
  const lanelet::routing::LaneletPath route({makeStraightLanelet(3)});
  const RouteMotionProfile profile = buildRouteMotionProfile(route, 0.0, 3.0, 2.5, 1.0, 2.0, false);

  ASSERT_TRUE(profile.feasible);
  const RouteMotionSample sample = sampleRouteMotionAtTime(profile.samples, 2.0);
  EXPECT_NEAR(sample.distance, 6.0, 0.05);
  EXPECT_NEAR(sample.speed, 3.0, 1e-6);
}

TEST(RightOfWay, ExposesYieldLineAndPriorityLanelets) {
  lanelet::Lanelet priority = makeStraightLanelet(10, 4.0);
  lanelet::Lanelet yielding = makeStraightLanelet(11);
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 5.0, -1.0, 0.0),
                                                            lanelet::Point3d(lanelet::utils::getId(), 5.0, 1.0, 0.0)});
  lanelet::AttributeMap attributes;
  attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  attributes[lanelet::AttributeName::Subtype] = lanelet::RightOfWay::RuleName;
  const lanelet::RightOfWay::Ptr rule =
      lanelet::RightOfWay::make(lanelet::utils::getId(), attributes, {priority}, {yielding}, stop_line);
  yielding.addRegulatoryElement(rule);

  const auto rules = yielding.regulatoryElementsAs<lanelet::RightOfWay>();
  ASSERT_EQ(rules.size(), 1U);
  EXPECT_EQ(rules.front()->getManeuver(yielding), lanelet::ManeuverType::Yield);
  ASSERT_EQ(rules.front()->rightOfWayLanelets().size(), 1U);
  EXPECT_EQ(rules.front()->rightOfWayLanelets().front().id(), priority.id());
  ASSERT_TRUE(rules.front()->stopLine().has_value());
}

TEST(RightOfWay, ProjectsReferenceLineBeyondRegulatingLanelet) {
  lanelet::Lanelet approach = makeStraightLanelet(20);
  lanelet::LineString3d left(lanelet::utils::getId(),
                             {approach.leftBound().back(), lanelet::Point3d(lanelet::utils::getId(), 40.0, 1.0, 0.0)});
  lanelet::LineString3d right(lanelet::utils::getId(),
                              {approach.rightBound().back(), lanelet::Point3d(lanelet::utils::getId(), 40.0, -1.0, 0.0)});
  lanelet::Lanelet successor(21, left, right);
  const lanelet::routing::LaneletPath full_route({approach, successor});
  const lanelet::routing::LaneletPath successor_route({successor});
  lanelet::LineString3d reference_line(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 25.0, -1.0, 0.0),
                                                                 lanelet::Point3d(lanelet::utils::getId(), 25.0, 1.0, 0.0)});

  const auto from_approach = projectYieldLineOnRoute(full_route, 0.0, 0, reference_line);
  ASSERT_TRUE(from_approach.has_value());
  EXPECT_EQ(from_approach->lanelet_index, 1U);
  EXPECT_NEAR(from_approach->distance, 25.0, 1e-6);
  const RouteMotionProfile nominal = buildRouteMotionProfile(full_route, 0.0, 4.0, 2.5, 1.0, 2.0, false);
  const double stop_distance = from_approach->distance - 2.5;
  const double successor_end = routeDistanceAtLaneletStart(full_route, 0.0, from_approach->lanelet_index) +
                               static_cast<double>(lanelet::geometry::length(successor.centerline2d()));
  const TimeInterval yielding_interval{timeAtRouteDistance(nominal.samples, stop_distance),
                                       timeAtRouteDistance(nominal.samples, successor_end)};
  EXPECT_TRUE(intervalsOverlap(yielding_interval, TimeInterval{6.0, 7.0}));

  const auto from_successor = projectYieldLineOnRoute(successor_route, 2.0, 0, reference_line);
  ASSERT_TRUE(from_successor.has_value());
  EXPECT_NEAR(from_successor->distance, 3.0, 1e-6);
  EXPECT_FALSE(projectYieldLineOnRoute(successor_route, 7.0, 0, reference_line).has_value());
}

TEST(RightOfWay, FindsRegulatedPredecessorForSuccessorRoute) {
  lanelet::Lanelet approach = makeStraightLanelet(22);
  lanelet::LineString3d left(lanelet::utils::getId(),
                             {approach.leftBound().back(), lanelet::Point3d(lanelet::utils::getId(), 40.0, 1.0, 0.0)});
  lanelet::LineString3d right(lanelet::utils::getId(),
                              {approach.rightBound().back(), lanelet::Point3d(lanelet::utils::getId(), 40.0, -1.0, 0.0)});
  lanelet::Lanelet successor(23, left, right);
  lanelet::Lanelet priority = makeStraightLanelet(24, 4.0);
  lanelet::LineString3d reference_line(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 25.0, -1.0, 0.0),
                                                                 lanelet::Point3d(lanelet::utils::getId(), 25.0, 1.0, 0.0)});
  lanelet::AttributeMap attributes;
  attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  attributes[lanelet::AttributeName::Subtype] = lanelet::RightOfWay::RuleName;
  approach.addRegulatoryElement(
      lanelet::RightOfWay::make(lanelet::utils::getId(), attributes, {priority}, {approach}, reference_line));
  lanelet::LaneletMap map;
  map.add(approach);
  map.add(successor);
  map.add(priority);
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));
  const auto graph = lanelet::routing::RoutingGraph::build(map, *traffic_rules);
  const auto predecessors = graph->previous(successor);
  ASSERT_EQ(predecessors.size(), 1U);
  EXPECT_EQ(predecessors.front().id(), approach.id());
  EXPECT_EQ(predecessors.front().regulatoryElementsAs<lanelet::RightOfWay>().size(), 1U);
}

TEST(RightOfWay, SharedApproachOnlyAppliesToConflictingBranch) {
  auto point = [](double x, double y) { return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0); };
  const auto approach_left_start = point(0.0, 1.0);
  const auto approach_right_start = point(0.0, -1.0);
  const auto approach_left_end = point(10.0, 1.0);
  const auto approach_right_end = point(10.0, -1.0);
  lanelet::Lanelet approach(40, lanelet::LineString3d(lanelet::utils::getId(), {approach_left_start, approach_left_end}),
                            lanelet::LineString3d(lanelet::utils::getId(), {approach_right_start, approach_right_end}));
  lanelet::Lanelet straight(41, lanelet::LineString3d(lanelet::utils::getId(), {approach_left_end, point(20.0, 1.0)}),
                            lanelet::LineString3d(lanelet::utils::getId(), {approach_right_end, point(20.0, -1.0)}));
  lanelet::Lanelet turning(42, lanelet::LineString3d(lanelet::utils::getId(), {approach_left_end, point(20.0, 11.0)}),
                           lanelet::LineString3d(lanelet::utils::getId(), {approach_right_end, point(20.0, 9.0)}));
  lanelet::Lanelet later_crossing(
      45, lanelet::LineString3d(lanelet::utils::getId(), {straight.leftBound().back(), point(10.0, 11.0)}),
      lanelet::LineString3d(lanelet::utils::getId(), {straight.rightBound().back(), point(10.0, 9.0)}));
  const auto priority_left_start = point(16.0, 20.0);
  const auto priority_right_start = point(14.0, 20.0);
  const auto priority_left_end = point(16.0, 10.0);
  const auto priority_right_end = point(14.0, 10.0);
  lanelet::Lanelet priority_approach(43, lanelet::LineString3d(lanelet::utils::getId(), {priority_left_start, priority_left_end}),
                                     lanelet::LineString3d(lanelet::utils::getId(), {priority_right_start, priority_right_end}));
  lanelet::Lanelet priority_crossing(44, lanelet::LineString3d(lanelet::utils::getId(), {priority_left_end, point(16.0, 2.0)}),
                                     lanelet::LineString3d(lanelet::utils::getId(), {priority_right_end, point(14.0, 2.0)}));
  lanelet::AttributeMap attributes;
  attributes[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RegulatoryElement;
  attributes[lanelet::AttributeName::Subtype] = lanelet::RightOfWay::RuleName;
  const lanelet::LineString3d stop_line(lanelet::utils::getId(), {point(11.0, -1.0), point(11.0, 1.0)});
  const auto rule = lanelet::RightOfWay::make(lanelet::utils::getId(), attributes, {priority_approach}, {approach}, stop_line);
  approach.addRegulatoryElement(rule);
  priority_approach.addRegulatoryElement(rule);
  lanelet::LaneletMap map;
  for (const auto& lanelet : {approach, straight, turning, later_crossing, priority_approach, priority_crossing})
    map.add(lanelet);
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));
  const auto graph = lanelet::routing::RoutingGraph::build(map, *traffic_rules);

  ASSERT_EQ(graph->following(priority_approach).size(), 1U);
  EXPECT_EQ(graph->following(priority_approach).front().id(), priority_crossing.id());
  EXPECT_FALSE(laneletConflictsWithPriorityContinuation(straight, *rule, *graph));
  EXPECT_TRUE(laneletConflictsWithPriorityContinuation(turning, *rule, *graph));
  EXPECT_TRUE(laneletConflictsWithPriorityContinuation(later_crossing, *rule, *graph));
  const lanelet::routing::LaneletPath straight_route({approach, straight, later_crossing});
  const lanelet::routing::LaneletPath turning_route({approach, turning});
  EXPECT_FALSE(conflictLaneletAtYieldLine(straight_route, 0, 1, *rule, *graph).has_value());
  EXPECT_EQ(conflictLaneletAtYieldLine(turning_route, 0, 1, *rule, *graph), 1U);
}

TEST(YieldInteraction, DetectsOnlyOverlappingOccupancyIntervals) {
  EXPECT_TRUE(intervalsOverlap(TimeInterval{1.0, 3.0}, TimeInterval{2.0, 4.0}));
  EXPECT_FALSE(intervalsOverlap(TimeInterval{1.0, 2.0}, TimeInterval{2.1, 4.0}));
}

TEST(YieldInteraction, InterpolatesEgoAcrossShortPriorityLaneletAndKeepsConflictOccupied) {
  auto makeLanelet = [](lanelet::Id id, double start, double end) {
    lanelet::LineString3d left(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), start, 1.0, 0.0),
                                                         lanelet::Point3d(lanelet::utils::getId(), end, 1.0, 0.0)});
    lanelet::LineString3d right(lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), start, -1.0, 0.0),
                                                          lanelet::Point3d(lanelet::utils::getId(), end, -1.0, 0.0)});
    return lanelet::Lanelet(id, left, right);
  };
  const lanelet::Lanelet priority = makeLanelet(30, 0.0, 1.0);
  const lanelet::Lanelet conflict = makeLanelet(31, 3.0, 5.0);
  perception_msgs::msg::EgoData ego;
  ego.state.header.stamp.sec = 0;
  ego.state.model_id = 1;
  ego.state.continuous_state.resize(13);
  ego.state.continuous_state[0] = -1.0;
  perception_msgs::msg::ObjectState future;
  future.header.stamp.sec = 7;
  future.model_id = 1;
  future.continuous_state.resize(13);
  future.continuous_state[0] = 6.0;
  ego.trajectory_planned.push_back(future);

  const auto occupancy = egoRouteOccupancy(ego, rclcpp::Time(0, 0, RCL_ROS_TIME), 7.0, {priority}, conflict);
  ASSERT_TRUE(occupancy.has_value());
  EXPECT_NEAR(occupancy->entry, 1.0, 0.25);
  EXPECT_NEAR(occupancy->exit, 6.0, 0.25);
  EXPECT_TRUE(conflictsDuringClearance(TimeInterval{6.5, 7.0}, *occupancy, 1.0));
  EXPECT_FALSE(conflictsDuringClearance(TimeInterval{7.5, 8.0}, *occupancy, 1.0));
}

TEST(FollowingInteraction, MatchesOnlyTheSameDirectedLanelet) {
  const lanelet::routing::LaneletPath route({makeStraightLanelet(60)});
  EXPECT_NEAR(*projectLeaderOnRoute(route, 0.0, lanelet::BasicPoint2d(8.0, 0.0), 0.0, 60), 8.0, 1e-6);
  EXPECT_FALSE(projectLeaderOnRoute(route, 0.0, lanelet::BasicPoint2d(8.0, 4.0), 0.0, std::nullopt));
  EXPECT_FALSE(projectLeaderOnRoute(route, 0.0, lanelet::BasicPoint2d(8.0, 0.0), M_PI, std::nullopt));
  EXPECT_FALSE(projectLeaderOnRoute(route, 0.0, lanelet::BasicPoint2d(8.0, 0.0), 0.0, 61));
}

TEST(FollowingInteraction, BrakesContinuouslyBeforeAStationaryLeader) {
  RouteMotionSample follower{0.0, 4.0, 0.0};
  constexpr double leader_clearance = 12.0;
  for (int step_index = 1; step_index <= 100; ++step_index) {
    const double time = 0.1 * step_index;
    const double available = std::max(0.0, leader_clearance - follower.distance - follower.speed);
    const double braking_speed = std::sqrt(4.0 * available);
    const double position_speed = (leader_clearance - follower.distance - 0.05 * follower.speed) / 1.05;
    const auto step = advanceFollowingStep(follower, time, 100.0, std::min({4.0, braking_speed, position_speed}),
                                           leader_clearance, 1.0, 4.0, 1.0, 2.0, 100.0);
    EXPECT_TRUE(step.feasible);
    EXPECT_GE(step.sample.distance, follower.distance);
    EXPECT_GE(step.sample.speed, follower.speed - 0.2 - 1e-6);
    follower = step.sample;
  }
  EXPECT_LE(follower.distance + follower.speed, leader_clearance + 1e-3);
  EXPECT_LT(follower.speed, 0.1);
}

TEST(FollowingInteraction, AcceleratesAfterLeaderLeavesTheRoute) {
  RouteMotionSample follower{3.0, 0.0, 0.0};
  for (int step_index = 1; step_index <= 50; ++step_index) {
    const auto step = advanceFollowingStep(follower, 0.1 * step_index, 100.0, 4.0, std::numeric_limits<double>::infinity(), 1.0,
                                           4.0, 1.0, 2.0, 100.0);
    ASSERT_TRUE(step.feasible);
    EXPECT_LE(step.sample.speed - follower.speed, 0.1 + 1e-6);
    follower = step.sample;
  }
  EXPECT_NEAR(follower.speed, 4.0, 1e-6);
}

TEST(FollowingInteraction, MarksAnUnavoidableCloseLeaderInfeasibleWithoutJump) {
  const RouteMotionSample initial{0.0, 10.0, 0.0};
  const auto step = advanceFollowingStep(initial, 0.1, 1.0, 0.0, 2.0, 1.0, 10.0, 1.0, 2.0, 100.0);
  EXPECT_FALSE(step.feasible);
  EXPECT_NEAR(step.sample.speed, 9.8, 1e-6);
  EXPECT_GT(step.sample.distance, initial.distance);
}

TEST(YieldInteraction, AcceptsSmallTimestampOffsetsInEitherDirection) {
  EXPECT_TRUE(isEgoDataTimestampUsable(0.08, 1.0));
  EXPECT_TRUE(isEgoDataTimestampUsable(-0.08, 1.0));
  EXPECT_FALSE(isEgoDataTimestampUsable(1.01, 1.0));
  EXPECT_FALSE(isEgoDataTimestampUsable(-10.0, 1.0));
}

}  // namespace
}  // namespace lanelet2_object_list_prediction
