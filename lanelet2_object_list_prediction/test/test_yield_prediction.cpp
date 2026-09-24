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

TEST(YieldInteraction, DetectsOnlyOverlappingOccupancyIntervals) {
  EXPECT_TRUE(intervalsOverlap(TimeInterval{1.0, 3.0}, TimeInterval{2.0, 4.0}));
  EXPECT_FALSE(intervalsOverlap(TimeInterval{1.0, 2.0}, TimeInterval{2.1, 4.0}));
}

TEST(YieldInteraction, AcceptsSmallTimestampOffsetsInEitherDirection) {
  EXPECT_TRUE(isEgoDataTimestampUsable(0.08, 1.0));
  EXPECT_TRUE(isEgoDataTimestampUsable(-0.08, 1.0));
  EXPECT_FALSE(isEgoDataTimestampUsable(1.01, 1.0));
  EXPECT_FALSE(isEgoDataTimestampUsable(-10.0, 1.0));
}

}  // namespace
}  // namespace lanelet2_object_list_prediction
