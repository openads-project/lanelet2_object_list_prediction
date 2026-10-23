# Implementation

This document describes how the features of `lanelet2_object_list_prediction` work. For an overview of the node, its topics and its parameters, see the [README](README.md).

## Participant-specific matching

The object's most likely classification selects the Lanelet2 traffic rules used for matching and routing. A separate routing graph is built for each set of rules.

| Classification | Traffic rules |
| --- | --- |
| `PEDESTRIAN`, `VRU` | Pedestrian |
| `BICYCLE`, `MICRO` | Bicycle |
| `MOTORCYCLE` | Bicycle on bicycle lanes, vehicle elsewhere |
| all others | Vehicle |

If a two-wheeler matches several lanelets, legal matches beat matches against the lanelet's direction, and bicycle lanes beat other lanelets. Only the best-ranked matches are kept.

Routing graphs only contain legal directions. With `allow_opposite_direction`, routes against a lanelet's direction are therefore built backwards along its legal predecessors.

With `enable: false`, `PEDESTRIAN` objects are not matched, and all other objects use the vehicle rules.

## Motion limits

The predicted speed along each path is limited in curves so that the lateral acceleration stays below `max_lateral_acceleration_mps2`. The curvature comes from the centerline heading, median-filtered to remove kinks of the map centerlines, e.g. at crossings:

1. The raw heading at every meter is the direction of a configurable (kHeadingChordLengthM) chord around it.
2. The median over the raw headings within ± kHeadingMedianHalfWidth m ignores short steps and kinks, which only tilt a few of them. In a turn, the headings change steadily, so the median keeps it.
3. The curvature at every meter is the change of the median heading per meter, from 1 m before to 1 m after it.

For stronger smoothing, increase `kHeadingMedianHalfWidth` to also ignore longer steps and kinks, at the cost of flattening S-curves shorter than the window, or increase `kHeadingChordLengthM` to average out more centerline jitter, at the cost of spreading sharp turns over the chord length and lowering their peak curvature.

<p align="center">
  <img src="../assets/curvature_median_filter.svg" alt="Curvature from the median-filtered centerline heading" width="800"/>
</p>

Before a curve, the object brakes with at most `max_longitudinal_deceleration_mps2`. After it, the object regains its current speed with at most `max_longitudinal_acceleration_mps2`. Predictions never get faster than the object is now. Both longitudinal limits are top-level parameters, as yielding uses them as well.

A path is infeasible if the object is too fast to brake down for a curve ahead. Each infeasible path gets the probability `infeasible_prediction_probability`, and the feasible paths share the rest. With a probability of 0.0, infeasible paths are not published. If no path is feasible, `unmatched_object_prediction_mode` is used instead.

With `enable: false`, objects keep their current speed along each path.

## Yielding

Routes are created for all objects first. Yielding is then evaluated against these nominal routes of the whole scene, so the result does not depend on the order of the objects.

A route yields to a Lanelet2 `RightOfWay` rule whose yield lanelet is on the route or just before it. A rule is usually set on the last lanelet before a junction, while its stop line can lie on a following lanelet. The stop line is therefore projected onto the whole route. Without a stop line, the object stops at the end of the yield lanelet. The object stops with its front at that point.

Priority lanelets are the rule's right-of-way lanelets and the lanelets directly following them. The conflict area is made up of the route's lanelets beyond the stop that are not priority lanelets. The object yields if another object's route, or ego's planned route from `~/route`, passes a priority lanelet and then overlaps the conflict area. Conflicts are checked in space only, so the object stays stopped for the rest of the prediction horizon. Ego's route is used rather than its planned trajectory, as the trajectory reacts to the predictions: when ego plans to brake for an object, its trajectory may end before the junction, and the object would no longer have to yield. The route is used along its suggested lanes from ego's current position, in the map frame.

The object brakes with at most `max_longitudinal_deceleration_mps2`. If it cannot stop in time, the path is infeasible, see [Motion limits](#motion-limits).

With `enable: false`, objects do not yield.

## Following

Following is evaluated after yielding, in a single pass. All objects first get their nominal predictions, including yielding. Each object then slows down behind the nominal predictions of the other objects and behind ego's planned trajectory from `~/ego_data`. As followers react to nominal predictions only, slowing down does not propagate to further followers.

Each prediction of another object is a possible leader, and so is ego. Ego's planned trajectory is used rather than its route, as following needs ego's motion over time. At each prediction time, ego is at the first trajectory point at or after it, while the trajectory covers it, in the map frame. A leader is on a route while it is inside one of the route's lanelets and its yaw differs from the lanelet's by at most `lanelet_match_max_yaw_diff_rad`, as for matching objects to lanelets, so crossing and oncoming traffic is not followed. It must start ahead of the follower's front, so traffic behind or beside the follower is not followed either. Per route, the closest leader counts.

Behind a leader, the speed is limited so that braking with at most `max_longitudinal_deceleration_mps2` reaches the leader's speed `min_distance_m` behind its rear, assuming the leader keeps its speed. When the leader leaves the route or speeds up, the follower regains its current speed with at most `max_longitudinal_acceleration_mps2`. Without a leader on a route, the prediction does not change.

A path is infeasible if its object runs into a leader despite braking, e.g. when approaching too fast. Getting closer than `min_distance_m`, e.g. when a leader cuts in, keeps the path feasible, as the object still brakes behind the leader. As for curves, see [Motion limits](#motion-limits), an infeasible path gets `infeasible_prediction_probability`, and without any feasible path the object gets the fallback prediction.

With `enable: false`, objects do not follow.

## Lane coverage matching

Instead of all lanelets within `lanelet_match_max_distance_m`, an object is matched to its nearest lanelets that pass the direction and traffic rule checks. Usually this is the lanelet its position lies on, and several where lanelets overlap, e.g. in junctions.

A nearest lanelet only counts if at least `min_coverage` of the object's footprint lies on it, together with the lanelets connected to it ahead and behind that also lie under the footprint. A vehicle across the joint of two consecutive lanelets therefore counts fully, while a vehicle across the boundary of two neighboring lanes does not. The footprint is the object's box along the tracker's yaw. Objects without a footprint are matched to their nearest lanelets within `lanelet_match_max_distance_m`.

With `enable: false`, objects are matched to all lanelets within `lanelet_match_max_distance_m`.

## Lane offset

With `keep_lane_offset`, predictions keep the object's lateral position in its lane instead of following the centerline. The position is relative to the distance from the centerline to the lane bound on the object's side: 0 on the centerline and ±1 on the left or right bound, limited to the lane. Along the route, every predicted state lies at the same relative position, so it follows changes of the lane width. The position is measured again with every object list, so a prediction returns to the centerline as soon as the object does.

Speeds, curves, yield lines and leaders are still evaluated along the centerline.

With `keep_lane_offset: false`, predictions follow the centerline.
