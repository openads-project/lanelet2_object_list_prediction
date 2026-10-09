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

The predicted speed along each path is limited in curves so that the lateral acceleration stays below `max_lateral_acceleration_mps2`. The curvature comes from the centerline heading, median-filtered to remove kinks of the map centerlines, e.g. at crossings.

Before a curve, the object brakes with at most `max_longitudinal_deceleration_mps2`. After it, the object regains its current speed with at most `max_longitudinal_acceleration_mps2`. Predictions never get faster than the object is now. Both longitudinal limits are top-level parameters, as yielding uses them as well.

A path is infeasible if the object is too fast to brake down for a curve ahead. Each infeasible path gets the probability `infeasible_prediction_probability`, and the feasible paths share the rest. With a probability of 0.0, infeasible paths are not published. If no path is feasible, `unmatched_object_prediction_mode` is used instead.

With `enable: false`, objects keep their current speed along each path.

## Yielding

Routes are created for all objects first. Yielding is then evaluated against these nominal routes of the whole scene, so the result does not depend on the order of the objects.

A route yields to a Lanelet2 `RightOfWay` rule whose yield lanelet is on the route or just before it. A rule is usually set on the last lanelet before a junction, while its stop line can lie on a following lanelet. The stop line is therefore projected onto the whole route. Without a stop line, the object stops at the end of the yield lanelet. The object stops with its front at that point.

Priority lanelets are the rule's right-of-way lanelets and the lanelets directly following them. The conflict area is made up of the route's lanelets beyond the stop that are not priority lanelets. The object yields if another object's route, or ego's planned route from `~/route`, passes a priority lanelet and then overlaps the conflict area. Conflicts are checked in space only, so the object stays stopped for the rest of the prediction horizon. Ego's route is used rather than its planned trajectory, as the trajectory reacts to the predictions: when ego plans to brake for an object, its trajectory may end before the junction, and the object would no longer have to yield. The route is used along its suggested lanes from ego's current position, in the map frame.

The object brakes with at most `max_longitudinal_deceleration_mps2`. If it cannot stop in time, the path is infeasible, see [Motion limits](#motion-limits).

With `enable: false`, objects do not yield.
