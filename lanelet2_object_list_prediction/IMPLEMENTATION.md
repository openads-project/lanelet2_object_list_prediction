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

Before a curve, the object brakes with at most `max_longitudinal_deceleration_mps2`. After it, the object regains its current speed with at most `max_longitudinal_acceleration_mps2`. Predictions never get faster than the object is now.

A path is infeasible if the object is too fast to brake down for a curve ahead. Each infeasible path gets the probability `infeasible_prediction_probability`, and the feasible paths share the rest. With a probability of 0.0, infeasible paths are not published. If no path is feasible, `unmatched_object_prediction_mode` is used instead.

With `enable: false`, objects keep their current speed along each path.
