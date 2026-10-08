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
