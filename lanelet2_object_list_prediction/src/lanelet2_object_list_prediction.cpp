// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <tuple>
#include <utility>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
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

bool isBicycleLane(const lanelet::ConstLanelet& lanelet) {
  return lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{}) ==
         static_cast<const char*>(lanelet::AttributeValueString::BicycleLane);
}

// Paths against the legal direction, extending `path` of inverted lanelets along the legal predecessors.
lanelet::routing::LaneletPaths oppositeDirectionPaths(const lanelet::ConstLanelets& path,
                                                      double remaining_cost,
                                                      const lanelet::routing::RoutingGraph& routing_graph) {
  remaining_cost -= static_cast<double>(lanelet::geometry::length(path.back().centerline2d()));
  lanelet::routing::LaneletPaths paths;
  for (const lanelet::ConstLanelet& predecessor :
       remaining_cost > 0.0 ? routing_graph.previous(path.back().invert(), false) : lanelet::ConstLanelets{}) {
    lanelet::ConstLanelets extended = path;
    extended.push_back(predecessor.invert());
    const lanelet::routing::LaneletPaths extended_paths = oppositeDirectionPaths(extended, remaining_cost, routing_graph);
    paths.insert(paths.end(), extended_paths.begin(), extended_paths.end());
  }
  if (paths.empty()) paths.emplace_back(path);
  return paths;
}

constexpr double kHeadingChordLengthM = 3.0;  // long enough to smooth centerline jitter, short enough for sharp turns
constexpr int kHeadingMedianHalfWidth = 6;    // removes centerline steps shorter than about 6 m, keeping sustained turns
constexpr int kSimulationStepsPerSample = 10;

// Curvature every meter from `start_arc_length` along a line, from the change of its heading. A median over the
// headings removes short steps and kinks of the line, e.g. at crossings, while turns of any length are kept.
std::vector<double> curvaturesAlongLine(const lanelet::CompoundLineString2d& line, double start_arc_length, std::size_t count) {
  const double length = static_cast<double>(lanelet::geometry::length(line));
  const int half_width = kHeadingMedianHalfWidth;
  const auto point = [&](double distance) {
    return lanelet::geometry::interpolatedPointAtDistance(line, std::max(0.0, distance));
  };

  // Unwrapped heading of a chord every meter, starting one meter before the first curvature, straight beyond the ends
  std::vector<double> headings;
  for (int i = -1 - half_width; i <= static_cast<int>(count) + half_width; ++i) {
    const double center =
        std::min(std::max(start_arc_length + i, kHeadingChordLengthM / 2.0), length - kHeadingChordLengthM / 2.0);
    const lanelet::BasicPoint2d chord = point(center + kHeadingChordLengthM / 2.0) - point(center - kHeadingChordLengthM / 2.0);
    const double heading = std::atan2(chord.y(), chord.x());
    headings.push_back(headings.empty() ? heading : headings.back() + wrap_angle_rad(heading - headings.back()));
  }

  // Median of the headings around the distance k - 1 m
  const auto median_heading = [&](std::size_t k) {
    std::vector<double> window(headings.begin() + static_cast<std::ptrdiff_t>(k),
                               headings.begin() + static_cast<std::ptrdiff_t>(k) + 2 * half_width + 1);
    std::nth_element(window.begin(), window.begin() + half_width, window.end());
    return window[half_width];
  };
  std::vector<double> curvatures(count);
  for (std::size_t i = 0; i < count; ++i) curvatures[i] = std::abs(median_heading(i + 2) - median_heading(i)) / 2.0;
  return curvatures;
}

// Travel distance and speed at each sample time along a route. Starting at `speed`, the object brakes with at most
// `max_deceleration` early enough to pass curves with at most `max_lateral_acceleration`, and accelerates with at most
// `max_acceleration` back towards its initial speed after them. Also returns whether the initial speed allows braking
// early enough for all curves.
std::pair<bool, std::vector<std::pair<double, double>>> limitedMotionAlongRoute(const lanelet::routing::LaneletPath& route,
                                                                                double start_arc_length,
                                                                                double speed,
                                                                                double sample_interval,
                                                                                std::size_t sample_count,
                                                                                double max_lateral_acceleration,
                                                                                double max_acceleration,
                                                                                double max_deceleration) {
  // Speed limit every meter ahead: the curve speed, lowered so that the limits further ahead can be reached by braking
  const auto limit_count = static_cast<std::size_t>(speed * sample_interval * static_cast<double>(sample_count)) + 2;
  const std::vector<double> curvatures = curvaturesAlongLine(
      lanelet::LaneletSequence(lanelet::ConstLanelets(route.begin(), route.end())).centerline2d(), start_arc_length, limit_count);
  std::vector<double> speed_limits(limit_count, speed);
  for (std::size_t i = limit_count; i-- > 0;) {
    speed_limits[i] = std::min(speed, std::sqrt(max_lateral_acceleration / curvatures[i]));
    if (i + 1 < limit_count) {
      speed_limits[i] = std::min(speed_limits[i], std::sqrt(std::pow(speed_limits[i + 1], 2) + 2.0 * max_deceleration));
    }
  }

  const bool feasible = speed <= speed_limits.front();

  // Simulate in small time steps, braking or accelerating towards the next speed limit
  const double dt = sample_interval / kSimulationStepsPerSample;
  double distance = 0.0;
  std::vector<std::pair<double, double>> motion;
  for (std::size_t step = 1; step <= sample_count * kSimulationStepsPerSample; ++step) {
    const auto next_limit = std::min(static_cast<std::size_t>(std::ceil(distance)), limit_count - 1);
    speed = std::clamp(speed_limits[next_limit], speed - max_deceleration * dt, speed + max_acceleration * dt);
    distance += speed * dt;
    if (step % kSimulationStepsPerSample == 0) motion.emplace_back(distance, speed);
  }
  return {feasible, motion};
}

}  // namespace

Lanelet2ObjectListPrediction::Lanelet2ObjectListPrediction() : Node("lanelet2_object_list_prediction") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false, false,
                                true);
  this->declareAndLoadParameter("lanelet_match_max_distance_m", lanelet_match_max_distance_m_,
                                "Maximum distance in meters for matching an object to a lanelet", true, false, false, 0.0, 100.0,
                                0.1);
  this->declareAndLoadParameter("lanelet_match_max_yaw_diff_rad", lanelet_match_max_yaw_diff_rad_,
                                "Maximum yaw difference in radians for accepting a lanelet match", true, false, false, 0.0,
                                3.14159265359);
  this->declareAndLoadParameter("prediction_horizon_s", prediction_horizon_s_, "Prediction horizon in seconds", true, false,
                                false, 0.1, 60.0, 0.1);
  this->declareAndLoadParameter("prediction_sample_interval_s", prediction_sample_interval_s_,
                                "Sampling interval of predicted states in seconds", true, false, false, 0.01, 10.0, 0.01);
  this->declareAndLoadParameter("unmatched_object_prediction_mode", unmatched_object_prediction_mode_,
                                "Prediction mode for objects that are not matched to the map", true, false, false, std::nullopt,
                                std::nullopt, std::nullopt, "Allowed values: static, kinematic");
  this->declareAndLoadParameter("infeasible_prediction_probability", infeasible_prediction_probability_,
                                "Probability of each prediction that cannot be followed within the motion limits", true, false,
                                false, 0.0, 1.0, 0.01);
  this->declareAndLoadParameter("participant_specific_matching.enable", participant_specific_matching_enable_,
                                "Match and route pedestrians and two-wheelers with their own traffic rules, preferring "
                                "bicycle lanes for two-wheelers");
  this->declareAndLoadParameter("participant_specific_matching.allow_opposite_direction",
                                participant_specific_matching_allow_opposite_direction_,
                                "Predict two-wheelers without a legal lanelet match against a lanelet's direction");
  this->declareAndLoadParameter(
      "motion_limits.enable", motion_limits_enable_,
      "Reduce the predicted speed in curves to respect the lateral and longitudinal acceleration limits");
  this->declareAndLoadParameter("motion_limits.max_lateral_acceleration_mps2", motion_limits_max_lateral_acceleration_mps2_,
                                "Maximum lateral acceleration in m/s^2 of predicted objects in curves", true, false, false, 0.1,
                                20.0, 0.1);
  this->declareAndLoadParameter(
      "motion_limits.max_longitudinal_acceleration_mps2", motion_limits_max_longitudinal_acceleration_mps2_,
      "Maximum longitudinal acceleration in m/s^2 of predicted objects regaining their current speed after slowing down, "
      "e.g. after a curve; predictions never exceed the current speed",
      true, false, false, 0.0, 20.0, 0.1);
  this->declareAndLoadParameter(
      "motion_limits.max_longitudinal_deceleration_mps2", motion_limits_max_longitudinal_deceleration_mps2_,
      "Maximum longitudinal deceleration in m/s^2 of predicted objects slowing down, e.g. before a curve", true, false, false,
      0.1, 20.0, 0.1);
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
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
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
  ll2_interface_ = std::make_unique<Lanelet2MapInterface>(*this, ll2_map_server_name_);

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&Lanelet2ObjectListPrediction::parametersCallback, this, std::placeholders::_1));

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      "~/tracked_object_list", 1, std::bind(&Lanelet2ObjectListPrediction::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<perception_msgs::msg::ObjectList>("~/object_list", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
}

void Lanelet2ObjectListPrediction::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr& msg) {
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
      RCLCPP_ERROR(this->get_logger(), "Could not transform object list from frame '%s' to frame '%s': %s. Skipping object list.",
                   msg->header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
      return;
    }
  } else {
    object_list_map_frame = *msg;
  }

  std::vector<PredictionObject> prediction_objects = matchObjectListToMap(object_list_map_frame);
  std::size_t matched_object_count = 0;
  for (PredictionObject& prediction_object : prediction_objects) {
    if (!prediction_object.lanelet_matches.empty()) {
      ++matched_object_count;
    }
    prediction_object.object.state_predictions =
        createPredictionsForMatchedObject(prediction_object, object_list_map_frame.header.stamp);
  }
  RCLCPP_DEBUG(this->get_logger(), "Matched %zu/%zu objects to at least one lanelet", matched_object_count,
               object_list_map_frame.objects.size());

  perception_msgs::msg::ObjectList out_msg = object_list_map_frame;
  out_msg.objects.clear();
  out_msg.objects.reserve(prediction_objects.size());
  for (const PredictionObject& prediction_object : prediction_objects) {
    out_msg.objects.push_back(prediction_object.object);
  }

  publisher_->publish(out_msg);
  RCLCPP_DEBUG(this->get_logger(), "Message published with stamp: '%d'", out_msg.header.stamp.sec);
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

    using perception_msgs::msg::ObjectClassification;
    const uint8_t type = perception_msgs::object_access::getClassWithHighestProbability(prediction_object.object).type;
    const bool participant_specific = participant_specific_matching_enable_;
    const bool pedestrian =
        participant_specific && (type == ObjectClassification::PEDESTRIAN || type == ObjectClassification::VRU);
    const bool two_wheeler =
        participant_specific && (type == ObjectClassification::BICYCLE || type == ObjectClassification::MICRO ||
                                 type == ObjectClassification::MOTORCYCLE);
    // Without participant-specific matching, pedestrians are not matched, as they are not constrained to the road network
    if (!participant_specific && type == ObjectClassification::PEDESTRIAN) {
      prediction_objects.push_back(prediction_object);
      continue;
    }

    geometry_msgs::msg::Point position;
    double object_yaw = 0.0;
    try {
      position = perception_msgs::object_access::getPosition(prediction_object.object);
      object_yaw = perception_msgs::object_access::getYaw(prediction_object.object);
    } catch (const std::exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not read position or yaw of object %zu: %s", object_index, ex.what());
      prediction_objects.push_back(prediction_object);
      continue;
    }

    const lanelet::BasicPoint2d position_2d(position.x, position.y);
    const auto candidate_lanelets =
        lanelet::geometry::findWithin2d(map->laneletLayer, position_2d, lanelet_match_max_distance_m_);

    for (const auto& candidate_lanelet : candidate_lanelets) {
      lanelet::ConstLanelet lanelet = candidate_lanelet.second;
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

      if (orientation_difference > lanelet_match_max_yaw_diff_rad_) {
        continue;
      }

      // Pedestrians and bicycles follow their own traffic rules, motorcycles only on bicycle lanes
      const ParticipantRouting* routing = &vehicle_routing_;
      if (pedestrian) {
        routing = &pedestrian_routing_;
      } else if (two_wheeler && (type != ObjectClassification::MOTORCYCLE || isBicycleLane(matched_lanelet))) {
        routing = &bicycle_routing_;
      }
      if (routing->traffic_rules == nullptr) {
        continue;
      }
      const bool opposite_direction = !routing->traffic_rules->canPass(matched_lanelet);
      if (opposite_direction && !(two_wheeler && participant_specific_matching_allow_opposite_direction_ &&
                                  routing->traffic_rules->canPass(matched_lanelet.invert()))) {
        continue;
      }

      prediction_object.lanelet_matches.push_back(LaneletMatch{matched_lanelet, candidate_lanelet.first, start_arc_length,
                                                               orientation_difference, routing->routing_graph.get(),
                                                               opposite_direction});
    }

    // Two-wheelers prefer legal matches over matches against a lanelet's direction, then bicycle lanes over roads
    auto& matches = prediction_object.lanelet_matches;
    const auto rank = [](const LaneletMatch& match) {
      return (match.opposite_direction ? 2 : 0) + (isBicycleLane(match.lanelet) ? 0 : 1);
    };
    if (two_wheeler && !matches.empty()) {
      const int best_rank = rank(*std::min_element(
          matches.begin(), matches.end(), [&](const LaneletMatch& a, const LaneletMatch& b) { return rank(a) < rank(b); }));
      matches.erase(std::remove_if(matches.begin(), matches.end(), [&](const auto& match) { return rank(match) > best_rank; }),
                    matches.end());
    }

    if (prediction_object.lanelet_matches.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Object %zu did not match any lanelet within %.2f m", object_index,
                   lanelet_match_max_distance_m_);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Object %zu matched to %zu lanelet candidate(s)", object_index,
                   prediction_object.lanelet_matches.size());
    }
    prediction_objects.push_back(prediction_object);
  }

  return prediction_objects;
}

std::vector<perception_msgs::msg::ObjectStatePrediction> Lanelet2ObjectListPrediction::createPredictionsForMatchedObject(
    const PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const {
  std::vector<perception_msgs::msg::ObjectStatePrediction> predictions;
  if (!prediction_object.lanelet_matches.empty()) {
    predictions = createMapBasedPredictions(prediction_object, base_time);
  }

  if (predictions.empty()) {
    if (unmatched_object_prediction_mode_ == "static") {
      predictions.push_back(createStationaryPrediction(prediction_object.object, base_time));
    } else {
      predictions.push_back(createConstantVelocityPrediction(prediction_object.object, base_time));
    }
    predictions.back().probability = 1.0;
  }
  return predictions;
}

std::vector<perception_msgs::msg::ObjectStatePrediction> Lanelet2ObjectListPrediction::createMapBasedPredictions(
    const PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const {
  std::vector<perception_msgs::msg::ObjectStatePrediction> predictions;

  double speed = 0.0;

  try {
    speed = perception_msgs::object_access::getVelocityMagnitude(prediction_object.object);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not read velocity for lanelet prediction: %s", ex.what());
    return predictions;
  }

  const std::size_t sample_count = getPredictionSampleCount();
  const double max_travel_distance = speed * prediction_sample_interval_s_ * static_cast<double>(sample_count);

  for (const LaneletMatch& match : prediction_object.lanelet_matches) {
    lanelet::routing::LaneletPaths routes;
    if (max_travel_distance <= std::numeric_limits<double>::epsilon()) {
      routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
    } else if (match.opposite_direction) {
      routes = oppositeDirectionPaths({match.lanelet}, max_travel_distance + match.start_arc_length, *match.routing_graph);
    } else {
      lanelet::routing::PossiblePathsParams params;
      params.routingCostLimit = max_travel_distance + match.start_arc_length;
      params.includeShorterPaths = true;
      params.includeLaneChanges = false;
      try {
        routes = match.routing_graph->possiblePaths(match.lanelet, params);
      } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not create lanelet routes from matched lanelet: %s", ex.what());
        continue;
      }
      if (routes.empty()) {
        routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
      }
    }

    for (const lanelet::routing::LaneletPath& lanelet_route : routes) {
      // Travel distance and speed at each sample
      bool feasible = true;
      std::vector<std::pair<double, double>> motion;
      if (motion_limits_enable_ && max_travel_distance > std::numeric_limits<double>::epsilon()) {
        std::tie(feasible, motion) = limitedMotionAlongRoute(
            lanelet_route, match.start_arc_length, speed, prediction_sample_interval_s_, sample_count,
            motion_limits_max_lateral_acceleration_mps2_, motion_limits_max_longitudinal_acceleration_mps2_,
            motion_limits_max_longitudinal_deceleration_mps2_);
      } else {
        for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
          motion.emplace_back(speed * prediction_sample_interval_s_ * static_cast<double>(sample_index + 1), speed);
        }
      }

      perception_msgs::msg::ObjectStatePrediction prediction;
      prediction.states.reserve(sample_count);
      for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
        prediction.states.push_back(sampleStateOnLaneletRoute(prediction_object.object.state, lanelet_route,
                                                              match.start_arc_length, motion[sample_index].first,
                                                              motion[sample_index].second, base_time, sample_index));
      }
      prediction.probability = feasible ? 1.0 : 0.0;  // marks feasibility until the probabilities are assigned below
      predictions.push_back(prediction);
    }
  }

  // Infeasible predictions get the configured probability, feasible ones share the rest equally. Without any feasible
  // prediction, the object gets the fallback prediction instead.
  const auto count = static_cast<double>(predictions.size());
  const auto feasible_count = static_cast<double>(
      std::count_if(predictions.begin(), predictions.end(), [](const auto& prediction) { return prediction.probability > 0.0; }));
  if (feasible_count == 0.0) return {};
  const double infeasible_probability = std::min(infeasible_prediction_probability_, 1.0 / count);
  const double feasible_probability = (1.0 - (count - feasible_count) * infeasible_probability) / feasible_count;
  for (perception_msgs::msg::ObjectStatePrediction& prediction : predictions) {
    prediction.probability = prediction.probability > 0.0 ? feasible_probability : infeasible_probability;
  }
  return predictions;
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
    RCLCPP_WARN(this->get_logger(), "Could not read velocity for kinematic prediction, using static fallback: %s", ex.what());
    return createStationaryPrediction(object, base_time);
  }

  for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double time_offset = prediction_sample_interval_s_ * static_cast<double>(sample_index + 1);
    perception_msgs::msg::ObjectState state = object.state;
    setPredictedStateKinematics(state, position.x + velocity.x * time_offset, position.y + velocity.y * time_offset,
                                position.z + velocity.z * time_offset, yaw, velocity, base_time, sample_index);
    prediction.states.push_back(state);
  }
  return prediction;
}

perception_msgs::msg::ObjectState Lanelet2ObjectListPrediction::sampleStateOnLaneletRoute(
    const perception_msgs::msg::ObjectState& base_state,
    const lanelet::routing::LaneletPath& route,
    double start_arc_length,
    double travel_distance,
    double speed,
    const builtin_interfaces::msg::Time& base_time,
    std::size_t sample_index) const {
  perception_msgs::msg::ObjectState state = base_state;
  if (route.empty()) {
    RCLCPP_WARN(this->get_logger(), "Lanelet route is empty, cannot sample lanelet prediction");
    return state;
  }

  double distance_on_route = start_arc_length + travel_distance;
  geometry_msgs::msg::Point fallback_position;
  try {
    fallback_position = perception_msgs::object_access::getPosition(base_state);
  } catch (const std::exception&) {
    fallback_position.x = 0.0;
    fallback_position.y = 0.0;
    fallback_position.z = 0.0;
  }

  for (std::size_t route_index = 0; route_index < route.size(); ++route_index) {
    const lanelet::ConstLineString2d centerline = route[route_index].centerline2d();
    if (centerline.size() < 2) {
      continue;
    }

    const double lanelet_length = static_cast<double>(lanelet::geometry::length(centerline));
    const bool is_last_lanelet = route_index + 1 == route.size();
    if (distance_on_route > lanelet_length && !is_last_lanelet) {
      distance_on_route -= lanelet_length;
      continue;
    }

    const double arc_length = std::clamp(distance_on_route, 0.0, lanelet_length);
    const lanelet::BasicPoint2d point = lanelet::geometry::interpolatedPointAtDistance(centerline, arc_length);
    const double yaw_sample_distance = std::min(0.5, std::max(0.01, lanelet_length * 0.1));
    double before_arc_length = std::max(0.0, arc_length - yaw_sample_distance);
    double after_arc_length = std::min(lanelet_length, arc_length + yaw_sample_distance);
    if (after_arc_length <= before_arc_length) {
      before_arc_length = 0.0;
      after_arc_length = lanelet_length;
    }

    const lanelet::BasicPoint2d before_point = lanelet::geometry::interpolatedPointAtDistance(centerline, before_arc_length);
    const lanelet::BasicPoint2d after_point = lanelet::geometry::interpolatedPointAtDistance(centerline, after_arc_length);
    const double yaw = std::atan2(after_point.y() - before_point.y(), after_point.x() - before_point.x());
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = speed * std::cos(yaw);
    velocity.y = speed * std::sin(yaw);
    velocity.z = 0.0;
    setPredictedStateKinematics(state, point.x(), point.y(), fallback_position.z, yaw, velocity, base_time, sample_index);
    return state;
  }

  const lanelet::ConstLineString2d last_centerline = route.back().centerline2d();
  const double last_length = static_cast<double>(lanelet::geometry::length(last_centerline));
  const lanelet::BasicPoint2d point = lanelet::geometry::interpolatedPointAtDistance(last_centerline, last_length);
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = 0.0;
  velocity.y = 0.0;
  velocity.z = 0.0;
  setPredictedStateKinematics(state, point.x(), point.y(), fallback_position.z, 0.0, velocity, base_time, sample_index);
  return state;
}

std::size_t Lanelet2ObjectListPrediction::getPredictionSampleCount() const {
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(prediction_horizon_s_ / prediction_sample_interval_s_)));
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
  const double time_offset = prediction_sample_interval_s_ * static_cast<double>(sample_index + 1);
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
  vehicle_routing_ = {};
  bicycle_routing_ = {};
  pedestrian_routing_ = {};
  routing_graph_map_ = ll2_interface_->getMapPtr();
  if (routing_graph_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map pointer is null, cannot build routing graph");
    return;
  }

  for (auto [routing, participant] : {std::pair{&vehicle_routing_, lanelet::Participants::Vehicle},
                                      std::pair{&bicycle_routing_, lanelet::Participants::Bicycle},
                                      std::pair{&pedestrian_routing_, lanelet::Participants::Pedestrian}}) {
    routing->traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(participant));
    routing->routing_graph = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *routing->traffic_rules);
  }

  RCLCPP_INFO(this->get_logger(), "Built lanelet2 routing graphs");
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
