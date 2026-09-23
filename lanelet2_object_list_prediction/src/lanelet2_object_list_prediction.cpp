// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <vector>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
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

constexpr double kRouteProfileResolutionM = 0.25;
constexpr double kCurvatureSampleDistanceM = 1.0;
constexpr double kKinematicEpsilon = 1e-6;

struct RouteMotionSample {
  double distance{0.0};
  double speed{0.0};
  double time{0.0};
};

struct RouteMotionProfile {
  std::vector<RouteMotionSample> samples;
  bool lateral_limit_feasible{true};
};

double remainingRouteLength(const lanelet::routing::LaneletPath& route, double start_arc_length) {
  double length = 0.0;
  for (std::size_t route_index = 0; route_index < route.size(); ++route_index) {
    const double lanelet_length = static_cast<double>(lanelet::geometry::length(route[route_index].centerline2d()));
    length += route_index == 0 ? std::max(0.0, lanelet_length - start_arc_length) : lanelet_length;
  }
  return length;
}

lanelet::BasicPoint2d pointOnRoute(const lanelet::routing::LaneletPath& route, double start_arc_length, double travel_distance) {
  double distance_on_route = start_arc_length + std::max(0.0, travel_distance);
  for (std::size_t route_index = 0; route_index < route.size(); ++route_index) {
    const lanelet::ConstLineString2d centerline = route[route_index].centerline2d();
    const double lanelet_length = static_cast<double>(lanelet::geometry::length(centerline));
    if (distance_on_route > lanelet_length && route_index + 1 < route.size()) {
      distance_on_route -= lanelet_length;
      continue;
    }
    return lanelet::geometry::interpolatedPointAtDistance(centerline, std::clamp(distance_on_route, 0.0, lanelet_length));
  }

  const lanelet::ConstLineString2d centerline = route.back().centerline2d();
  const double centerline_length = static_cast<double>(lanelet::geometry::length(centerline));
  return lanelet::geometry::interpolatedPointAtDistance(centerline, centerline_length);
}

double routeCurvature(const lanelet::routing::LaneletPath& route,
                      double start_arc_length,
                      double route_length,
                      double travel_distance) {
  const double before_distance = std::max(0.0, travel_distance - kCurvatureSampleDistanceM);
  const double after_distance = std::min(route_length, travel_distance + kCurvatureSampleDistanceM);
  if (after_distance - before_distance < kKinematicEpsilon) return 0.0;

  const lanelet::BasicPoint2d before = pointOnRoute(route, start_arc_length, before_distance);
  const lanelet::BasicPoint2d center = pointOnRoute(route, start_arc_length, travel_distance);
  const lanelet::BasicPoint2d after = pointOnRoute(route, start_arc_length, after_distance);
  const double a = (center - before).norm();
  const double b = (after - center).norm();
  const double c = (after - before).norm();
  const double denominator = a * b * c;
  if (denominator < kKinematicEpsilon) return 0.0;

  const double cross =
      (center.x() - before.x()) * (after.y() - before.y()) - (center.y() - before.y()) * (after.x() - before.x());
  return 2.0 * std::abs(cross) / denominator;
}

RouteMotionProfile buildRouteMotionProfile(const lanelet::routing::LaneletPath& route,
                                           double start_arc_length,
                                           double initial_speed,
                                           double max_lateral_acceleration,
                                           double max_longitudinal_acceleration,
                                           double max_longitudinal_deceleration,
                                           bool stop_at_route_end) {
  const double route_length = remainingRouteLength(route, start_arc_length);
  const std::size_t segment_count =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(route_length / kRouteProfileResolutionM)));
  RouteMotionProfile profile;
  profile.samples.resize(segment_count + 1);
  std::vector<double> curve_speed_limits(segment_count + 1, initial_speed);

  for (std::size_t index = 0; index <= segment_count; ++index) {
    const double distance = route_length * static_cast<double>(index) / static_cast<double>(segment_count);
    profile.samples[index].distance = distance;
    const double curvature = routeCurvature(route, start_arc_length, route_length, distance);
    if (curvature > kKinematicEpsilon) {
      curve_speed_limits[index] = std::min(initial_speed, std::sqrt(max_lateral_acceleration / curvature));
    }
  }
  std::vector<double> speed_limits = curve_speed_limits;
  if (stop_at_route_end) speed_limits.back() = 0.0;

  // Propagate curve speed limits backwards so braking starts early enough.
  for (std::size_t index = segment_count; index > 0; --index) {
    const double distance = profile.samples[index].distance - profile.samples[index - 1].distance;
    const double reachable_speed =
        std::sqrt(speed_limits[index] * speed_limits[index] + 2.0 * max_longitudinal_deceleration * distance);
    speed_limits[index - 1] = std::min(speed_limits[index - 1], reachable_speed);
  }

  profile.samples.front().speed = initial_speed;
  if (initial_speed > curve_speed_limits.front() + kKinematicEpsilon) profile.lateral_limit_feasible = false;
  for (std::size_t index = 1; index <= segment_count; ++index) {
    const double distance = profile.samples[index].distance - profile.samples[index - 1].distance;
    const double previous_speed = profile.samples[index - 1].speed;
    const double minimum_reachable_speed =
        std::sqrt(std::max(0.0, previous_speed * previous_speed - 2.0 * max_longitudinal_deceleration * distance));
    const double maximum_reachable_speed =
        std::sqrt(previous_speed * previous_speed + 2.0 * max_longitudinal_acceleration * distance);
    const double desired_speed = std::min(initial_speed, speed_limits[index]);
    profile.samples[index].speed = std::clamp(desired_speed, minimum_reachable_speed, maximum_reachable_speed);
    if (profile.samples[index].speed > curve_speed_limits[index] + kKinematicEpsilon) {
      profile.lateral_limit_feasible = false;
    }

    const double average_speed = 0.5 * (previous_speed + profile.samples[index].speed);
    profile.samples[index].time = average_speed > kKinematicEpsilon ? profile.samples[index - 1].time + distance / average_speed
                                                                    : std::numeric_limits<double>::infinity();
  }
  return profile;
}

RouteMotionSample sampleRouteMotionAtTime(const std::vector<RouteMotionSample>& profile, double target_time) {
  const auto upper = std::lower_bound(profile.begin(), profile.end(), target_time,
                                      [](const RouteMotionSample& sample, double time) { return sample.time < time; });
  if (upper == profile.begin()) return *upper;
  if (upper == profile.end()) return profile.back();

  const RouteMotionSample& previous = *(upper - 1);
  const double segment_duration = upper->time - previous.time;
  if (!std::isfinite(segment_duration) || segment_duration < kKinematicEpsilon) return previous;

  const double elapsed = std::clamp(target_time - previous.time, 0.0, segment_duration);
  const double acceleration = (upper->speed - previous.speed) / segment_duration;
  RouteMotionSample sample;
  sample.time = target_time;
  sample.speed = std::max(0.0, previous.speed + acceleration * elapsed);
  sample.distance = previous.distance + previous.speed * elapsed + 0.5 * acceleration * elapsed * elapsed;
  return sample;
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
  this->declareAndLoadParameter("max_lateral_acceleration_mps2", max_lateral_acceleration_mps2_,
                                "Maximum lateral acceleration used to limit map-based prediction speed", true, false, false, 0.01,
                                20.0, 0.01);
  this->declareAndLoadParameter("max_longitudinal_deceleration_mps2", max_longitudinal_deceleration_mps2_,
                                "Maximum longitudinal deceleration magnitude used before curves", true, false, false, 0.01, 20.0,
                                0.01);
  this->declareAndLoadParameter("max_longitudinal_acceleration_mps2", max_longitudinal_acceleration_mps2_,
                                "Maximum longitudinal acceleration used to return to the observed speed after curves", true,
                                false, false, 0.01, 20.0, 0.01);
  this->declareAndLoadParameter("infeasible_hypothesis_probability", infeasible_hypothesis_probability_,
                                "Probability assigned to each laterally infeasible route when feasible alternatives exist", true,
                                false, false, 0.0, 1.0, 0.01);
  this->declareAndLoadParameter("unmatched_object_prediction_mode", unmatched_object_prediction_mode_,
                                "Prediction mode for objects that are not matched to the map", true, false, false, std::nullopt,
                                std::nullopt, std::nullopt, "Allowed values: static, kinematic");
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

    const auto classification = perception_msgs::object_access::getClassWithHighestProbability(prediction_object.object);
    // No lanelet matching for pedestrians, as they are not constrained to the road network
    if (classification.type == perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
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

      if (traffic_rules_ == nullptr || !traffic_rules_->canPass(matched_lanelet)) {
        continue;
      }

      prediction_object.lanelet_matches.push_back(
          LaneletMatch{matched_lanelet, candidate_lanelet.first, start_arc_length, orientation_difference});
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
  }

  const double probability_sum = std::accumulate(
      predictions.begin(), predictions.end(), 0.0,
      [](double sum, const perception_msgs::msg::ObjectStatePrediction& prediction) { return sum + prediction.probability; });
  if (probability_sum <= kKinematicEpsilon) {
    const double probability = 1.0 / static_cast<double>(predictions.size());
    for (perception_msgs::msg::ObjectStatePrediction& prediction : predictions) {
      prediction.probability = probability;
    }
  }
  return predictions;
}

std::vector<perception_msgs::msg::ObjectStatePrediction> Lanelet2ObjectListPrediction::createMapBasedPredictions(
    const PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const {
  std::vector<perception_msgs::msg::ObjectStatePrediction> predictions;
  if (routing_graph_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Routing graph is not available, cannot create lanelet predictions");
    return predictions;
  }

  double speed = 0.0;

  try {
    speed = perception_msgs::object_access::getVelocityMagnitude(prediction_object.object);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not read velocity for lanelet prediction: %s", ex.what());
    return predictions;
  }

  const std::size_t sample_count = getPredictionSampleCount();
  const double max_travel_distance = speed * prediction_sample_interval_s_ * static_cast<double>(sample_count);
  std::vector<bool> lateral_limit_feasibility;

  for (const LaneletMatch& match : prediction_object.lanelet_matches) {
    lanelet::routing::LaneletPaths routes;
    if (max_travel_distance <= std::numeric_limits<double>::epsilon()) {
      routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
    } else {
      lanelet::routing::PossiblePathsParams params;
      params.routingCostLimit = max_travel_distance + match.start_arc_length;
      params.includeShorterPaths = true;
      params.includeLaneChanges = false;
      try {
        routes = routing_graph_->possiblePaths(match.lanelet, params);
      } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not create lanelet routes from matched lanelet: %s", ex.what());
        continue;
      }
      if (routes.empty()) {
        routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
      }
    }

    for (const lanelet::routing::LaneletPath& lanelet_route : routes) {
      perception_msgs::msg::ObjectStatePrediction prediction;
      prediction.states.reserve(sample_count);
      const double route_length = remainingRouteLength(lanelet_route, match.start_arc_length);
      const bool stop_at_route_end = route_length + kKinematicEpsilon < max_travel_distance;
      const RouteMotionProfile motion_profile =
          buildRouteMotionProfile(lanelet_route, match.start_arc_length, speed, max_lateral_acceleration_mps2_,
                                  max_longitudinal_acceleration_mps2_, max_longitudinal_deceleration_mps2_, stop_at_route_end);
      for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
        const double sample_time = prediction_sample_interval_s_ * static_cast<double>(sample_index + 1);
        const RouteMotionSample motion = sampleRouteMotionAtTime(motion_profile.samples, sample_time);
        prediction.states.push_back(sampleStateOnLaneletRoute(prediction_object.object.state, lanelet_route,
                                                              match.start_arc_length, motion.distance, motion.speed, base_time,
                                                              sample_index));
      }
      predictions.push_back(prediction);
      lateral_limit_feasibility.push_back(motion_profile.lateral_limit_feasible);
    }
  }

  const std::size_t feasible_count =
      static_cast<std::size_t>(std::count(lateral_limit_feasibility.begin(), lateral_limit_feasibility.end(), true));
  const std::size_t infeasible_count = predictions.size() - feasible_count;
  if (feasible_count > 0) {
    const double infeasible_probability =
        infeasible_count > 0 ? std::min(infeasible_hypothesis_probability_, 1.0 / static_cast<double>(infeasible_count + 1))
                             : 0.0;
    const double feasible_probability =
        (1.0 - infeasible_probability * static_cast<double>(infeasible_count)) / static_cast<double>(feasible_count);
    for (std::size_t index = 0; index < predictions.size(); ++index) {
      predictions[index].probability = lateral_limit_feasibility[index] ? feasible_probability : infeasible_probability;
    }
  } else if (!predictions.empty()) {
    const double probability = 1.0 / static_cast<double>(predictions.size());
    for (auto& prediction : predictions) prediction.probability = probability;
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
  routing_graph_.reset();
  traffic_rules_.reset();
  routing_graph_map_ = ll2_interface_->getMapPtr();
  if (routing_graph_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map pointer is null, cannot build routing graph");
    return;
  }

  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(static_cast<const char*>(lanelet::Locations::Germany),
                                                                       static_cast<const char*>(lanelet::Participants::Vehicle));
  routing_graph_ = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *traffic_rules_);

  RCLCPP_INFO(this->get_logger(), "Built lanelet2 routing graph");
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
