// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

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

Lanelet2ObjectListPrediction::Lanelet2ObjectListPrediction() : Node("lanelet2_object_list_prediction") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false, false,
                                true);
  this->declareAndLoadParameter("lanelet_match_max_distance_m", lanelet_match_max_distance_m_,
                                "Maximum distance in meters for matching an object to a lanelet", true, false, false, 0.0, 100.0,
                                0.1);
  this->declareAndLoadParameter("lanelet_match_max_yaw_diff_rad", lanelet_match_max_yaw_diff_rad_,
                                "Maximum yaw difference in radians for accepting a lanelet match", true, false, false, 0.0,
                                3.14159265359, std::nullopt);
  this->declareAndLoadParameter("prediction_horizon_s", prediction_horizon_s_, "Prediction horizon in seconds", true, false,
                                false, 0.1, 60.0, 0.1);
  this->declareAndLoadParameter("prediction_sample_interval_s", prediction_sample_interval_s_,
                                "Sampling interval of predicted states in seconds", true, false, false, 0.01, 10.0, 0.01);
  this->declareAndLoadParameter("unmatched_object_prediction_mode", unmatched_object_prediction_mode_,
                                "Prediction mode for objects that are not matched to the map", true, false, false, std::nullopt,
                                std::nullopt, std::nullopt, "Allowed values: static, kinematic");
  this->declareAndLoadParameter("velocity_ema_alpha", velocity_ema_alpha_,
                                "EMA smoothing factor for velocity updates (0=frozen, 1=raw)", true, false, false, 0.0, 1.0);
  this->declareAndLoadParameter("velocity_hold_time_s", velocity_hold_time_s_,
                                "Seconds to hold the last velocity estimate before decay begins", true, false, false, 0.0, 10.0,
                                0.1);
  this->declareAndLoadParameter("velocity_decay_time_constant_s", velocity_decay_time_constant_s_,
                                "Exponential decay time constant in seconds after the hold window", true, false, false, 0.1, 60.0,
                                0.1);
  this->declareAndLoadParameter("arc_length_ema_alpha", arc_length_ema_alpha_,
                                "EMA smoothing factor for arc-length projection (0=frozen, 1=raw)", true, false, false, 0.0, 1.0);
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
      "~/object_list", 1, std::bind(&Lanelet2ObjectListPrediction::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<perception_msgs::msg::ObjectList>("~/predicted_object_list", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
}

void Lanelet2ObjectListPrediction::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr& msg) {
  RCLCPP_INFO(this->get_logger(), "Message received with stamp: '%d'", msg->header.stamp.sec);

  if (!checkMap(true)) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map is not loaded yet, skipping object list");
    return;
  }

  perception_msgs::msg::ObjectList object_list_map_frame;
  if (msg->header.frame_id != ll2_interface_->map_frame_id_) {
    try {
      auto transform = tf_buffer_->lookupTransform(ll2_interface_->map_frame_id_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, object_list_map_frame, transform);
      object_list_map_frame.header.stamp = msg->header.stamp;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform object list from frame '%s' to frame '%s': %s. Skipping object list.",
                   msg->header.frame_id.c_str(), ll2_interface_->map_frame_id_.c_str(), ex.what());
      return;
    }
  } else {
    object_list_map_frame = *msg;
  }

  const rclcpp::Time current_stamp(object_list_map_frame.header.stamp);
  estimateVelocities(object_list_map_frame, current_stamp);
  last_message_stamp_ = current_stamp;

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
  RCLCPP_INFO(this->get_logger(), "Message published with stamp: '%d'", out_msg.header.stamp.sec);
}

void Lanelet2ObjectListPrediction::estimateVelocities(perception_msgs::msg::ObjectList& object_list,
                                                      const rclcpp::Time& current_stamp) {
  if (current_stamp < last_message_stamp_) {
    RCLCPP_WARN(this->get_logger(), "Time jumped backward, clearing position history");
    position_history_.clear();
    match_state_.clear();
    smoothed_velocity_.clear();
  }

  // Time elapsed since the previous message, used for time-proportional velocity decay.
  const double dt_frame =
      (last_message_stamp_.nanoseconds() > 0) ? std::max(0.0, (current_stamp - last_message_stamp_).seconds()) : 0.0;

  for (auto& object : object_list.objects) {
    geometry_msgs::msg::Point position = perception_msgs::object_access::getPosition(object.state);

    // Step 1: Obtain the best available raw velocity
    geometry_msgs::msg::Vector3 raw_velocity;
    bool have_raw_velocity = false;

    try {
      const double sensor_speed = perception_msgs::object_access::getVelocityMagnitude(object.state);
      if (sensor_speed > std::numeric_limits<double>::epsilon()) {
        raw_velocity = perception_msgs::object_access::getVelocityXYZ(object.state);
        have_raw_velocity = true;
        position_history_[object.id] = {position, current_stamp};
      }
    } catch (const std::exception&) {
      // velocity field not set -> fall through to position-delta estimation
    }

    if (!have_raw_velocity) {
      auto it = position_history_.find(object.id);
      if (it != position_history_.end() && current_stamp > it->second.stamp) {
        const double dt = (current_stamp - it->second.stamp).seconds();
        // Require a minimum interval to avoid extremely noisy estimates from near-simultaneous messages.
        constexpr double kMinDt_s = 0.05;
        if (dt >= kMinDt_s) {
          raw_velocity.x = (position.x - it->second.position.x) / dt;
          raw_velocity.y = (position.y - it->second.position.y) / dt;
          raw_velocity.z = (position.z - it->second.position.z) / dt;
          if (std::hypot(raw_velocity.x, raw_velocity.y) > std::numeric_limits<double>::epsilon()) {
            have_raw_velocity = true;
          }
          // Only advance the reference position once enough time has elapsed so the
          // next delta is computed over a meaningful interval.
          it->second.position = position;
          it->second.stamp = current_stamp;
        }
        // If dt < kMinDt_s or dt == 0, keep the old reference to accumulate more time.
      } else if (it == position_history_.end()) {
        position_history_[object.id] = {position, current_stamp};
      }
      // If current_stamp == it->second.stamp (duplicate timestamp), do not update the
      // reference position so the next frame's delta spans actual elapsed time.
    }

    // Step 2: Update per-object EMA
    auto& sv = smoothed_velocity_[object.id];
    if (have_raw_velocity) {
      // first measurement
      if (sv.last_update_stamp.nanoseconds() == 0) {
        sv.velocity = raw_velocity;
      }
      // subsequent measurements are smoothed with EMA
      else {
        sv.velocity.x = velocity_ema_alpha_ * raw_velocity.x + (1.0 - velocity_ema_alpha_) * sv.velocity.x;
        sv.velocity.y = velocity_ema_alpha_ * raw_velocity.y + (1.0 - velocity_ema_alpha_) * sv.velocity.y;
        sv.velocity.z = velocity_ema_alpha_ * raw_velocity.z + (1.0 - velocity_ema_alpha_) * sv.velocity.z;
      }
      sv.last_update_stamp = current_stamp;
    }
    // if no new measurement is available, hold the last velocity for a short window, then decay slowly
    else if (sv.last_update_stamp.nanoseconds() != 0 && dt_frame > 0.0) {
      const double time_since_update = std::max(0.0, (current_stamp - sv.last_update_stamp).seconds());
      if (time_since_update > velocity_hold_time_s_) {
        const double decay = std::exp(-dt_frame / velocity_decay_time_constant_s_);
        sv.velocity.x *= decay;
        sv.velocity.y *= decay;
        sv.velocity.z *= decay;
      }
    }

    // Step 3: Write smoothed velocity to the object state, overriding sensor zero on dt==0 frames.
    if (sv.last_update_stamp.nanoseconds() != 0 &&
        std::hypot(sv.velocity.x, sv.velocity.y) > std::numeric_limits<double>::epsilon()) {
      const double yaw = perception_msgs::object_access::getYaw(object.state);
      perception_msgs::object_access::setVelocityXYZYaw(object.state, sv.velocity, yaw, false);
    }
  }
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

    // Look up per-object state once before iterating over all candidates.
    const auto state_it = match_state_.find(prediction_object.object.id);

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

      const bool use_inverted = (inverted_difference < lanelet_difference);

      if (use_inverted) {
        matched_lanelet = lanelet.invert();
        start_arc_length = inverted_arc_length;
        orientation_difference = inverted_difference;
      } else {
        orientation_difference = lanelet_difference;
      }

      if (orientation_difference > lanelet_match_max_yaw_diff_rad_) {
        continue;
      }

      prediction_object.lanelet_matches.push_back(
          LaneletMatch{matched_lanelet, candidate_lanelet.first, start_arc_length, orientation_difference});
    }

    if (prediction_object.lanelet_matches.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Object %zu did not match any lanelet within %.2f m", object_index,
                   lanelet_match_max_distance_m_);
    } else {
      // Pick the closest lanelet; tie-break by heading alignment, then by ID for determinism.
      std::sort(prediction_object.lanelet_matches.begin(), prediction_object.lanelet_matches.end(),
                [](const LaneletMatch& a, const LaneletMatch& b) {
                  if (std::abs(a.distance - b.distance) > 1e-3) return a.distance < b.distance;
                  if (std::abs(a.orientation_difference - b.orientation_difference) > 1e-3)
                    return a.orientation_difference < b.orientation_difference;
                  return a.lanelet.id() < b.lanelet.id();
                });
      prediction_object.lanelet_matches.resize(1);

      // Persist the winner and smooth its arc-length with EMA to reduce position-noise
      // jitter. Arc-length EMA only applies when lanelet and direction are both unchanged.
      const lanelet::Id winner_id = prediction_object.lanelet_matches[0].lanelet.id();
      const bool winner_inverted = prediction_object.lanelet_matches[0].lanelet.inverted();
      const bool same_state = state_it != match_state_.end() && state_it->second.lanelet_id == winner_id &&
                              state_it->second.inverted == winner_inverted;

      double& arc = prediction_object.lanelet_matches[0].start_arc_length;
      if (same_state) {
        arc = arc_length_ema_alpha_ * arc + (1.0 - arc_length_ema_alpha_) * state_it->second.smoothed_arc_length;
      }
      match_state_[prediction_object.object.id] = {winner_id, winner_inverted, arc};
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

  const double probability = 1.0 / static_cast<double>(predictions.size());
  for (perception_msgs::msg::ObjectStatePrediction& prediction : predictions) {
    prediction.probability = probability;
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

  for (const LaneletMatch& match : prediction_object.lanelet_matches) {
    lanelet::routing::LaneletPaths routes;
    if (max_travel_distance <= std::numeric_limits<double>::epsilon()) {
      routes.push_back(lanelet::routing::LaneletPath({match.lanelet}));
    } else {
      lanelet::routing::PossiblePathsParams params;
      params.routingCostLimit = max_travel_distance + match.start_arc_length;
      params.includeShorterPaths = false;
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
      for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
        const double travel_distance = speed * prediction_sample_interval_s_ * static_cast<double>(sample_index + 1);
        prediction.states.push_back(sampleStateOnLaneletRoute(prediction_object.object.state, lanelet_route,
                                                              match.start_arc_length, travel_distance, base_time, sample_index));
      }
      predictions.push_back(prediction);
    }
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
    const builtin_interfaces::msg::Time& base_time,
    std::size_t sample_index) const {
  perception_msgs::msg::ObjectState state = base_state;
  if (route.empty()) {
    RCLCPP_WARN(this->get_logger(), "Lanelet route is empty, cannot sample lanelet prediction");
    return state;
  }

  const double speed = travel_distance / (prediction_sample_interval_s_ * static_cast<double>(sample_index + 1));
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
  routing_graph_map_ = ll2_interface_->getMapPtr();
  if (routing_graph_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Lanelet2 map pointer is null, cannot build routing graph");
    return;
  }

  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      static_cast<const char*>(lanelet::Locations::Germany), static_cast<const char*>(lanelet::Participants::Vehicle));
  routing_graph_ = lanelet::routing::RoutingGraph::build(*routing_graph_map_, *traffic_rules);

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
