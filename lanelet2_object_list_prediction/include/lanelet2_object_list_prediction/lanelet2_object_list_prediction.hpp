// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lanelet2_routing/Forward.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lanelet2_object_list_prediction {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Lanelet2ObjectListPrediction class
 */
class Lanelet2ObjectListPrediction : public rclcpp::Node {
 public:
  Lanelet2ObjectListPrediction();

 private:
  /**
   * @brief Lanelet match candidate for one perceived object
   */
  struct LaneletMatch {
    lanelet::ConstLanelet lanelet;  ///< Matched lanelet in the direction used for routing
    double distance;                ///< Lateral distance from object position to lanelet geometry in meters
    double start_arc_length;        ///< Arc length of the projected object position along the matched centerline
    double orientation_difference;  ///< Absolute yaw difference between object heading and lanelet direction in radians
  };

  /**
   * @brief Internal object representation used while predicting
   */
  struct PredictionObject {
    perception_msgs::msg::Object object;        ///< Object in map frame, later enriched with state predictions
    std::vector<LaneletMatch> lanelet_matches;  ///< Lanelet candidates accepted for map-based prediction
  };

  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Checks if map is loaded and handles map updates
   *
   * @param[in] handle_update whether to handle map update
   * @return whether map is loaded (and updated, if supposed to handle update)
   */
  bool checkMap(bool handle_update);

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr& msg);

  /**
   * @brief Match all objects in an object list to lanelets in the current map
   *
   * @param object_list object list in map frame
   * @return internal prediction objects containing the original objects and their map matches
   */
  std::vector<PredictionObject> matchObjectListToMap(const perception_msgs::msg::ObjectList& object_list) const;

  /**
   * @brief Rebuilds the lanelet2 routing graph from the current map
   */
  void rebuildRoutingGraphFromMap();

  /**
   * @brief Dispatches prediction creation for one object
   *
   * @param prediction_object object and lanelet matches in map frame
   * @param base_time time stamp of the first received state
   * @return map-based predictions or the configured fallback prediction
   */
  std::vector<perception_msgs::msg::ObjectStatePrediction> createPredictionsForMatchedObject(
      const PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const;

  /**
   * @brief Creates route alternatives for an object matched to lanelets
   *
   * @param prediction_object object and lanelet matches in map frame
   * @param base_time time stamp of the input object list
   * @return one prediction per possible lanelet route
   */
  std::vector<perception_msgs::msg::ObjectStatePrediction> createMapBasedPredictions(
      const PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const;

  /**
   * @brief Creates a stationary fallback prediction
   *
   * @param object object in map frame
   * @param base_time time stamp of the input object list
   * @return one prediction with repeated object position and zero velocity
   */
  perception_msgs::msg::ObjectStatePrediction createStationaryPrediction(const perception_msgs::msg::Object& object,
                                                                         const builtin_interfaces::msg::Time& base_time) const;

  /**
   * @brief Creates a constant-velocity fallback prediction
   *
   * @param object object in map frame
   * @param base_time time stamp of the input object list
   * @return one prediction sampled by integrating the object's current velocity
   */
  perception_msgs::msg::ObjectStatePrediction createConstantVelocityPrediction(
      const perception_msgs::msg::Object& object, const builtin_interfaces::msg::Time& base_time) const;

  /**
   * @brief Samples one predicted state along a lanelet route
   *
   * @param base_state current object state used as template
   * @param route lanelet route to sample
   * @param start_arc_length current object position along the first route lanelet
   * @param travel_distance distance to travel along the route from the current position
   * @param base_time time stamp of the input object list
   * @param sample_index zero-based prediction sample index
   * @return predicted object state at the requested sample
   */
  perception_msgs::msg::ObjectState sampleStateOnLaneletRoute(const perception_msgs::msg::ObjectState& base_state,
                                                              const lanelet::routing::LaneletPath& route,
                                                              double start_arc_length,
                                                              double travel_distance,
                                                              const builtin_interfaces::msg::Time& base_time,
                                                              std::size_t sample_index) const;

  /**
   * @brief Computes the number of predicted states per prediction
   *
   * @return fixed sample count derived from horizon and sample interval
   */
  std::size_t getPredictionSampleCount() const;

  /**
   * @brief Writes pose, velocity and timestamp into a predicted state
   *
   * @param state state message to update
   * @param x x position in map frame
   * @param y y position in map frame
   * @param z z position in map frame
   * @param yaw yaw angle in map frame
   * @param velocity velocity vector in map frame
   * @param base_time time stamp of the input object list
   * @param sample_index zero-based prediction sample index
   */
  void setPredictedStateKinematics(perception_msgs::msg::ObjectState& state,
                                   double x,
                                   double y,
                                   double z,
                                   double yaw,
                                   const geometry_msgs::msg::Vector3& velocity,
                                   const builtin_interfaces::msg::Time& base_time,
                                   std::size_t sample_index) const;

 private:
  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp::Publisher<perception_msgs::msg::ObjectList>::SharedPtr publisher_;

  /**
   * @brief TF buffer for object-list transformations
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief TF listener for object-list transformations
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief Lanelet2 map interface
   */
  std::unique_ptr<Lanelet2MapInterface> ll2_interface_;

  /**
   * @brief Name of lanelet2_map_server node (parameter)
   */
  std::string ll2_map_server_name_ = "lanelet2_map_server";

  /**
   * @brief Maximum object-to-lanelet matching distance in meters (parameter)
   */
  double lanelet_match_max_distance_m_ = 0.0;

  /**
   * @brief Maximum yaw difference for accepting a lanelet match in radians (parameter)
   */
  double lanelet_match_max_yaw_diff_rad_ = 1.57079632679;

  /**
   * @brief Prediction horizon in seconds (parameter)
   */
  double prediction_horizon_s_ = 5.0;

  /**
   * @brief Sampling interval of predicted states in seconds (parameter)
   */
  double prediction_sample_interval_s_ = 0.5;

  /**
   * @brief Prediction mode for unmatched objects: "static" or "kinematic" (parameter)
   */
  std::string unmatched_object_prediction_mode_ = "kinematic";

  /**
   * @brief Lanelet2 routing graph
   */
  lanelet::routing::RoutingGraphUPtr routing_graph_;

  /**
   * @brief Map pointer used when the routing graph was built
   */
  lanelet::LaneletMapConstPtr routing_graph_map_;
};

}  // namespace lanelet2_object_list_prediction
