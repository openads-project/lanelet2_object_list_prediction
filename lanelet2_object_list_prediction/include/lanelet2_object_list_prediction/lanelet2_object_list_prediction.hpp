// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lanelet2_routing/Forward.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lanelet2_object_list_prediction {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/** One sample of a route-relative longitudinal motion profile. */
struct RouteMotionSample {
  double distance{0.0};
  double speed{0.0};
  double time{0.0};
};

/** Motion profile and its combined kinematic feasibility. */
struct RouteMotionProfile {
  std::vector<RouteMotionSample> samples;
  bool feasible{true};
};

enum class PredictionParticipant { Vehicle, Bicycle, Pedestrian };

/**
 * @brief Lanelet2ObjectListPrediction class
 */
class Lanelet2ObjectListPrediction : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node, declares parameters, and calls setup
   */
  Lanelet2ObjectListPrediction();

 private:
  /**
   * @brief Lanelet match candidate for one perceived object
   */
  struct LaneletMatch {
    lanelet::ConstLanelet lanelet;   ///< Matched lanelet in the direction used for routing
    PredictionParticipant participant{PredictionParticipant::Vehicle};
    double distance;                 ///< Lateral distance from object position to lanelet
                                     ///< geometry in meters
    double centerline_distance{0.0};  ///< Distance used to rank motorcycle lane matches
    double start_arc_length;         ///< Arc length of the projected object position
                                     ///< along the matched centerline
    double orientation_difference;   ///< Absolute yaw difference between object
                                     ///< heading and lanelet direction in radians
    bool reversing{false};           ///< Velocity points backward along the legal lanelet
    double longitudinal_speed{0.0};  ///< Speed along the direction of travel
    double lateral_speed{0.0};       ///< Signed velocity across the direction of travel
  };

  /**
   * @brief Internal object representation used while predicting
   */
  struct PredictionObject {
    perception_msgs::msg::Object object;        ///< Object in map frame, later enriched with state predictions
    std::vector<LaneletMatch> lanelet_matches;  ///< Lanelet candidates accepted
                                                ///< for map-based prediction

    struct Hypothesis {
      lanelet::routing::LaneletPath route;
      PredictionParticipant participant{PredictionParticipant::Vehicle};
      double match_weight{1.0};
      double start_arc_length{0.0};
      double initial_speed{0.0};
      double initial_lateral_speed{0.0};
      bool reversing{false};
      bool stop_at_route_end{false};
      bool feasible{true};
      std::vector<RouteMotionSample> motion_profile;
      perception_msgs::msg::ObjectStatePrediction prediction;
    };

    std::vector<Hypothesis> hypotheses;  ///< Map-based hypotheses retained for
                                         ///< scene-level interaction processing
  };

  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of
   * parameter
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

  /** Stores the latest ego state and planned trajectory for interaction
   * prediction. */
  void egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr& msg);

  /**
   * @brief Match all objects in an object list to lanelets in the current map
   *
   * @param object_list object list in map frame
   * @return internal prediction objects containing the original objects and
   * their map matches
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
      PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const;

  /**
   * @brief Creates route alternatives for an object matched to lanelets
   *
   * @param prediction_object object and lanelet matches in map frame
   * @param base_time time stamp of the input object list
   * @return one prediction per possible lanelet route
   */
  void createMapBasedPredictions(PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const;

  /** Applies right-of-way constraints using nominal hypotheses from the
   * complete scene. */
  void applyYieldInteractions(std::vector<PredictionObject>& prediction_objects,
                              const builtin_interfaces::msg::Time& base_time,
                              const std::optional<perception_msgs::msg::EgoData>& ego_data) const;

  /** Applies a single car-following pass using the yielded scene as input. */
  void applyFollowingInteractions(std::vector<PredictionObject>& prediction_objects,
                                  const builtin_interfaces::msg::Time& base_time,
                                  const std::optional<perception_msgs::msg::EgoData>& ego_data) const;

  /** Rebuilds sampled messages from retained motion profiles and normalizes
   * their probabilities. */
  void finalizeMapBasedPredictions(PredictionObject& prediction_object, const builtin_interfaces::msg::Time& base_time) const;

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
   * @param start_arc_length current object position along the first route
   * lanelet
   * @param travel_distance distance to travel along the route from the current
   * position
   * @param speed longitudinal speed at the sampled position
   * @param initial_speed observed speed along the route
   * @param initial_lateral_speed observed speed across the route
   * @param reversing whether the vehicle is moving backward in its lane
   * @param base_time time stamp of the input object list
   * @param sample_index zero-based prediction sample index
   * @return predicted object state at the requested sample
   */
  perception_msgs::msg::ObjectState sampleStateOnLaneletRoute(const perception_msgs::msg::ObjectState& base_state,
                                                              const lanelet::routing::LaneletPath& route,
                                                              double start_arc_length,
                                                              double travel_distance,
                                                              double speed,
                                                              double initial_speed,
                                                              double initial_lateral_speed,
                                                              bool reversing,
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

  /** Subscriber for ego state and planned trajectory. */
  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr ego_data_subscriber_;

  /** Latest ego message, transformed on demand when an object list arrives. */
  perception_msgs::msg::EgoData::ConstSharedPtr latest_ego_data_;

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

  /** Timeout for considering ego vehicle data [s]. */
  double input_ego_data_timeout_ = 1.0;

  /** Name of lanelet2_map_server node. */
  std::string processing_map_matching_ll2_map_server_name_ = "lanelet2_map_server";

  /** Max distance from a lanelet to consider it a match [m]. */
  double processing_map_matching_max_distance_ = 0.5;

  /** Max yaw difference from a lanelet direction to consider it a match [deg]. */
  double processing_map_matching_max_delta_yaw_deg_ = 90.0;

  /** Fallback mode for objects not matched to map [kinematic|static]. */
  std::string processing_map_matching_fallback_mode_ = "kinematic";

  /** Enable kinematic limitations. */
  bool processing_kinematic_limitations_enable_ = true;

  /** Max lateral acceleration for predictions [m/s^2]. */
  double processing_kinematic_limitations_max_lateral_acceleration_ = 2.5;

  /** Max longitudinal deceleration for predictions [m/s^2]. */
  double processing_kinematic_limitations_max_longitudinal_deceleration_ = 2.0;

  /** Max longitudinal acceleration for predictions [m/s^2]. */
  double processing_kinematic_limitations_max_longitudinal_acceleration_ = 1.0;

  /** Enable yielding. */
  bool processing_yielding_enable_ = true;

  /** Clearance between front and yield line [m]. */
  double processing_yielding_clearance_distance_ = 0.5;

  /** Time to wait after priority traffic has cleared [s]. */
  double processing_yielding_clearance_time_ = 1.0;

  /** Enable following, avoiding collisions with leading objects. */
  bool processing_following_enable_ = true;

  /** Min distance to the leading object [m]. */
  double processing_following_headway_distance_ = 2.0;

  /** Min time headway to the leading object [s]. */
  double processing_following_headway_time_ = 1.0;

  /** Enable special roundabout handling. */
  bool processing_roundabout_enable_ = true;

  /** Tolerance for initial alignment with roundabout centerline, not respecting kinematic limitations [m]. */
  double processing_roundabout_initial_alignment_tolerance_ = 1.0;

  /** Prediction time horizon [s]. */
  double output_prediction_horizon_ = 5.0;

  /** Time interval between prediction samples [s]. */
  double output_sample_interval_ = 0.5;

  /** Probability for infeasible hypotheses. */
  double output_infeasible_hypothesis_probability_ = 0.0;

  /**
   * @brief Lanelet2 routing graph
   */
  lanelet::routing::RoutingGraphUPtr routing_graph_;

  /** Routing graph for bicycles, including bicycle-only lanelets. */
  lanelet::routing::RoutingGraphUPtr bicycle_routing_graph_;
  lanelet::routing::RoutingGraphUPtr pedestrian_routing_graph_;

  /**
   * @brief Traffic rules used for lanelet matching and routing
   */
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_;

  /** Traffic rules used to match bicycles to bicycle-accessible lanelets. */
  lanelet::traffic_rules::TrafficRulesUPtr bicycle_traffic_rules_;
  lanelet::traffic_rules::TrafficRulesUPtr pedestrian_traffic_rules_;

  /**
   * @brief Map pointer used when the routing graph was built
   */
  lanelet::LaneletMapConstPtr routing_graph_map_;
};

}  // namespace lanelet2_object_list_prediction
