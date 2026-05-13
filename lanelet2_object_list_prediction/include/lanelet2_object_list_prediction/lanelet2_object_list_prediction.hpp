#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

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
    lanelet::ConstLanelet lanelet;
    double distance;
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
   * @brief Match all objects in an object list to lanelets
   *
   * @param object_list object list in map frame
   * @return lanelet match candidates per object
   */
  std::vector<std::vector<LaneletMatch>> matchObjectsToLanelets(const perception_msgs::msg::ObjectList& object_list) const;

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
  std::unique_ptr<LL2MapInterface> ll2_interface_;

  /**
   * @brief Name of lanelet2_map_server node (parameter)
   */
  std::string ll2_map_server_name_ = "lanelet2_map_server";

  /**
   * @brief Maximum object-to-lanelet matching distance in meters (parameter)
   */
  double lanelet_match_max_distance_m_ = 0.0;
};

}  // namespace lanelet2_object_list_prediction
