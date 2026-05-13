#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_map_interface/lanelet2_map_interface.hpp>

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
  void topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

  /**
   * @brief Lanelet2 map interface
   */
  std::unique_ptr<LL2MapInterface> ll2_interface_;

  /**
   * @brief Name of lanelet2_map_server node (parameter)
   */
  std::string ll2_map_server_name_ = "lanelet2_map_server";
};

}  // namespace lanelet2_object_list_prediction
