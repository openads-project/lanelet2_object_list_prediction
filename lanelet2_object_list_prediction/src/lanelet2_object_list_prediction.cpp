#include <functional>

#include <lanelet2_object_list_prediction/lanelet2_object_list_prediction.hpp>

namespace lanelet2_object_list_prediction {

Lanelet2ObjectListPrediction::Lanelet2ObjectListPrediction() : Node("lanelet2_object_list_prediction") {
  this->declareAndLoadParameter("ll2_map_server_name", ll2_map_server_name_, "Name of lanelet2_map_server node", false, false,
                                true);
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
  // map interface
  ll2_interface_ = std::make_unique<LL2MapInterface>(*this, ll2_map_server_name_);

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

  // publish message
  perception_msgs::msg::ObjectList out_msg;
  out_msg = *msg;
  publisher_->publish(out_msg);
  RCLCPP_INFO(this->get_logger(), "Message published with stamp: '%d'", out_msg.header.stamp.sec);
}

bool Lanelet2ObjectListPrediction::checkMap(bool handle_update) {
  bool map_status = ll2_interface_->map_loaded_;
  // update routing graph on map update
  if (handle_update && ll2_interface_->update_pending_ && ll2_interface_->map_loaded_) {
    ll2_interface_->update_pending_ = false;
    map_status = map_status && !ll2_interface_->update_pending_;
  }
  return map_status;
}

}  // namespace lanelet2_object_list_prediction

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
