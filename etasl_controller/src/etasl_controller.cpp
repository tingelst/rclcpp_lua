#include "etasl_controller/etasl_controller.hpp"

namespace etasl_controller
{
EtaslController::EtaslController() 
: etasl_(std::make_shared<etasl_driver::EtaslDriver>(300, 0.0, 0.001))
{
}

controller_interface::controller_interface_ret_t EtaslController::init(
    std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string& controller_name)
{
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    return ret;
  }
  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}

controller_interface::controller_interface_ret_t EtaslController::update()
{
  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_error(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EtaslController::add_input_scalar(const std::string& input_name)
{
  auto callback = [this, input_name](std_msgs::msg::Float64::UniquePtr msg) {
    scalar_input_map_["global." + input_name] = msg->data;
  };
  scalar_subscribers_.push_back(lifecycle_node_->create_subscription<std_msgs::msg::Float64>(
      "~/" + input_name, rclcpp::SystemDefaultsQoS(), callback));
}

void EtaslController::add_output_scalar(const std::string& output_name)
{
  scalar_output_names_.push_back(output_name);
  scalar_publishers_.push_back(
      lifecycle_node_->create_publisher<std_msgs::msg::Float64>("~/" + output_name, rclcpp::SystemDefaultsQoS()));
}

void EtaslController::read_task_specification_string(const std::string& task_specification)
{
  etasl_->readTaskSpecificationString(task_specification);
}

void EtaslController::read_task_specification_file(const std::string& filename) 
{
  etasl_->readTaskSpecificationFile(filename);
}

bool EtaslController::reset()
{
  return true;
}

void EtaslController::set_op_mode(const hardware_interface::OperationMode& mode)
{
}

void EtaslController::halt()
{
}

}  // namespace etasl_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(etasl_controller::EtaslController, controller_interface::ControllerInterface);