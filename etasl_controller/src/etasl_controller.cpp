#include "etasl_controller/etasl_controller.hpp"

namespace etasl_controller
{
EtaslController::EtaslController()
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