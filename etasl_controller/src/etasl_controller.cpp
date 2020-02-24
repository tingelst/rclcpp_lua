#include "etasl_controller/etasl_controller.hpp"

namespace etasl_controller
{
EtaslController::EtaslController() : etasl_(std::make_shared<etasl_driver::EtaslDriver>(300, 0.0, 0.001))
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
  etasl_->setInput(scalar_input_map_);

  std::map<std::string, double> joint_position_map;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_map[joint_names_[i]] = registered_joint_state_handles_[i]->get_position();
  }
  etasl_->setJointPos(joint_position_map);

  // Solve the optimization problem
  etasl_->updateStep(0.01);
  // Set the desired joint positions
  etasl_->getJointPos(joint_position_map);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    registered_joint_cmd_handles_[i]->set_cmd(joint_position_map[joint_names_[i]]);
  }

  if (!scalar_publishers_.empty())
  {
    etasl_->getOutput(scalar_output_map_);
    for (size_t index = 0; index < scalar_publishers_.size(); ++index)
    {
      auto msg = std_msgs::msg::Float64();
      msg.data = scalar_output_map_["global." + scalar_output_names_[index]];
      scalar_publishers_[index]->publish(msg);
    }
  }

  set_op_mode(hardware_interface::OperationMode::ACTIVE);

  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  auto logger = lifecycle_node_->get_logger();

  if (auto robot_hardware = robot_hardware_.lock())
  {
    if (joint_names_.empty())
    {
      RCLCPP_WARN(logger, "no joint names specified");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // register handles
    registered_joint_state_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index)
    {
      auto ret =
          robot_hardware->get_joint_state_handle(joint_names_[index].c_str(), &registered_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "unable to obtain joint state handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_joint_cmd_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index)
    {
      auto ret =
          robot_hardware->get_joint_command_handle(joint_names_[index].c_str(), &registered_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "unable to obtain joint command handle for %s", joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
    registered_operation_mode_handles_.resize(write_op_names_.size());
    for (size_t index = 0; index < write_op_names_.size(); ++index)
    {
      auto ret = robot_hardware->get_operation_mode_handle(write_op_names_[index].c_str(),
                                                           &registered_operation_mode_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "unable to obtain operation mode handle for %s", write_op_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
  }
  else
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EtaslController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  std::map<std::string, double> joint_position_map;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_map[joint_names_[i]] = registered_joint_state_handles_[i]->get_position();
  }

  std::map<std::string, double> converged_values_map;
  if (etasl_->initialize(joint_position_map, 10.0, 0.004, 1E-4, converged_values_map) < 0)
  {
    RCLCPP_ERROR(lifecycle_node_->get_logger(), "Could not initialize the eTaSl solver");
  }

  // Activate all scalar publishers
  for (auto pub : scalar_publishers_)
  {
    pub->on_activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;


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

void EtaslController::set_joint_names(const std::vector<std::string>& joint_names)
{
  joint_names_ = joint_names;
}

void EtaslController::console()
{
    return etasl_->console();
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