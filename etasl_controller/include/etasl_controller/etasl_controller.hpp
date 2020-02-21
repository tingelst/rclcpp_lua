#ifndef ETASL_CONTROLLER__ETASL_CONTROLLER_HPP_
#define ETASL_CONTROLLER__ETASL_CONTROLLER_HPP_

#include "etasl_controller/visibility_control.h"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"

#include "etasl_driver/etasl_driver.hpp"

#include "kdl/frames.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64.hpp"

namespace etasl_controller
{
using ScalarMap = std::map<std::string, double>;
using VectorMap = std::map<std::string, KDL::Vector>;
using RotationMap = std::map<std::string, KDL::Rotation>;
using FrameMap = std::map<std::string, KDL::Frame>;
using TwistMap = std::map<std::string, KDL::Twist>;
using WrenchMap = std::map<std::string, KDL::Wrench>;

class EtaslController : public controller_interface::ControllerInterface
{
public:
  ETASL_CONTROLLER_PUBLIC
  EtaslController();

  ETASL_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
                                                        const std::string& controller_name) override;

  ETASL_CONTROLLER_PUBLIC
  controller_interface::controller_interface_ret_t update() override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;

  ETASL_CONTROLLER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  void add_input_scalar(const std::string& input_name);
  void add_output_scalar(const std::string& output_name);

  void read_task_specification_string(const std::string& task_specification);
  void read_task_specification_file(const std::string& filename);
  
  void set_joint_names(const std::vector<std::string>& joint_names);

private:
  bool reset();
  void set_op_mode(const hardware_interface::OperationMode& mode);
  void halt();


  std::vector<std::string> joint_names_;
  std::vector<std::string> write_op_names_;

  std::vector<hardware_interface::JointCommandHandle*> registered_joint_cmd_handles_;
  std::vector<const hardware_interface::JointStateHandle*> registered_joint_state_handles_;
  std::vector<hardware_interface::OperationModeHandle*> registered_operation_mode_handles_;

  bool is_halted_ = false;

  ScalarMap scalar_input_map_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> scalar_subscribers_;

  std::vector<std::string> scalar_output_names_;
  ScalarMap scalar_output_map_;
  std::vector<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> scalar_publishers_;

  std::shared_ptr<etasl_driver::EtaslDriver> etasl_;
};

}  // namespace etasl_controller

#endif  // ETASL_CONTROLLER__ETASL_CONTROLLER_HPP_
