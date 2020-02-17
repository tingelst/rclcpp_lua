// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HARDWARE_MANAGER__HARDWARE_MANAGER_HPP_
#define HARDWARE_MANAGER__HARDWARE_MANAGER_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "hardware_manager/hardware_loader_interface.hpp"
#include "hardware_manager/visibility_control.h"

#include "hardware_interface/robot_hardware.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"

namespace hardware_manager
{

class RobotHardwareManager : public rclcpp::Node
{
public:
  HARDWARE_MANAGER_PUBLIC
  RobotHardwareManager(
    const std::string & name = "hardware_manager");

  HARDWARE_MANAGER_PUBLIC
  virtual
  ~RobotHardwareManager() = default;

  HARDWARE_MANAGER_PUBLIC
  std::shared_ptr<hardware_interface::RobotHardware>
  load_hardware(
    const std::string & hardware_name,
    const std::string & hardware_type);

  HARDWARE_MANAGER_PUBLIC
  std::vector<std::shared_ptr<hardware_interface::RobotHardware>>
  get_loaded_hardware() const;

  HARDWARE_MANAGER_PUBLIC
  void register_hardware_loader(RobotHardwareLoaderInterfaceSharedPtr loader);

  template<
    typename T,
    typename std::enable_if<std::is_convertible<
      T *, hardware_interface::RobotHardwareInterface *>::value, T>::type * = nullptr>
  std::shared_ptr<hardware_interface::RobotHardwareInterface>
  add_hardware(std::shared_ptr<T> hardware, std::string hardware_name)
  {
    return add_hardware_impl(hardware, hardware_name);
  }

protected:
  HARDWARE_MANAGER_PUBLIC
  std::shared_ptr<hardware_interface::RobotHardware>
  add_hardware_impl(
    std::shared_ptr<hardware_interface::RobotHardware> hardware,
    const std::string & hardware_name);

private:
  std::vector<RobotHardwareLoaderInterfaceSharedPtr> loaders_;
  std::vector<std::shared_ptr<hardware_interface::RobotHardware>> loaded_hardwares_;
};

}  // namespace hardware_manager

#endif  // HARDWARE_MANAGER__HARDWARE_MANAGER_HPP_
