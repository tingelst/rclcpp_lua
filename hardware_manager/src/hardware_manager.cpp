// Copyright 2020 Norwegian University of Science and Technology
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

#include "hardware_manager/hardware_manager.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/robot_hardware_interface.hpp"

#include "hardware_manager/hardware_loader_pluginlib.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/logging_macros.h"

namespace hardware_manager
{

RobotHardwareManager::RobotHardwareManager(
  const std::string & manager_node_name)
: rclcpp::Node(manager_node_name),
  // add pluginlib loader by default
  loaders_({std::make_shared<RobotHardwareLoaderPluginlib>()})
{
}

std::shared_ptr<hardware_interface::RobotHardware>
RobotHardwareManager::load_hardware(
  const std::string & hardware_name,
  const std::string & hardware_type)
{
  RCUTILS_LOG_INFO("Loading hardware '%s'\n", hardware_name.c_str());

  auto it = std::find_if(
    loaders_.cbegin(), loaders_.cend(),
    [&](auto loader)
    {return loader->is_available(hardware_type);});

  std::shared_ptr<hardware_interface::RobotHardware> hardware(nullptr);
  if (it != loaders_.cend()) {
    hardware = (*it)->create(hardware_type);
  } else {
    const std::string error_msg("Loader for hardware '" + hardware_name + "' not found\n");
    RCUTILS_LOG_ERROR("%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  return add_hardware_impl(hardware, hardware_name);
}

std::vector<std::shared_ptr<hardware_interface::RobotHardware>>
RobotHardwareManager::get_loaded_hardware() const
{
  return loaded_hardwares_;
}

void RobotHardwareManager::register_hardware_loader(RobotHardwareLoaderInterfaceSharedPtr loader)
{
  loaders_.push_back(loader);
}

std::shared_ptr<hardware_interface::RobotHardware>
RobotHardwareManager::add_hardware_impl(
  std::shared_ptr<hardware_interface::RobotHardware> hardware,
  const std::string & hardware_name)
{
  loaded_hardwares_.emplace_back(hardware);
  return loaded_hardwares_.back();
}


}  // namespace hardware_manager
