// Copyright 2020 Norwegian University of Science and Technology
// Copyright 2020 PAL Robotics SL
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

#ifndef HARDWARE_MANAGER__HARDWARE_LOADER_PLUGINLIB_HPP_
#define HARDWARE_MANAGER__HARDWARE_LOADER_PLUGINLIB_HPP_

#include "hardware_manager/hardware_loader_interface.hpp"

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"

namespace hardware_manager
{

class RobotHardwareLoaderPluginlib : public RobotHardwareLoaderInterface
{
public:
  HARDWARE_MANAGER_PUBLIC
  RobotHardwareLoaderPluginlib();

  HARDWARE_MANAGER_PUBLIC
  virtual ~RobotHardwareLoaderPluginlib() = default;

  HARDWARE_MANAGER_PUBLIC
  std::shared_ptr<hardware_interface::RobotHardware>
  create(const std::string & hardware_type);

  HARDWARE_MANAGER_PUBLIC
  bool is_available(const std::string & hardware_type) const;

private:
  std::shared_ptr<pluginlib::ClassLoader<hardware_interface::RobotHardware>> loader_;
};

}  // namespace hardware_manager

#endif  // HARDWARE_MANAGER__HARDWARE_LOADER_PLUGINLIB_HPP_
