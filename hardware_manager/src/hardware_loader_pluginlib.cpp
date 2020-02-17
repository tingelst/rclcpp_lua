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

#include "hardware_manager/hardware_loader_pluginlib.hpp"

#include <memory>
#include <string>

namespace hardware_manager
{

RobotHardwareLoaderPluginlib::RobotHardwareLoaderPluginlib()
: RobotHardwareLoaderInterface(),
  loader_(std::make_shared<pluginlib::ClassLoader<hardware_interface::RobotHardware>>(
      "hardware_interface", "hardware_interface::RobotHardware"))
{
}

std::shared_ptr<hardware_interface::RobotHardware> RobotHardwareLoaderPluginlib::create(
  const std::string & hardware_type)
{
  return loader_->createSharedInstance(hardware_type);
}

bool RobotHardwareLoaderPluginlib::is_available(const std::string & hardware_type) const
{
  return loader_->isClassAvailable(hardware_type);
}

}  // namespace hardware_manager
