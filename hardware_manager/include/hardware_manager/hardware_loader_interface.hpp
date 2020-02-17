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

#ifndef HARDWARE_MANAGER__HARDWARE_LOADER_INTERFACE_HPP_
#define HARDWARE_MANAGER__HARDWARE_LOADER_INTERFACE_HPP_

#include <memory>
#include <string>

#include "hardware_interface/robot_hardware.hpp"

#include "hardware_manager/visibility_control.h"

namespace hardware_manager
{

class RobotHardwareLoaderInterface
{
public:
  HARDWARE_MANAGER_PUBLIC
  RobotHardwareLoaderInterface() = default;

  HARDWARE_MANAGER_PUBLIC
  virtual ~RobotHardwareLoaderInterface() = default;

  HARDWARE_MANAGER_PUBLIC
  virtual std::shared_ptr<hardware_interface::RobotHardware> create(
    const std::string & hardware_type) = 0;

  HARDWARE_MANAGER_PUBLIC
  virtual bool is_available(const std::string & hardware_type) const = 0;
};

using RobotHardwareLoaderInterfaceSharedPtr = std::shared_ptr<RobotHardwareLoaderInterface>;

}  // namespace hardware_manager


#endif  // HARDWARE_MANAGER__HARDWARE_LOADER_INTERFACE_HPP_
