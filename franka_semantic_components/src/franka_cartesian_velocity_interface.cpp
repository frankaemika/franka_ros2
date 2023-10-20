// Copyright (c) 2023 Franka Emika GmbH
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

#include "franka_semantic_components/franka_cartesian_velocity_interface.hpp"

#include <cstring>
#include <iostream>
#include <string>
#include "rclcpp/logging.hpp"

namespace {
std::vector<double> combineArraysToVector(const std::array<double, 6>& cartesian_velocity_command,
                                          const std::array<double, 2>& elbow_command) {
  std::vector<double> full_command;
  full_command.reserve(cartesian_velocity_command.size() + elbow_command.size());
  full_command.insert(full_command.end(), cartesian_velocity_command.begin(),
                      cartesian_velocity_command.end());
  full_command.insert(full_command.end(), elbow_command.begin(), elbow_command.end());

  return full_command;
}
}  // namespace

namespace franka_semantic_components {

FrankaCartesianVelocityInterface::FrankaCartesianVelocityInterface(bool command_elbow_active)
    : FrankaSemanticComponentInterface("cartesian_velocity_command", 0, 6) {
  command_elbow_active_ = command_elbow_active;
  if (command_elbow_active_) {
    command_interface_names_.reserve(command_interface_size_);
    command_interfaces_.reserve(command_interface_size_);
  }

  for (const auto& velocity_command_name : hw_cartesian_velocities_names_) {
    auto full_interface_name =
        velocity_command_name + "/" + cartesian_velocity_command_interface_name_;
    command_interface_names_.emplace_back(full_interface_name);
  }
  if (command_elbow_active_) {
    for (const auto& elbow_command_name : hw_elbow_command_names_) {
      auto full_elbow_command_name = elbow_command_name + "/" + elbow_command_interface_name_;
      command_interface_names_.emplace_back(full_elbow_command_name);
    }
  }
}

/**
 * Sets the given command.
 *
 * @param[in] command Command to set.
 */
bool FrankaCartesianVelocityInterface::setCommand(
    const std::array<double, 6>& cartesian_velocity_command,
    const std::array<double, 2>& elbow_command) noexcept {
  if (!command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must be claimed to command elbow.");
    return false;
  }
  auto full_command = combineArraysToVector(cartesian_velocity_command, elbow_command);

  return set_values(full_command);
}

bool FrankaCartesianVelocityInterface::setCommand(
    const std::array<double, 6>& cartesian_velocity_command) noexcept {
  if (command_elbow_active_) {
    RCLCPP_ERROR(rclcpp::get_logger("franka_cartesian_velocity_interface"),
                 "Elbow command interface must not claimed, if elbow is not commanded.");
    return false;
  }

  std::vector<double> full_command;

  full_command.reserve(cartesian_velocity_command.size());
  full_command.insert(full_command.end(), cartesian_velocity_command.begin(),
                      cartesian_velocity_command.end());

  return set_values(full_command);
}

}  // namespace franka_semantic_components