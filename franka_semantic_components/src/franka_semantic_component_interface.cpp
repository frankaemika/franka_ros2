// Copyright (c) 2023 Franka Robotics GmbH
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

// modified ros2 control semantic control interface to add command interface access.
// https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp

#include "franka_semantic_components/franka_semantic_component_interface.hpp"

namespace franka_semantic_components {

FrankaSemanticComponentInterface::FrankaSemanticComponentInterface(const std::string& name,
                                                                   size_t state_interface_size,
                                                                   size_t command_interface_size) {
  name_ = name;
  state_interface_names_.reserve(state_interface_size);
  command_interface_names_.reserve(command_interface_size);

  state_interfaces_.reserve(state_interface_size);
  command_interfaces_.reserve(command_interface_size);
}

bool FrankaSemanticComponentInterface::assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  return controller_interface::get_ordered_interfaces(state_interfaces, state_interface_names_, "",
                                                      state_interfaces_);
}

bool FrankaSemanticComponentInterface::assign_loaned_command_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces) {
  return controller_interface::get_ordered_interfaces(command_interfaces, command_interface_names_,
                                                      "", command_interfaces_);
}

void FrankaSemanticComponentInterface::release_interfaces() {
  state_interfaces_.clear();
  command_interfaces_.clear();
}

std::vector<std::string> FrankaSemanticComponentInterface::get_state_interface_names() {
  if (state_interface_names_.empty()) {
    for (auto i = 0u; i < state_interface_names_.capacity(); ++i) {
      state_interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
    }
  }
  return state_interface_names_;
}

std::vector<std::string> FrankaSemanticComponentInterface::get_command_interface_names() {
  if (command_interface_names_.empty()) {
    for (auto i = 0u; i < command_interface_names_.capacity(); ++i) {
      command_interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
    }
  }
  return command_interface_names_;
}

std::vector<double> FrankaSemanticComponentInterface::get_values_state_interfaces() const {
  std::vector<double> state_interface_values;
  // insert all the state_interface_values
  for (const auto& state_interface : state_interfaces_) {
    state_interface_values.emplace_back(state_interface.get().get_value());
  }
  return state_interface_values;
}

std::vector<double> FrankaSemanticComponentInterface::get_values_command_interfaces() const {
  std::vector<double> command_interface_values;
  // insert all the command_interface_values
  for (const auto& command_interface : command_interfaces_) {
    command_interface_values.emplace_back(command_interface.get().get_value());
  }

  return command_interface_values;
}

bool FrankaSemanticComponentInterface::set_values(const std::vector<double>& values) {
  // check we have sufficient memory
  if (values.capacity() != command_interfaces_.size()) {
    return false;
  }
  // // insert all the values
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].get().set_value(values[i]);
  }

  return true;
}

}  // namespace franka_semantic_components
