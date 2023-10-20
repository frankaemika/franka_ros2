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

// modified ros2 control semantic control interface to add command interface access.
// https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp

#pragma once

#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace franka_semantic_components {

class FrankaSemanticComponentInterface {
 public:
  explicit FrankaSemanticComponentInterface(const std::string& name,
                                            size_t state_interface_size = 0,
                                            size_t command_interface_size = 0) {
    name_ = name;
    state_interface_names_.reserve(state_interface_size);
    command_interface_names_.reserve(command_interface_size);

    state_interfaces_.reserve(state_interface_size);
    command_interfaces_.reserve(command_interface_size);
  }

  ~FrankaSemanticComponentInterface() = default;

  /// Assign loaned state interfaces from the hardware.
  /**
   * Assign loaned state interfaces on the controller start.
   *
   * \param[in] state_interfaces vector of interfaces provided by the controller.
   */
  bool assign_loaned_state_interfaces(
      std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    return controller_interface::get_ordered_interfaces(state_interfaces, state_interface_names_,
                                                        "", state_interfaces_);
  }

  /// Assign loaned command interfaces from the hardware.
  /**
   * Assign loaned command interfaces on the controller start.
   *
   * \param[in] command_interfaces vector of interfaces provided by the controller.
   */
  bool assign_loaned_command_interfaces(
      std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces) {
    return controller_interface::get_ordered_interfaces(
        command_interfaces, command_interface_names_, "", command_interfaces_);
  }

  /// Release loaned interfaces from the hardware.
  void release_interfaces() {
    state_interfaces_.clear();
    command_interfaces_.clear();
  }

  /// Definition of state interface names for the component.
  /**
   * The function should be used in "state_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined state interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with state interface names for the semantic component.
   */
  virtual std::vector<std::string> get_state_interface_names() {
    if (state_interface_names_.empty()) {
      for (auto i = 0u; i < state_interface_names_.capacity(); ++i) {
        state_interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
      }
    }
    return state_interface_names_;
  }

  /// Definition of command interface names for the component.
  /**
   * The function should be used in "command_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined command interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with command interface names for the semantic component.
   */
  virtual std::vector<std::string> get_command_interface_names() {
    if (command_interface_names_.empty()) {
      for (auto i = 0u; i < command_interface_names_.capacity(); ++i) {
        command_interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
      }
    }
    return command_interface_names_;
  }

  /// Return all values.
  /**
   * \return true if it gets all the values, else false
   */
  bool get_values(std::vector<double>& values) const {
    // check we have sufficient memory
    if (values.capacity() != state_interfaces_.size()) {
      return false;
    }
    // insert all the values
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      values.emplace_back(state_interfaces_[i].get().get_value());
    }
    return true;
  }

  bool set_values(const std::vector<double>& values) const {
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

 protected:
  std::string name_;
  std::vector<std::string> state_interface_names_;
  std::vector<std::string> command_interface_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      command_interfaces_;
};

}  // namespace franka_semantic_components
