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

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "franka/control_types.h"
#include "franka/robot_state.h"
#include "franka_semantic_components/franka_semantic_component.hpp"

/**
 * @brief Franka Cartesian Velocity interface abstraction on top of hardware_interface to set the
 * full cartesian velocity Command should have the form [linear_velocity, angular_velocity] = [vx,
 * vy, vz, wx, wy, wz] Optionally elbow can be commanded. [joint_3_position, joint_4_sign]
 */
namespace franka_semantic_components {
class FrankaCartesianVelocityInterface : public FrankaSemanticComponentInterface {
 public:
  explicit FrankaCartesianVelocityInterface(bool command_elbow_activate);

  virtual ~FrankaCartesianVelocityInterface() = default;

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  bool setCommand(const std::array<double, 6>& command) noexcept;

  /**
   * Sets the given command.
   *
   * @param[in] cartesian_velocity_command Command to set.
   * @param[in] elbow Elbow to set.
   */
  bool setCommand(const std::array<double, 6>& command,
                  const std::array<double, 2>& elbow) noexcept;

 private:
  std::array<std::string, 6> hw_cartesian_velocities_names_{"vx", "vy", "vz", "wx", "wy", "wz"};
  std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};
  const size_t state_interface_size_{0};
  const size_t command_interface_size_{8};
  bool command_elbow_active_;

  const std::string cartesian_velocity_command_interface_name_{"cartesian_velocity"};
  const std::string elbow_command_interface_name_{"elbow_command"};
};

}  // namespace franka_semantic_components
