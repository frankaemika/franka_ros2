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

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "franka/robot_state.h"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaRobotState
    : public semantic_components::SemanticComponentInterface<franka_msgs::msg::FrankaRobotState> {
 public:
  explicit FrankaRobotState(const std::string& name);

  virtual ~FrankaRobotState() = default;

  /**
   * @param[in/out] message Initializes this message to contain the respective frame_id information
   */
  auto initialize_robot_state_msg(franka_msgs::msg::FrankaRobotState& message) -> void;

  /**
   * Constructs and return a FrankaRobotState message from the current values.
   * \return FrankaRobotState message from values;
   */
  auto get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool;

 protected:
  franka::RobotState* robot_state_ptr;

 private:
  const std::string robot_name_{"panda"};
  const std::string state_interface_name_{"robot_state"};
};

}  // namespace franka_semantic_components