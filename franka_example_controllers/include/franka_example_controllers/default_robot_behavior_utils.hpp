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

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

#include <string>

/**
 * @brief Helper function to prepare default robot behavior messages of the robot.
 *
 */
class DefaultRobotBehavior {
 public:
  DefaultRobotBehavior() = default;

  /**
   * @brief prepares default collision behavior of the robot for the control examples.
   *
   * @return default full collision robot behavior service request message
   */
  franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr
  getDefaultCollisionBehaviorRequest();
};
