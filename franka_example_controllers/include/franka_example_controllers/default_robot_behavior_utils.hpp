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

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

namespace DefaultRobotBehavior {

inline franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr
getDefaultCollisionBehaviorRequest() {
  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_nominal = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_nominal = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_torque_thresholds_acceleration = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_acceleration = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_nominal = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_nominal = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_acceleration = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_acceleration = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)

  return request;
}

}  // namespace DefaultRobotBehavior
