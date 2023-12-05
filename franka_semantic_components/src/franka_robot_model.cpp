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

#include "franka_semantic_components/franka_robot_model.hpp"

#include <cstring>
#include <iostream>
#include "rclcpp/logging.hpp"
namespace {

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> &&
                     std::is_trivially_copyable_v<To>,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible_v<To>,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace franka_semantic_components {

FrankaRobotModel::FrankaRobotModel(const std::string& franka_model_interface_name,
                                   const std::string& franka_state_interface_name)
    : SemanticComponentInterface(franka_model_interface_name, 2) {
  interface_names_.emplace_back(franka_model_interface_name);
  interface_names_.emplace_back(franka_state_interface_name);
}

void FrankaRobotModel::initialize() {
  auto franka_state_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == arm_id_ + "/" + robot_state_interface_name_;
      });

  auto franka_model_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == arm_id_ + "/" + robot_model_interface_name_;
      });

  if (franka_state_interface != state_interfaces_.end() &&
      franka_model_interface != state_interfaces_.end()) {
    robot_model = bit_cast<franka_hardware::Model*>((*franka_model_interface).get().get_value());
    robot_state = bit_cast<franka::RobotState*>((*franka_state_interface).get().get_value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_model_semantic_component"),
                 "Franka interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    throw std::runtime_error("Franka state interfaces does not exist");
  }
  initialized = true;
}
}  // namespace franka_semantic_components
