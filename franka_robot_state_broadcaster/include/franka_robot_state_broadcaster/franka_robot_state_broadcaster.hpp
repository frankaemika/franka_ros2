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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka_robot_state_broadcaster_parameters.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace franka_robot_state_broadcaster {
class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 protected:
  std::shared_ptr<ParamListener> param_listener;
  Params params;

  std::string arm_id{"panda"};
  std::string state_interface_name{"robot_state"};
  std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>> franka_state_publisher;
  std::shared_ptr<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>>
      realtime_franka_state_publisher;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state;
};

}  // namespace franka_robot_state_broadcaster
