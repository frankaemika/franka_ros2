// Copyright (c) 2021 Franka Emika GmbH
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

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace balance_controller {

/**
 * The gravity compensation controller only sends zero torques so that the robot does gravity
 * compensation
 */
class BalanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type init(const std::string& controller_name) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update() override;

 private:
  void updateJointStates();

  Vector7d joint_positions_;
  std::string arm_id_;
  const int num_joints = 7;
};
}  // namespace balance_controller
