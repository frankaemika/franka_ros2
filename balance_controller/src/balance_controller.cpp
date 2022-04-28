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

#include <balance_controller/balance_controller.hpp>

#include <exception>
#include <string>

namespace balance_controller {

controller_interface::InterfaceConfiguration
BalanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
BalanceController::state_interface_configuration() const {
  return {};
}

controller_interface::return_type BalanceController::update() {
  updateJointStates();

  /*
  auto now = std::chrono::system_clock::now();
  delta_t_ = (now - last_update_time_stamp_).count();
  last_update_time_stamp_ = now;

  // Integral of the error
  auto x_error_integral = x_prev_error_integral_ + current_position_.x * delta_t_;
  auto y_error_integral = y_prev_error_integral_ + current_position_.y * delta_t_;

  x_prev_error_integral_ = x_error_integral;
  y_prev_error_integral_ = y_error_integral;

  // Anti-windup
  // x_error_integral = std::max(std::min(x_error_integral, MAX_VALUE), -MAX_VALUE);

  // Derivative of the error
  auto x_error_derivative = (current_position_.x - previous_position_.x) / delta_t_;
  auto y_error_derivative = (current_position_.y - previous_position_.y) / delta_t_;

  // Controll coefficients
  double k_p = 5;
  double k_i = 0.2;
  double k_d = 0.1;

  auto x_delta =
      k_p * current_position_.x + k_i * x_prev_error_integral_ + k_d * x_error_derivative;
  auto y_delta =
      k_p * current_position_.y + k_i * y_prev_error_integral_ + k_d * y_error_derivative;

  /*
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0);
  }
  */
  //command_interfaces_[0].set_value(joint_positions_[0]-0.0005);
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0);
  }
  //command_interfaces_.at(6).set_value(-0.6);
  command_interfaces_.at(5).set_value(1.25);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BalanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = node_->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BalanceController::init(
    const std::string& controller_name) {
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void BalanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_(i) = position_interface.get_value();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController,
                       controller_interface::ControllerInterface)
