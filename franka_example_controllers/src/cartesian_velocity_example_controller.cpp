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

#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double cycle = std::floor(pow(
      -1.0,
      (elapsed_time_.seconds() - std::fmod(elapsed_time_.seconds(), k_time_max_)) / k_time_max_));
  double v =
      cycle * k_v_max_ / 2.0 * (1.0 - std::cos(2.0 * M_PI / k_time_max_ * elapsed_time_.seconds()));
  double v_x = std::cos(k_angle_) * v;
  double v_z = -std::sin(k_angle_) * v;

  std::array<double, 6> cartesian_velocity_command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};

  if (franka_cartesian_velocity_->setCommand(cartesian_velocity_command)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianVelocityExampleController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_nominal = {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
  request->upper_torque_thresholds_nominal = {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
  request->lower_torque_thresholds_acceleration = {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
  request->upper_torque_thresholds_acceleration = {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
  request->lower_force_thresholds_nominal = {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_nominal = {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};
  request->lower_force_thresholds_acceleration = {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_acceleration = {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};

  client->async_send_request(request);

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)