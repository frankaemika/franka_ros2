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

#include <franka_example_controllers/joint_velocity_example_controller.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  rclcpp::Duration time_max(8.0, 0.0);
  double omega_max = 0.1;
  double cycle = std::floor(std::pow(
      -1.0, (elapsed_time_.seconds() - std::fmod(elapsed_time_.seconds(), time_max.seconds())) /
                time_max.seconds()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.seconds() * elapsed_time_.seconds()));

  for (int i = 0; i < num_joints; i++) {
    if (i == 3 || i == 4) {
      command_interfaces_[i].set_value(omega);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_acceleration = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
  request->upper_torque_thresholds_acceleration = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
  request->lower_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
  request->upper_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
  request->lower_force_thresholds_acceleration = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_acceleration = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};
  request->lower_force_thresholds_nominal = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_nominal = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};

  if (!client->wait_for_service(20s)) {
    RCLCPP_FATAL(get_node()->get_logger(), "service server can't be found.");
    return CallbackReturn::FAILURE;
  }

  client->async_send_request(request);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerInterface)