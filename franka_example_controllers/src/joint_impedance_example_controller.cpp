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

#include "joint_impedance_example_controller.hpp"

#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::return_type JointImpedanceExampleController::update() {
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  std::vector<double> joint_torques;
  for (auto& state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == "position") {
      joint_positions.push_back(state_interface.get_value());
    }
    if (state_interface.get_interface_name() == "velocity") {
      joint_velocities.push_back(state_interface.get_value());
    }
    if (state_interface.get_interface_name() == "effort") {
      joint_torques.push_back(state_interface.get_value());
    }
  }
  Vector7 q_goal;
  q_ = Vector7(joint_positions.data());
  dq_ = Vector7(joint_velocities.data());

  if (first_time_) {
    first_time_ = false;
    initial_q_ = q_;
    start_time_ = this->node_->now();
  }
  auto time = this->node_->now() - start_time_;
  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time.seconds()));
  q_goal = initial_q_;
  for (int i = 3; i < 5; ++i) {
    q_goal(i) += delta_angle;
  }

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7 tau_d_calculated =
      0.04 * k_gains_.cwiseProduct(q_goal - q_) + 0.04 * d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type JointImpedanceExampleController::init(
    const std::string& controller_name) {
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  k_gains_ << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
  d_gains_ << 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0;
  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointImpedanceExampleController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  arm_id_ = node_->get_parameter("arm_id").as_string();
  dq_filtered_.setZero();
  return LifecycleNodeInterface::on_configure(previous_state);
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerInterface)