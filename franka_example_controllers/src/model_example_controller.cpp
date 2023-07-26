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

#include <franka_example_controllers/model_example_controller.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_example_controllers {

controller_interface::CallbackReturn ModelExampleController::on_init() {
  try {
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ModelExampleController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ModelExampleController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_model_name : franka_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_model_name);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn ModelExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_model_ = std::make_unique<franka_semantic_components::FrankaModel>(
      franka_semantic_components::FrankaModel(arm_id_ + "/" + k_robot_model_interface_name,
                                              arm_id_ + "/" + k_robot_state_interface_name));

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_model_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ModelExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  std::array<double, 49> mass = franka_model_->getMassMatrix();
  std::array<double, 7> coriolis = franka_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = franka_model_->getGravityForceVector();
  std::array<double, 16> pose = franka_model_->getPoseMatrix(franka::Frame::kJoint4);
  std::array<double, 42> joint4_body_jacobian_wrt_joint4 =
      franka_model_->getBodyJacobian(franka::Frame::kJoint4);
  std::array<double, 42> endeffector_jacobian_wrt_base =
      franka_model_->getZeroJacobian(franka::Frame::kEndEffector);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "mass :" << mass);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "coriolis :" << coriolis);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "gravity :" << gravity);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "joint_pose :" << pose);
  RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint4_body_jacobian in joint4 frame :" << joint4_body_jacobian_wrt_joint4);
  RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "end_effector_jacobian in base frame :" << endeffector_jacobian_wrt_base);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelExampleController,
                       controller_interface::ControllerInterface)
