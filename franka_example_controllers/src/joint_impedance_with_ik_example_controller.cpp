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

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/joint_impedance_with_ik_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <chrono>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceWithIKExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceWithIKExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config;
}

void JointImpedanceWithIKExampleController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    // TODO(yazi_ba) Can we get the state from its name?
    const auto& position_interface = state_interfaces_.at(16 + i);
    const auto& velocity_interface = state_interfaces_.at(23 + i);
    const auto& effort_interface = state_interfaces_.at(30 + i);
    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
    joint_efforts_current_[i] = effort_interface.get_value();
  }
}

Eigen::Vector3d JointImpedanceWithIKExampleController::compute_new_position() {
  elapsed_time_ = elapsed_time_ + trajectory_period_;
  double radius = 0.1;

  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));

  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);

  Eigen::Vector3d new_position = position_;
  new_position.x() -= delta_x;
  new_position.z() -= delta_z;

  return new_position;
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
JointImpedanceWithIKExampleController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current,
    const std::vector<double>& joint_velocities_current,
    const std::vector<double>& joint_efforts_current) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_id_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_id_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  service_request->ik_request.robot_state.joint_state.name = {
      arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
      arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;
  service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
  service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

  // If Franka Hand is not connected, the following line should be commented out.
  service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  return service_request;
}

Vector7d JointImpedanceWithIKExampleController::compute_torque_command(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  return tau_d_calculated;
}

controller_interface::return_type JointImpedanceWithIKExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getInitialOrientationAndTranslation();
    initialization_flag_ = false;
  }
  update_joint_states();

  Eigen::Vector3d new_position = compute_new_position();

  auto service_request =
      create_ik_service_request(new_position, orientation_, joint_positions_current_,
                                joint_velocities_current_, joint_efforts_current_);

  using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
  auto response_received_callback =
      [&](ServiceResponseFuture future) {  // NOLINT(performance-unnecessary-value-param)
        const auto& response = future.get();

        if (response->error_code.val == response->error_code.SUCCESS) {
          joint_positions_desired_ = response->solution.joint_state.position;
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
        }
      };
  auto result_future_ =
      compute_ik_client_->async_send_request(service_request, response_received_callback);

  Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command(
      joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceWithIKExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

bool JointImpedanceWithIKExampleController::assign_parameters() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  return true;
}

CallbackReturn JointImpedanceWithIKExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

  while (!compute_ik_client_->wait_for_service(1ms) && !collision_client->wait_for_service(1ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
  }

  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = collision_client->async_send_request(request);

  auto success = future_result.get();

  if (!success->success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceWithIKExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  dq_filtered_.setZero();
  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceWithIKExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceWithIKExampleController,
                       controller_interface::ControllerInterface)