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

#include "franka_semantic_components/franka_state.hpp"

#include <cstring>

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

franka_msgs::msg::Errors errorsToMessage(const franka::Errors& error) {
  franka_msgs::msg::Errors message;
  message.joint_position_limits_violation =
      static_cast<decltype(message.joint_position_limits_violation)>(
          error.joint_position_limits_violation);
  message.cartesian_position_limits_violation =
      static_cast<decltype(message.cartesian_position_limits_violation)>(
          error.cartesian_position_limits_violation);
  message.self_collision_avoidance_violation =
      static_cast<decltype(message.self_collision_avoidance_violation)>(
          error.self_collision_avoidance_violation);
  message.joint_velocity_violation =
      static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
  message.cartesian_velocity_violation =
      static_cast<decltype(message.cartesian_velocity_violation)>(
          error.cartesian_velocity_violation);
  message.force_control_safety_violation =
      static_cast<decltype(message.force_control_safety_violation)>(
          error.force_control_safety_violation);
  message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
  message.cartesian_reflex =
      static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
  message.max_goal_pose_deviation_violation =
      static_cast<decltype(message.max_goal_pose_deviation_violation)>(
          error.max_goal_pose_deviation_violation);
  message.max_path_pose_deviation_violation =
      static_cast<decltype(message.max_path_pose_deviation_violation)>(
          error.max_path_pose_deviation_violation);
  message.cartesian_velocity_profile_safety_violation =
      static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
          error.cartesian_velocity_profile_safety_violation);
  message.joint_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
          error.joint_position_motion_generator_start_pose_invalid);
  message.joint_motion_generator_position_limits_violation =
      static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
          error.joint_motion_generator_position_limits_violation);
  message.joint_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
          error.joint_motion_generator_velocity_limits_violation);
  message.joint_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
          error.joint_motion_generator_velocity_discontinuity);
  message.joint_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
          error.joint_motion_generator_acceleration_discontinuity);
  message.cartesian_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
          error.cartesian_position_motion_generator_start_pose_invalid);
  message.cartesian_motion_generator_elbow_limit_violation =
      static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
          error.cartesian_motion_generator_elbow_limit_violation);
  message.cartesian_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
          error.cartesian_motion_generator_velocity_limits_violation);
  message.cartesian_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
          error.cartesian_motion_generator_velocity_discontinuity);
  message.cartesian_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
          error.cartesian_motion_generator_acceleration_discontinuity);
  message.cartesian_motion_generator_elbow_sign_inconsistent =
      static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
          error.cartesian_motion_generator_elbow_sign_inconsistent);
  message.cartesian_motion_generator_start_elbow_invalid =
      static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
          error.cartesian_motion_generator_start_elbow_invalid);
  message.cartesian_motion_generator_joint_position_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
          error.cartesian_motion_generator_joint_position_limits_violation);
  message.cartesian_motion_generator_joint_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
          error.cartesian_motion_generator_joint_velocity_limits_violation);
  message.cartesian_motion_generator_joint_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
          error.cartesian_motion_generator_joint_velocity_discontinuity);
  message.cartesian_motion_generator_joint_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
          error.cartesian_motion_generator_joint_acceleration_discontinuity);
  message.cartesian_position_motion_generator_invalid_frame =
      static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
          error.cartesian_position_motion_generator_invalid_frame);
  message.force_controller_desired_force_tolerance_violation =
      static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
          error.force_controller_desired_force_tolerance_violation);
  message.controller_torque_discontinuity =
      static_cast<decltype(message.controller_torque_discontinuity)>(
          error.controller_torque_discontinuity);
  message.start_elbow_sign_inconsistent =
      static_cast<decltype(message.start_elbow_sign_inconsistent)>(
          error.start_elbow_sign_inconsistent);
  message.communication_constraints_violation =
      static_cast<decltype(message.communication_constraints_violation)>(
          error.communication_constraints_violation);
  message.power_limit_violation =
      static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
  message.joint_p2p_insufficient_torque_for_planning =
      static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
          error.joint_p2p_insufficient_torque_for_planning);
  message.tau_j_range_violation =
      static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
  message.instability_detected =
      static_cast<decltype(message.instability_detected)>(error.instability_detected);
  return message;
}

}  // anonymous namespace

namespace franka_semantic_components {

FrankaState::FrankaState(const std::string& name) : SemanticComponentInterface(name, 1) {
  interface_names_.emplace_back(name_);
  // TODO: Set default values to NaN
}

bool FrankaState::get_values_as_message(franka_msgs::msg::FrankaState& message) {
  auto franka_state_interface = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [](const auto& interface) { return interface.get().get_name() == "panda/franka_state"; });

  if (franka_state_interface != state_interfaces_.end()) {
    robot_state_ptr_ = bit_cast<franka::RobotState*>((*franka_state_interface).get().get_value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_state_semantic_component"),
                 "Franka state interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    return false;
  }

  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->cartesian_contact),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->K_F_ext_hat_K),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->O_F_ext_hat_K),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->O_dP_EE_d),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->O_dP_EE_c),
      "Robot state Cartesian members do not have same size");
  static_assert(
      sizeof(robot_state_ptr_->cartesian_collision) == sizeof(robot_state_ptr_->O_ddP_EE_c),
      "Robot state Cartesian members do not have same size");
  for (size_t i = 0; i < robot_state_ptr_->cartesian_collision.size(); i++) {
    message.cartesian_collision[i] = robot_state_ptr_->cartesian_collision[i];
    message.cartesian_contact[i] = robot_state_ptr_->cartesian_contact[i];
    message.k_f_ext_hat_k[i] = robot_state_ptr_->K_F_ext_hat_K[i];
    message.o_f_ext_hat_k[i] = robot_state_ptr_->O_F_ext_hat_K[i];
    message.o_dp_ee_d[i] = robot_state_ptr_->O_dP_EE_d[i];
    message.o_dp_ee_c[i] = robot_state_ptr_->O_dP_EE_c[i];
    message.o_ddp_ee_c[i] = robot_state_ptr_->O_ddP_EE_c[i];
  }

  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->q_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->dq),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->dq_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->ddq_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->tau_J),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->dtau_J),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->tau_J_d),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->theta),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->dtheta),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->joint_collision),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->joint_contact),
                "Robot state joint members do not have same size");
  static_assert(sizeof(robot_state_ptr_->q) == sizeof(robot_state_ptr_->tau_ext_hat_filtered),
                "Robot state joint members do not have same size");
  for (size_t i = 0; i < robot_state_ptr_->q.size(); i++) {
    message.q[i] = robot_state_ptr_->q[i];
    message.q_d[i] = robot_state_ptr_->q_d[i];
    message.dq[i] = robot_state_ptr_->dq[i];
    message.dq_d[i] = robot_state_ptr_->dq_d[i];
    message.ddq_d[i] = robot_state_ptr_->ddq_d[i];
    message.tau_j[i] = robot_state_ptr_->tau_J[i];
    message.dtau_j[i] = robot_state_ptr_->dtau_J[i];
    message.tau_j_d[i] = robot_state_ptr_->tau_J_d[i];
    message.theta[i] = robot_state_ptr_->theta[i];
    message.dtheta[i] = robot_state_ptr_->dtheta[i];
    message.joint_collision[i] = robot_state_ptr_->joint_collision[i];
    message.joint_contact[i] = robot_state_ptr_->joint_contact[i];
    message.tau_ext_hat_filtered[i] = robot_state_ptr_->tau_ext_hat_filtered[i];
  }

  static_assert(sizeof(robot_state_ptr_->elbow) == sizeof(robot_state_ptr_->elbow_d),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr_->elbow) == sizeof(robot_state_ptr_->elbow_c),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr_->elbow) == sizeof(robot_state_ptr_->delbow_c),
                "Robot state elbow configuration members do not have same size");
  static_assert(sizeof(robot_state_ptr_->elbow) == sizeof(robot_state_ptr_->ddelbow_c),
                "Robot state elbow configuration members do not have same size");

  for (size_t i = 0; i < robot_state_ptr_->elbow.size(); i++) {
    message.elbow[i] = robot_state_ptr_->elbow[i];
    message.elbow_d[i] = robot_state_ptr_->elbow_d[i];
    message.elbow_c[i] = robot_state_ptr_->elbow_c[i];
    message.delbow_c[i] = robot_state_ptr_->delbow_c[i];
    message.ddelbow_c[i] = robot_state_ptr_->ddelbow_c[i];
  }

  static_assert(sizeof(robot_state_ptr_->O_T_EE) == sizeof(robot_state_ptr_->F_T_EE),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr_->O_T_EE) == sizeof(robot_state_ptr_->EE_T_K),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr_->O_T_EE) == sizeof(robot_state_ptr_->O_T_EE_d),
                "Robot state transforms do not have same size");
  static_assert(sizeof(robot_state_ptr_->O_T_EE) == sizeof(robot_state_ptr_->O_T_EE_c),
                "Robot state transforms do not have same size");
  for (size_t i = 0; i < robot_state_ptr_->O_T_EE.size(); i++) {
    message.o_t_ee[i] = robot_state_ptr_->O_T_EE[i];
    message.f_t_ee[i] = robot_state_ptr_->F_T_EE[i];
    message.ee_t_k[i] = robot_state_ptr_->EE_T_K[i];
    message.o_t_ee_d[i] = robot_state_ptr_->O_T_EE_d[i];
    message.o_t_ee_c[i] = robot_state_ptr_->O_T_EE_c[i];
  }
  message.m_ee = robot_state_ptr_->m_ee;
  message.m_load = robot_state_ptr_->m_load;
  message.m_total = robot_state_ptr_->m_total;

  for (size_t i = 0; i < robot_state_ptr_->I_load.size(); i++) {
    message.i_ee[i] = robot_state_ptr_->I_ee[i];
    message.i_load[i] = robot_state_ptr_->I_load[i];
    message.i_total[i] = robot_state_ptr_->I_total[i];
  }

  for (size_t i = 0; i < robot_state_ptr_->F_x_Cload.size(); i++) {
    message.f_x_cee[i] = robot_state_ptr_->F_x_Cee[i];
    message.f_x_cload[i] = robot_state_ptr_->F_x_Cload[i];
    message.f_x_ctotal[i] = robot_state_ptr_->F_x_Ctotal[i];
  }

  message.time = robot_state_ptr_->time.toSec();
  message.control_command_success_rate = robot_state_ptr_->control_command_success_rate;
  message.current_errors = errorsToMessage(robot_state_ptr_->current_errors);
  message.last_motion_errors = errorsToMessage(robot_state_ptr_->last_motion_errors);

  switch (robot_state_ptr_->robot_mode) {
    case franka::RobotMode::kOther:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_OTHER;
      break;

    case franka::RobotMode::kIdle:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_IDLE;
      break;

    case franka::RobotMode::kMove:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_MOVE;
      break;

    case franka::RobotMode::kGuiding:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_GUIDING;
      break;

    case franka::RobotMode::kReflex:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_REFLEX;
      break;

    case franka::RobotMode::kUserStopped:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_USER_STOPPED;
      break;

    case franka::RobotMode::kAutomaticErrorRecovery:
      message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
      break;
  }
  return true;
}

}  // namespace franka_semantic_components
