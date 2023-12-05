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

#include "translation_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace franka_semantic_components {
namespace translation {

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

/**
 * @param input_wrench The wrench which should be translated
 * @return geometry_msgs::msg::Wrench The translated wrench
 */
auto toWrench(const std::array<double, 6>& input_wrench) -> geometry_msgs::msg::Wrench {
  geometry_msgs::msg::Wrench output_wrench;
  output_wrench.force = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                            .x(input_wrench[0])
                            .y(input_wrench[1])
                            .z(input_wrench[2]);
  output_wrench.torque = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                             .x(input_wrench[3])
                             .y(input_wrench[4])
                             .z(input_wrench[5]);

  return output_wrench;
}

/**
 * @param input_twist The twist which should be translated
 * @return geometry_msgs::msg::Twist The translated twist
 */
auto toTwist(const std::array<double, 6>& input_twist) -> geometry_msgs::msg::Twist {
  geometry_msgs::msg::Twist output_twist;
  output_twist.linear = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                            .x(input_twist[0])
                            .y(input_twist[1])
                            .z(input_twist[2]);
  output_twist.angular = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                             .x(input_twist[3])
                             .y(input_twist[4])
                             .z(input_twist[5]);

  return output_twist;
}

/**
 * @param input_accel The acceleration which should be translated
 * @return geometry_msgs::msg::Accel The translated acceleration
 */
auto toAccel(const std::array<double, 6>& input_accel) -> geometry_msgs::msg::Accel {
  geometry_msgs::msg::Accel output_accel;
  output_accel.linear = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                            .x(input_accel[0])
                            .y(input_accel[1])
                            .z(input_accel[2]);
  output_accel.angular = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                             .x(input_accel[3])
                             .y(input_accel[4])
                             .z(input_accel[5]);

  return output_accel;
}

/**
 * @param input_pose The pose which should be translated
 * @return geometry_msgs::msg::Pose The translated pose
 */
auto toPose(const std::array<double, 16>& input_pose) -> geometry_msgs::msg::Pose {
  const Eigen::Map<const Eigen::Matrix4d> transformation_matrix(input_pose.data());
  const Eigen::Quaterniond quaternion(transformation_matrix.topLeftCorner<3, 3>());
  const Eigen::Translation3d translation(transformation_matrix.block<3, 1>(0, 3));

  geometry_msgs::msg::Pose output_pose;
  output_pose.position = geometry_msgs::build<geometry_msgs::msg::Point>()
                             .x(translation.x())
                             .y(translation.y())
                             .z(translation.z());
  output_pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                .x(quaternion.x())
                                .y(quaternion.y())
                                .z(quaternion.z())
                                .w(quaternion.w());

  return output_pose;
}

/**
 * @param mass The mass for the inertia
 * @param center_of_mass The center of mass for the inertia
 * @param inertia_matrix The inertia matrix
 * @return geometry_msgs::msg::Inertia The translated inertia
 */
auto toInertia(double mass,
               const std::array<double, 3>& center_of_mass,
               const std::array<double, 9>& inertia_matrix) -> geometry_msgs::msg::Inertia {
  geometry_msgs::msg::Inertia output_inertia =
      geometry_msgs::build<geometry_msgs::msg::Inertia>()
          .m(mass)
          .com(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                   .x(center_of_mass[0])
                   .y(center_of_mass[1])
                   .z(center_of_mass[2]))
          // Definition of inertia matrix can be found here:
          // https://docs.ros2.org/latest/api/geometry_msgs/msg/Inertia.html
          .ixx(inertia_matrix[0])
          .ixy(inertia_matrix[1])
          .ixz(inertia_matrix[2])
          .iyy(inertia_matrix[4])
          .iyz(inertia_matrix[5])
          .izz(inertia_matrix[8]);

  return output_inertia;
}

/**
 * The indicators within this message represent if a collision/contact is active for Cartesian/joint
 * space
 * @param cartesian_collision The Cartesian collision flags
 * @param cartesian_contact The Cartesian contact flags
 * @param joint_collision The joint collision flags
 * @param joint_contact The joint contact flags
 * @return franka_msgs::msg::CollisionIndicators The translated CollisionIndicator message
 */
auto toCollisionIndicators(const std::array<double, 6>& cartesian_collision,
                           const std::array<double, 6>& cartesian_contact,
                           const std::array<double, 7>& joint_collision,
                           const std::array<double, 7>& joint_contact)
    -> franka_msgs::msg::CollisionIndicators {
  franka_msgs::msg::CollisionIndicators collision_indicators;
  collision_indicators.is_cartesian_linear_collision =
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(cartesian_collision[0])
          .y(cartesian_collision[1])
          .z(cartesian_collision[2]);
  collision_indicators.is_cartesian_angular_collision =
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(cartesian_collision[3])
          .y(cartesian_collision[4])
          .z(cartesian_collision[5]);

  collision_indicators.is_cartesian_linear_contact =
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(cartesian_contact[0])
          .y(cartesian_contact[1])
          .z(cartesian_contact[2]);
  collision_indicators.is_cartesian_angular_contact =
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(cartesian_contact[3])
          .y(cartesian_contact[4])
          .z(cartesian_contact[5]);

  std::copy(joint_collision.cbegin(), joint_collision.cend(),
            collision_indicators.is_joint_collision.begin());
  std::copy(joint_contact.cbegin(), joint_contact.cend(),
            collision_indicators.is_joint_contact.begin());

  return collision_indicators;
}

/**
 * @param elbow The elbow position
 * @param elbow_d The desired elbow position
 * @param elbow_c The commanded elbow position
 * @param delbow_c The commanded elbow velocity
 * @param ddelbow_c The commanded elbow acceleration
 * @return franka_msgs::msg::Elbow The translated elbow message
 */
auto toElbow(const std::array<double, 2>& elbow,
             const std::array<double, 2>& elbow_d,
             const std::array<double, 2>& elbow_c,
             const std::array<double, 2>& delbow_c,
             const std::array<double, 2>& ddelbow_c) -> franka_msgs::msg::Elbow {
  franka_msgs::msg::Elbow elbow_message;

  for (size_t i = 0; i < elbow.size(); i++) {
    elbow_message.position.at(i) = elbow.at(i);
    elbow_message.desired_position.at(i) = elbow_d.at(i);
    elbow_message.commanded_position.at(i) = elbow_c.at(i);
    elbow_message.commanded_velocity.at(i) = delbow_c.at(i);
    elbow_message.commanded_acceleration.at(i) = ddelbow_c.at(i);
  }

  return elbow_message;
}

}  // namespace translation
}  // namespace franka_semantic_components
