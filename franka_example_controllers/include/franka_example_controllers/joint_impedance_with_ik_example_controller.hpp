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

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * joint impedance example controller get desired pose and use moveit inverse kinematics
 */
class JointImpedanceWithIKExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void update_joint_states();
  Eigen::Vector3d compute_new_position();
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      Eigen::Vector3d new_position,
      Eigen::Quaterniond new_orientation,
      std::vector<double> joint_positions_desired,
      std::vector<double> joint_positions_current,
      std::vector<double> joint_efforts_current);

  Vector7d compute_torque_command(Vector7d joint_positions_desired,
                                  Vector7d joint_positions_current,
                                  Vector7d joint_velocities_current);

  bool assign_parameters();

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

  double trajectory_period_{0.001};
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string arm_id_;

  double elapsed_time_{0.0};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  int num_joints_{7};

  std::vector<double> joint_positions_desired;
  std::vector<double> joint_positions_current{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current{0, 0, 0, 0, 0, 0, 0};
};
}  // namespace franka_example_controllers