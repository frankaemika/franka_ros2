// Copyright (c) 2026 Franka Robotics GmbH
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

#include <franka/model.h>
#include <franka/robot_state.h>
#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <franka_gazebo_hardware/gravity_compensation_model.hpp>
#include <franka_hardware/model.hpp>
#include <gz/sim/EntityComponentManager.hh>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <map>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <string>
#include <vector>

namespace franka_gazebo_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Gazebo ros2_control hardware component that wraps the stock
 * gz_ros2_control::GazeboSimSystem and broadens it into the simulation
 * counterpart of franka_hardware::FrankaHardwareInterface.
 *
 * The wrapped system is loaded through pluginlib and handles all joint and
 * sensor logic. Every hardware-interface call is forwarded to it unchanged.
 * On top of that this class adds two Franka-specific responsibilities:
 *   - model-based gravity compensation on the effort commands (write()),
 *   - exporting the `robot_model` and `robot_state` state interfaces consumed
 *     by franka_semantic_components::FrankaRobotModel, so model-based
 *     controllers can activate in simulation just like on real hardware, and
 *   - exporting the six `<prefix><robot_type>_tcp` force/torque state interfaces
 *     a semantic_components::ForceTorqueSensor requests, fed from an estimated
 *     external wrench (see updateRobotState()), and
 *   - exporting the 16 `<prefix><i>/cartesian_pose_state` interfaces claimed by
 *     FrankaCartesianPoseInterface, carrying the synthetic column-major O_T_EE.
 */
class FrankaGazeboHardwareInterface : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  auto initSim(rclcpp::Node::SharedPtr& model_nh,
               std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info,
               sim::EntityComponentManager& ecm,
               unsigned int update_rate) -> bool override;

  auto on_init(const hardware_interface::HardwareInfo& system_info) -> CallbackReturn override;

  auto on_configure(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

  auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;

  auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;

  auto perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                   const std::vector<std::string>& stop_interfaces)
      -> hardware_interface::return_type override;

  auto read(const rclcpp::Time& time,
            const rclcpp::Duration& period) -> hardware_interface::return_type override;

  auto write(const rclcpp::Time& time,
             const rclcpp::Duration& period) -> hardware_interface::return_type override;

 private:
  /**
   * Builds the gravity model and the mapping from simulated joint entities to
   * their position/effort role in that model.
   *
   * @param hardware_info Parsed ros2_control hardware description (URDF + interfaces)
   * @param joints Map from simulated joint name to its Gazebo entity
   */
  auto initGravityModel(const hardware_interface::HardwareInfo& hardware_info,
                        const std::map<std::string, sim::Entity>& joints) -> void;

  /**
   * Builds the offline libfranka model from the URDF and wraps it for the
   * semantic component, and reads the `robot_type`/`prefix` hardware parameters
   * that name the exported model/state interfaces.
   *
   * @param hardware_info Parsed ros2_control hardware description (URDF + params)
   * @throws std::exception if the URDF cannot be turned into a franka::Model
   */
  auto initRobotModel(const hardware_interface::HardwareInfo& hardware_info) -> void;

  /**
   * Builds a synthetic franka::RobotState from the current simulated joint
   * positions/velocities and publishes it into robot_state_box_, so the
   * model semantic component sees a live configuration in simulation.
   *
   * @param time Current simulation time, used for RobotState::time.
   * @param period Time since the previous read(), used for the dq finite-diff.
   */
  auto updateRobotState(const rclcpp::Time& time, const rclcpp::Duration& period) -> void;

  /**
   * Estimates the external wrench from a measured-torque residual and writes it
   * into the RobotState (O_F_ext_hat_K, K_F_ext_hat_K, tau_ext_hat_filtered) and
   * the tcp_wrench_ force/torque interface storage.
   *
   * tau_ext = tau_measured - gravity - coriolis - M*ddq, low-pass filtered, then
   * mapped to the EE frame via the Jacobian-transpose pseudo-inverse. tau_measured
   * is the JointTransmittedWrench projected on each joint axis (the same value the
   * wrapped system exposes as `effort`); ddq is a finite-diff of dq (skipped on the
   * first cycle / dt<=0). K_F_ext_hat_K is the same wrench rotated into the
   * end-effector / stiffness frame K via O_R_EE^T (force and torque separately).
   *
   * O_F_ext_hat_K is the external wrench the environment applies TO the robot, matching
   * libfranka's convention (a contact pushing the flange +Z reads O_F_ext_hat_K.z > 0).
   * The raw (J^T)^+ solve yields the opposite sign (the reaction DART reports as the
   * child->parent JointTransmittedWrench), so the wrench and tau_ext_hat_filtered are
   * negated. At rest the residual is ~0, so the sign correction leaves the baseline at 0.
   *
   * @param robot_state RobotState (q/dq already filled) updated in place.
   * @param gravity_torque Model gravity(q).
   * @param coriolis_torque Model coriolis(q,dq).
   * @param zero_jacobian Base-frame EE Jacobian (6x7, column-major).
   * @param period Time since the previous read(), for the dq finite-diff.
   */
  auto estimateExternalWrench(franka::RobotState& robot_state,
                              const std::array<double, 7>& gravity_torque,
                              const std::array<double, 7>& coriolis_torque,
                              const std::array<double, 42>& zero_jacobian,
                              const rclcpp::Duration& period) -> void;

  // Loader must outlive the wrapped system, so it is declared first.
  std::shared_ptr<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>> system_loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> wrapped_system_;

  sim::EntityComponentManager* ecm_ = nullptr; /**< Not owned; valid for the sim's lifetime. */
  rclcpp::Node::SharedPtr model_node_; /**< Owns the logger/clock for write()'s error log. */
  GravityCompensationModel gravity_model_;
  // Aligned with gravity_model_.configurationJoints(): each entity's position is
  // read from the ECM and fed to the gravity model in the same order.
  std::vector<sim::Entity> configuration_entities_;
  // Aligned with gravity_model_.effortJoints(): each entity receives the computed
  // gravity torque on its effort command in the same order.
  std::vector<sim::Entity> effort_entities_;
  // Per-cycle buffers, sized once in initGravityModel() so write() never allocates.
  std::vector<double> joint_positions_;
  std::vector<double> gravity_torques_;

  // Offline libfranka model built from the URDF, and the thin franka_hardware
  // wrapper the semantic component dereferences. Mirrors franka_hardware's
  // FrankaHardwareInterface: the StateInterface smuggles a pointer-to-pointer
  // (&hw_model_ptr_) reinterpret_cast to double*, so hw_model_ptr_ must stay
  // alive and stable for the controller's lifetime.
  std::unique_ptr<franka::Model> franka_model_;
  std::unique_ptr<franka_hardware::Model> hw_model_;
  franka_hardware::Model* hw_model_ptr_ = nullptr;
  // True only when a single-arm franka::Model could be built for this robot_type. When
  // false (e.g. the dual-arm mobile platform has no single franka::Model), the decorator
  // degrades to gravity compensation + passthrough and exports no model/robot_state/
  // force-torque interfaces.
  bool model_available_ = false;

  // Mirrors franka_hardware's robot_state_box_. read() writes a synthetic
  // franka::RobotState here; consumers copy via try_get()/get() through the
  // smuggled &robot_state_box_ptr_ pointer.
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState> robot_state_box_;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_ptr_ =
      &robot_state_box_;

  // The first seven configuration joints (in model order) are the arm joints
  // whose q/dq populate the synthetic RobotState consumed by the model.
  static constexpr std::size_t kArmJointCount = 7;

  // Backing storage for the six <prefix><robot_type>_tcp force/torque state
  // interfaces (force.x/y/z, torque.x/y/z). Unlike robot_state/robot_model these
  // are plain doubles read directly by semantic_components::ForceTorqueSensor, not
  // pointer-smuggled. Filled each cycle with the estimated external wrench
  // O_F_ext_hat_K, matching the same value in the synthetic RobotState.
  std::array<double, 6> tcp_wrench_{};

  // Backing storage for the 16 "<prefix><i>/cartesian_pose_state" interfaces claimed by
  // FrankaCartesianPoseInterface (Cartesian pose/impedance controllers). Holds the
  // column-major 4x4 synthetic O_T_EE, refreshed each read(); initialised to identity.
  std::array<double, 16> cartesian_pose_state_{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // Each arm joint's rotation axis (unit Xyz), cached once at init from the
  // JointAxis ECM component. The wrapped system reads the same component at
  // registration time to build its effort state interface; a per-cycle lookup
  // returns null at run time, so estimateExternalWrench() projects the live
  // JointTransmittedWrench onto this cached axis instead.
  std::array<std::array<double, 3>, kArmJointCount> joint_axes_{};

  // Finite-difference state for ddq (joint acceleration) used to subtract the
  // inertial term M*ddq from the measured-torque residual. prev_dq_ holds the
  // previous cycle's joint velocity; prev_dq_valid_ guards the first cycle.
  std::array<double, kArmJointCount> prev_dq_{};
  bool prev_dq_valid_ = false;
  // First-order low-pass state for the external-torque residual, exported as
  // tau_ext_hat_filtered. Initialised to zero (no contact at start-up).
  std::array<double, kArmJointCount> tau_ext_filtered_{};

  // Reusable workspace for the Jacobian-transpose least-squares wrench solve in
  // estimateExternalWrench(). Hoisted to members so the read path computes the external
  // wrench (J^T * F = tau_ext) without allocating a fresh CompleteOrthogonalDecomposition
  // every cycle: jacobian_transpose_ caches J^T, wrench_solver_ reuses its decomposition
  // storage across .compute() calls, and external_wrench_ receives the solution.
  Eigen::Matrix<double, 7, 6> jacobian_transpose_;
  Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 7, 6>> wrench_solver_;
  Eigen::Matrix<double, 6, 1> external_wrench_;

  std::string robot_type_;
  std::string prefix_;
};

}  // namespace franka_gazebo_hardware
