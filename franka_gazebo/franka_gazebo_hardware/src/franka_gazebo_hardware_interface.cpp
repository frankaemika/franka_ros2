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

#include <sys/wait.h>

#include <Eigen/Dense>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <franka_gazebo_hardware/franka_gazebo_hardware_interface.hpp>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace franka_gazebo_hardware {

namespace {
// Tight coupling to gz_ros2_control internals: these strings name the upstream
// pluginlib base interface and the stock system we decorate. If upstream renames
// or refactors GazeboSimSystem or its base interface, pluginlib can no longer
// resolve them and createSharedInstance throws at load time. The try/catch in
// initSim() now surfaces that as a logged error instead of crashing sim bring-up,
// but the names must still be kept in sync with gz_ros2_control by hand.
constexpr char kSystemPackage[] = "gz_ros2_control";
constexpr char kSystemBaseClass[] = "gz_ros2_control::GazeboSimSystemInterface";
constexpr char kWrappedSystemClass[] = "gz_ros2_control/GazeboSimSystem";

// Hardware-parameter keys, mirroring franka_hardware::FrankaHardwareInterface.
constexpr char kRobotTypeParam[] = "robot_type";
constexpr char kPrefixParam[] = "prefix";

// State-interface suffixes consumed by franka_semantic_components::FrankaRobotModel.
// These mirror the private k_robot_model_interface_name / k_robot_state_interface_name
// in franka_hardware/include/franka_hardware/franka_hardware_interface.hpp, which are
// not exported in a shared header, so they are duplicated here intentionally.
constexpr char kRobotModelInterfaceName[] = "robot_model";
constexpr char kRobotStateInterfaceName[] = "robot_state";

// Cartesian pose state interface name and element count, mirroring franka_hardware's
// k_HW_IF_CARTESIAN_POSE_STATE. FrankaCartesianPoseInterface (used by the Cartesian
// pose/impedance controllers) claims 16 "<i>/cartesian_pose_state" interfaces carrying
// a column-major 4x4 O_T_EE pose.
constexpr char kCartesianPoseStateInterfaceName[] = "cartesian_pose_state";
constexpr std::size_t kCartesianPoseSize = 16;

// Force/torque sensor frame suffix and the six interface names a name-constructed
// semantic_components::ForceTorqueSensor requests, in its fixed order (force x/y/z,
// torque x/y/z). Verified against controller_interface/semantic_components/
// force_torque_sensor.hpp. The full interface prefix is prefix_ + robot_type_ + suffix
// (e.g. "fr3_tcp"), matching gravity_compensation_example_controller's
// arm_prefix_ + robot_type_ + "_tcp".
constexpr char kForceTorqueSensorSuffix[] = "_tcp";
constexpr std::array<const char*, 6> kForceTorqueInterfaceNames{"force.x",  "force.y",  "force.z",
                                                                "torque.x", "torque.y", "torque.z"};

// A column-major 4x4 identity transform, used for the flange/EE/stiffness frames of
// the synthetic RobotState (the simulation does not model a tool).
constexpr std::array<double, 16> kIdentityPose{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

// Low-pass cutoff for the external-torque residual, mirroring the real robot's
// tau_ext_hat_filtered. 5 Hz tames finite-difference ddq noise while keeping
// contact transients; alpha = dt / (dt + 1 / (2*pi*fc)) per cycle.
constexpr double kExternalTorqueCutoffHz = 5.0;
constexpr double kGravityAcceleration = 9.81;

// Robot types that describe a multi-arm platform rather than a single 7-DOF arm.
// franka_description ships an xacro for these (so the model path exists), but they
// cannot be built into a single franka::Model. Their model-build failure is an
// expected, benign degradation - logged as a WARN, not an ERROR.
auto isMultiArmRobotType(const std::string& robot_type) -> bool {
  return robot_type == "fr3_duo" || robot_type.find("duo") != std::string::npos;
}

// Generates a clean, un-prefixed robot URDF (links link0..link8, joints joint1..7)
// from franka_description's xacro. libfranka's franka::Model hardcodes the last arm
// link name "link8", so it rejects the prefixed simulation URDF (e.g. "fr3_link8").
// The model description is therefore decoupled from the ros2_control joint names,
// exactly as on real hardware: the prefixed simulation URDF stays in use for
// gz_ros2_control joint binding and for the gravity model, while franka::Model is
// built from this un-prefixed description. Generation is hand=false gazebo=false so
// the result is the bare arm kinematics/dynamics with no gripper or world joint.
auto modelXacroPath(const std::string& robot_type) -> std::string {
  const std::string package_share =
      ament_index_cpp::get_package_share_directory("franka_description");
  return package_share + "/robots/" + robot_type + "/" + robot_type + ".urdf.xacro";
}

auto generateModelUrdf(const std::string& robot_type) -> std::string {
  // robot_type is a trusted URDF-sourced hardware parameter, but it is interpolated
  // into the popen shell command below. Reject anything outside [A-Za-z0-9_] so a stray
  // quote cannot break out of the single-quoted xacro path (defence in depth; the value
  // is also a franka_description directory name, so this never rejects a valid robot).
  const bool is_safe = !robot_type.empty() &&
                       std::all_of(robot_type.begin(), robot_type.end(), [](unsigned char ch) {
                         return std::isalnum(ch) != 0 || ch == '_';
                       });
  if (!is_safe) {
    throw std::runtime_error("Refusing to generate model URDF for unsafe robot_type '" +
                             robot_type + "'");
  }

  const std::string xacro_path = modelXacroPath(robot_type);
  const std::string command =
      "xacro '" + xacro_path + "' no_prefix:=true hand:=false gazebo:=false";

  FILE* pipe = popen(command.c_str(), "r");  // NOLINT(cert-env33-c): fixed command, trusted inputs
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to start xacro for '" + xacro_path + "'");
  }

  std::string urdf;
  try {
    std::array<char, 4096> buffer{};
    while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
      urdf += buffer.data();
    }
  } catch (...) {
    pclose(pipe);
    throw;
  }

  const int wait_status = pclose(pipe);
  const int exit_code = WIFEXITED(wait_status) ? WEXITSTATUS(wait_status) : -1;
  if (wait_status != 0 || urdf.empty()) {
    throw std::runtime_error("xacro failed (exit " + std::to_string(exit_code) +
                             ") generating the model URDF from '" + xacro_path + "'");
  }
  return urdf;
}
}  // namespace

auto FrankaGazeboHardwareInterface::initSim(rclcpp::Node::SharedPtr& model_nh,
                                            std::map<std::string, sim::Entity>& joints,
                                            const hardware_interface::HardwareInfo& hardware_info,
                                            sim::EntityComponentManager& ecm,
                                            unsigned int update_rate) -> bool {
  ecm_ = &ecm;
  model_node_ = model_nh;

  try {
    system_loader_ =
        std::make_shared<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>(
            kSystemPackage, kSystemBaseClass);
    wrapped_system_ = system_loader_->createSharedInstance(kWrappedSystemClass);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(model_nh->get_logger(), "Failed to load wrapped gz_ros2_control system '%s': %s",
                 kWrappedSystemClass, exception.what());
    return false;
  }

  if (!wrapped_system_->initSim(model_nh, joints, hardware_info, ecm, update_rate)) {
    return false;
  }

  try {
    initGravityModel(hardware_info, joints);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(model_nh->get_logger(), "Failed to build the gravity compensation model: %s",
                 exception.what());
    return false;
  }

  try {
    initRobotModel(hardware_info);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(model_nh->get_logger(), "Failed to build the franka robot model: %s",
                 exception.what());
    return false;
  }
  return true;
}

auto FrankaGazeboHardwareInterface::initGravityModel(
    const hardware_interface::HardwareInfo& hardware_info,
    const std::map<std::string, sim::Entity>& joints) -> void {
  std::set<std::string> effort_joint_names;
  for (const auto& joint : hardware_info.joints) {
    for (const auto& command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        effort_joint_names.insert(joint.name);
      }
    }
  }

  if (effort_joint_names.empty()) {
    return;
  }

  std::set<std::string> simulated_joint_names;
  for (const auto& [joint_name, entity] : joints) {
    simulated_joint_names.insert(joint_name);
  }

  gravity_model_.build(hardware_info.original_xml, simulated_joint_names, effort_joint_names);

  for (const auto& configuration_joint : gravity_model_.configurationJoints()) {
    configuration_entities_.push_back(joints.at(configuration_joint.joint_name));
  }
  for (const auto& effort_joint : gravity_model_.effortJoints()) {
    effort_entities_.push_back(joints.at(effort_joint.joint_name));
  }

  // Cache each arm joint's rotation axis while the JointAxis ECM component is still
  // populated (the wrapped system reads it here too, at joint registration). At run
  // time the same lookup returns null, so estimateExternalWrench() relies on these
  // cached axes to project the JointTransmittedWrench - exactly the projection the
  // wrapped system uses for its effort state interface.
  const std::size_t arm_joint_count =
      std::min<std::size_t>(kArmJointCount, configuration_entities_.size());
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    const auto* axis = ecm_->Component<sim::components::JointAxis>(configuration_entities_[i]);
    if (axis == nullptr) {
      continue;
    }
    const auto& axis_xyz = axis->Data().Xyz();
    joint_axes_[i] = {axis_xyz[0], axis_xyz[1], axis_xyz[2]};
  }

  // Pre-allocate the per-cycle buffers once, so write() never touches the heap.
  joint_positions_.assign(configuration_entities_.size(), 0.0);
  gravity_torques_.assign(effort_entities_.size(), 0.0);
}

auto FrankaGazeboHardwareInterface::initRobotModel(
    const hardware_interface::HardwareInfo& hardware_info) -> void {
  robot_type_ = hardware_info.hardware_parameters.at(kRobotTypeParam);
  const auto prefix_iterator = hardware_info.hardware_parameters.find(kPrefixParam);
  prefix_ = (prefix_iterator != hardware_info.hardware_parameters.end()) ? prefix_iterator->second
                                                                         : std::string{};

  // The robot_model / robot_state / force-torque export is a single-arm (fr3-type)
  // feature: it needs a <robot_type>.urdf.xacro from franka_description that yields a
  // single 7-DOF franka::Model. franka::Model also requires the un-prefixed link names
  // it hardcodes (link0..link8), so it is built from a separately generated clean URDF
  // rather than the prefixed simulation URDF in hardware_info.original_xml (which stays
  // in use for gz_ros2_control joint binding). libfranka builds the Pinocchio model
  // fully offline; no robot needed.
  //
  // Three degradation cases are deliberately split:
  //   * A multi-arm robot_type (e.g. fr3_duo): franka_description ships its xacro, so
  //     the model path exists, but it cannot be built into a single 7-DOF franka::Model.
  //     This is an expected, benign degradation: the decorator falls back to gravity
  //     compensation + plain GazeboSimSystem passthrough and exports no model/robot_state/
  //     force-torque interfaces. Logged as a calm WARN, not an error.
  //   * No <robot_type>.urdf.xacro at all: the decorator degrades the same way. Also a WARN.
  //   * The xacro IS present for a single-arm robot but generation or the franka::Model
  //     build fails - or franka_description is not even on the share path. Those are
  //     genuine misconfigurations that silently strip the model interfaces and make
  //     model_example_controller fail to activate later with a misleading "interface not
  //     found". They are logged as ERROR so the real cause surfaces here.
  std::string xacro_path;
  try {
    xacro_path = modelXacroPath(robot_type_);
  } catch (const std::exception& exception) {
    // franka_description is not on the share path at all - an environment error, not the
    // benign dual-arm case.
    model_available_ = false;
    franka_model_.reset();
    hw_model_.reset();
    hw_model_ptr_ = nullptr;
    RCLCPP_ERROR(model_node_->get_logger(),
                 "Cannot locate franka_description to build the model for robot_type '%s' (%s) - "
                 "robot_model/robot_state/force-torque interfaces will not be exported.",
                 robot_type_.c_str(), exception.what());
    return;
  }

  const bool xacro_present = std::filesystem::exists(xacro_path);
  try {
    const std::string model_urdf = generateModelUrdf(robot_type_);
    franka_model_ = std::make_unique<franka::Model>(model_urdf);
    hw_model_ = std::make_unique<franka_hardware::Model>(franka_model_.get());
    hw_model_ptr_ = hw_model_.get();
    model_available_ = true;
  } catch (const std::exception& exception) {
    model_available_ = false;
    franka_model_.reset();
    hw_model_.reset();
    hw_model_ptr_ = nullptr;
    if (xacro_present && isMultiArmRobotType(robot_type_)) {
      RCLCPP_WARN(model_node_->get_logger(),
                  "Multi-arm robot_type '%s' is not supported by this single-arm Gazebo hardware "
                  "interface (%s) - robot_model/robot_state/force-torque interfaces will not be "
                  "exported; gravity compensation and joint control are unaffected.",
                  robot_type_.c_str(), exception.what());
    } else if (xacro_present) {
      RCLCPP_ERROR(model_node_->get_logger(),
                   "Model URDF for robot_type '%s' exists at '%s' but the model build failed "
                   "(%s) - robot_model/robot_state/force-torque interfaces will not be exported; "
                   "model-based controllers will fail to activate.",
                   robot_type_.c_str(), xacro_path.c_str(), exception.what());
    } else {
      RCLCPP_WARN(model_node_->get_logger(),
                  "No single-arm model URDF for robot_type '%s' (%s) - robot_model/robot_state/"
                  "force-torque interfaces will not be exported; gravity compensation and joint "
                  "control are unaffected.",
                  robot_type_.c_str(), exception.what());
    }
  }
}

auto FrankaGazeboHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
    -> CallbackReturn {
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Both initializations are intentional: the base SystemInterface::on_init stores
  // the HardwareInfo this decorator needs, while wrapped_system_->on_init runs the
  // stock gz_ros2_control setup that actually drives the simulated joints.
  return wrapped_system_->on_init(system_info);
}

auto FrankaGazeboHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_configure(previous_state);
}

auto FrankaGazeboHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_activate(previous_state);
}

auto FrankaGazeboHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    -> CallbackReturn {
  return wrapped_system_->on_deactivate(previous_state);
}

auto FrankaGazeboHardwareInterface::export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> {
  auto state_interfaces = wrapped_system_->export_state_interfaces();

  // The model/robot_state/force-torque interfaces are a single-arm feature. When no
  // franka::Model could be built for this robot_type (e.g. the dual-arm mobile platform),
  // only the wrapped system's interfaces are exported - exactly the pre-feature behavior.
  if (!model_available_) {
    return state_interfaces;
  }

  // Mirror franka_hardware::FrankaHardwareInterface::export_state_interfaces(): the
  // semantic component reads a pointer smuggled over a double* StateInterface.
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      prefix_ + robot_type_, kRobotStateInterfaceName,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &robot_state_box_ptr_)));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      prefix_ + robot_type_, kRobotModelInterfaceName,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_model_ptr_)));

  // Force/torque sensor state interfaces requested by a name-constructed
  // semantic_components::ForceTorqueSensor (e.g. gravity_compensation_example_controller).
  // Real franka_hardware does not export these, so this is a simulation-specific
  // addition; the values carry the estimated external wrench in the stiffness frame.
  const std::string force_torque_sensor_name = prefix_ + robot_type_ + kForceTorqueSensorSuffix;
  for (std::size_t i = 0; i < kForceTorqueInterfaceNames.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        force_torque_sensor_name, kForceTorqueInterfaceNames[i], &tcp_wrench_[i]));
  }

  // Cartesian pose state interfaces claimed by FrankaCartesianPoseInterface, named
  // "<prefix><i>/cartesian_pose_state" exactly as franka_hardware exports them so the
  // Cartesian pose/impedance controllers claim them identically. The 16 values carry the
  // column-major 4x4 synthetic O_T_EE, refreshed each read() in updateRobotState().
  for (std::size_t i = 0; i < kCartesianPoseSize; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix_ + std::to_string(i), kCartesianPoseStateInterfaceName, &cartesian_pose_state_[i]));
  }

  return state_interfaces;
}

auto FrankaGazeboHardwareInterface::export_command_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
  return wrapped_system_->export_command_interfaces();
}

auto FrankaGazeboHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) -> hardware_interface::return_type {
  return wrapped_system_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

auto FrankaGazeboHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    -> hardware_interface::return_type {
  const auto result = wrapped_system_->read(time, period);
  if (result != hardware_interface::return_type::OK) {
    return result;
  }

  // A bad cycle must not throw into the Gazebo system-update loop; the model just
  // keeps the previous RobotState for this step.
  try {
    updateRobotState(time, period);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR_THROTTLE(model_node_->get_logger(), *model_node_->get_clock(), 1000,
                          "Robot state update skipped this cycle: %s", exception.what());
  }

  return result;
}

auto FrankaGazeboHardwareInterface::updateRobotState(const rclcpp::Time& time,
                                                     const rclcpp::Duration& period) -> void {
  if (hw_model_ptr_ == nullptr) {
    return;
  }

  // Two simulation conventions are baked into this synthetic RobotState:
  //   1. "desired = measured": with no motion-generator command stream in
  //      simulation, every desired/commanded mirror (q_d, dq_d, O_T_EE_d/_c,
  //      O_dP_EE_d/_c, elbow_d/_c) is set equal to the measured value, and the
  //      remaining command derivatives (ddq_d, O_ddP_EE_c, delbow_c, ddelbow_c)
  //      are left at zero. tau_J_d is the *desired* torque without gravity; with
  //      no command it stays zero (a default-constructed Errors is all-false).
  //   2. The external wrench is *estimated*, not zero: a measured-torque residual
  //      tau_ext = tau_measured - gravity - coriolis - M*ddq is mapped to the EE
  //      via the Jacobian pseudo-inverse. It is only as faithful as the SDF/URDF
  //      inertials agree with franka::Model, and ddq is finite-differenced (noisy
  //      under fast motion); a 5 Hz low-pass keeps parity with the real robot's
  //      filtered estimate. Quasi-static contact is reproduced well; expect a small
  //      non-contact baseline from model mismatch and inertial residue.
  // Everything else is either measured from the Gazebo ECM (q, dq), computed from
  // the offline model (tau_J, O_T_EE, cartesian velocity, elbow) or a bare-arm
  // constant (identity tool frames, zero load/inertia).
  //
  // Unlike write()'s gravity path (which runs from pre-allocated buffers), this read
  // path is NOT heap-free: the five franka::Model calls below (pose/zeroJacobian/mass/
  // gravity/coriolis) allocate Eigen temporaries internally. That is acceptable - the
  // Gazebo system-update loop is not hard real-time. The one workspace we control, the
  // Jacobian-transpose least-squares solver in estimateExternalWrench(), is hoisted to a
  // member and reused so this path does not also allocate its decomposition every cycle.
  franka::RobotState robot_state;

  // No tool is modelled: identity flange/EE/stiffness frames, zero inertial load.
  robot_state.F_T_EE = kIdentityPose;
  robot_state.F_T_NE = kIdentityPose;
  robot_state.NE_T_EE = kIdentityPose;
  robot_state.EE_T_K = kIdentityPose;

  // Measured joint position/velocity from the Gazebo ECM.
  const std::size_t arm_joint_count =
      std::min<std::size_t>(kArmJointCount, configuration_entities_.size());
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    const auto* position =
        ecm_->Component<sim::components::JointPosition>(configuration_entities_[i]);
    const auto* velocity =
        ecm_->Component<sim::components::JointVelocity>(configuration_entities_[i]);
    robot_state.q[i] =
        (position != nullptr && !position->Data().empty()) ? position->Data()[0] : 0.0;
    robot_state.dq[i] =
        (velocity != nullptr && !velocity->Data().empty()) ? velocity->Data()[0] : 0.0;
  }

  // Base translational acceleration: the resting-base gravity vector. franka::Model's
  // gravity/coriolis read this as the gravity load, so it must be set before the inverse
  // dynamics below — otherwise gravity(q) collapses to zero.
  robot_state.O_ddP_O = {0.0, 0.0, -kGravityAcceleration};

  // Link-side joint torque tau_J from the model's inverse dynamics
  // (gravity + Coriolis at the current q, dq). This matches the measured link-side
  // torque of a gravity-loaded arm and stays clean of finite-diff noise. The raw
  // measured (transmitted) torque is read separately in estimateExternalWrench()
  // for the contact residual.
  const std::array<double, 7> gravity_torque = franka_model_->gravity(robot_state);
  const std::array<double, 7> coriolis_torque = franka_model_->coriolis(robot_state);
  for (std::size_t i = 0; i < robot_state.tau_J.size(); ++i) {
    robot_state.tau_J[i] = gravity_torque[i] + coriolis_torque[i];
  }

  // No joint elasticity is modelled, so the motor side equals the link side.
  robot_state.theta = robot_state.q;
  robot_state.dtheta = robot_state.dq;

  // Cartesian end-effector pose and velocity from the model.
  robot_state.O_T_EE = franka_model_->pose(franka::Frame::kEndEffector, robot_state.q,
                                           robot_state.F_T_EE, robot_state.EE_T_K);
  const std::array<double, 42> zero_jacobian = franka_model_->zeroJacobian(
      franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  std::array<double, 6> cartesian_velocity{};
  for (std::size_t row = 0; row < cartesian_velocity.size(); ++row) {
    for (std::size_t column = 0; column < robot_state.dq.size(); ++column) {
      // zeroJacobian is a 6x7 matrix in column-major order: element(row, column).
      cartesian_velocity[row] +=
          zero_jacobian[row + cartesian_velocity.size() * column] * robot_state.dq[column];
    }
  }

  // libfranka elbow: {position of joint 3, flip sign of joint 4}.
  robot_state.elbow = {robot_state.q[2], robot_state.q[3] >= 0.0 ? 1.0 : -1.0};

  // "desired = measured" convention (see the comment block above).
  robot_state.q_d = robot_state.q;
  robot_state.dq_d = robot_state.dq;
  robot_state.O_T_EE_d = robot_state.O_T_EE;
  robot_state.O_T_EE_c = robot_state.O_T_EE;
  robot_state.O_dP_EE_d = cartesian_velocity;
  robot_state.O_dP_EE_c = cartesian_velocity;
  robot_state.elbow_d = robot_state.elbow;
  robot_state.elbow_c = robot_state.elbow;

  estimateExternalWrench(robot_state, gravity_torque, coriolis_torque, zero_jacobian, period);

  // Static simulation defaults: a healthy robot ready to move, full command
  // success, monotonic timestamp from the simulation clock. Inertias/load,
  // contacts/collisions and error flags keep their zero/false defaults.
  robot_state.control_command_success_rate = 1.0;
  robot_state.robot_mode = franka::RobotMode::kMove;
  constexpr int64_t kNanosecondsPerMillisecond = 1000000;
  robot_state.time =
      franka::Duration(static_cast<uint64_t>(time.nanoseconds() / kNanosecondsPerMillisecond));

  // Refresh the exported cartesian_pose_state interfaces from the synthetic pose.
  cartesian_pose_state_ = robot_state.O_T_EE;

  robot_state_box_.set(robot_state);
}

auto FrankaGazeboHardwareInterface::estimateExternalWrench(
    franka::RobotState& robot_state,
    const std::array<double, 7>& gravity_torque,
    const std::array<double, 7>& coriolis_torque,
    const std::array<double, 42>& zero_jacobian,
    const rclcpp::Duration& period) -> void {
  const std::size_t arm_joint_count =
      std::min<std::size_t>(kArmJointCount, configuration_entities_.size());

  // Measured link-side torque in libfranka's tau_J convention. DART's
  // JointTransmittedWrench torque projected onto the cached joint axis is the same
  // value the wrapped system exposes as the effort state interface, and already
  // matches the load-balancing torque the model's gravity(q) returns at rest, so the
  // residual tau_measured - gravity cancels with no external contact.
  std::array<double, 7> tau_measured{};
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    const auto* wrench =
        ecm_->Component<sim::components::JointTransmittedWrench>(configuration_entities_[i]);
    if (wrench == nullptr) {
      continue;
    }
    const auto& torque = wrench->Data().torque();
    const auto& axis = joint_axes_[i];
    tau_measured[i] = torque.x() * axis[0] + torque.y() * axis[1] + torque.z() * axis[2];
  }

  // ddq by finite-difference of dq; skipped on the first cycle and when dt<=0 so
  // the inertial term M*ddq is dropped (treated as zero) for those cycles.
  std::array<double, 7> ddq{};
  const double dt = period.seconds();
  if (prev_dq_valid_ && dt > 0.0) {
    for (std::size_t i = 0; i < arm_joint_count; ++i) {
      ddq[i] = (robot_state.dq[i] - prev_dq_[i]) / dt;
    }
  }
  prev_dq_ = robot_state.dq;
  prev_dq_valid_ = true;

  const std::array<double, 49> mass_matrix = franka_model_->mass(robot_state);

  // Residual: tau_ext = tau_measured - gravity - coriolis - M*ddq. tau_measured is
  // already in libfranka's tau_J sign (JointTransmittedWrench projected on the axis),
  // matching the model gravity/coriolis convention, so at rest it cancels and only a
  // contact torque survives. mass_matrix is the full 7x7 inertia (column-major), so its
  // stride stays kArmJointCount even when fewer joints are simulated.
  std::array<double, 7> tau_ext{};
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    double inertial = 0.0;
    for (std::size_t j = 0; j < arm_joint_count; ++j) {
      inertial += mass_matrix[i + kArmJointCount * j] * ddq[j];  // column-major M
    }
    tau_ext[i] = tau_measured[i] - gravity_torque[i] - coriolis_torque[i] - inertial;
  }

  // First-order low-pass, mirroring tau_ext_hat_filtered.
  const double time_constant = 1.0 / (2.0 * M_PI * kExternalTorqueCutoffHz);
  const double alpha = (dt > 0.0) ? dt / (dt + time_constant) : 0.0;
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    tau_ext_filtered_[i] += alpha * (tau_ext[i] - tau_ext_filtered_[i]);
  }

  // Map the filtered residual to a base-frame wrench via the Jacobian-transpose
  // pseudo-inverse: least-squares solution of J^T * F = tau_ext. The reported external
  // wrench uses the reaction-sign convention: it opposes the applied external force, so a
  // contact pushing the flange +x reads a measured external force of -x. At rest the
  // residual is ~0, so the rest baseline is unaffected.
  //
  // wrench_solver_ is a member: .compute() reuses its decomposition storage every cycle
  // instead of allocating a fresh CompleteOrthogonalDecomposition workspace per call.
  const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(zero_jacobian.data());
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau(tau_ext_filtered_.data());
  jacobian_transpose_.noalias() = jacobian.transpose();
  wrench_solver_.compute(jacobian_transpose_);
  external_wrench_.noalias() = wrench_solver_.solve(tau);
  for (std::size_t i = 0; i < arm_joint_count; ++i) {
    robot_state.tau_ext_hat_filtered[i] = -tau_ext_filtered_[i];
  }

  // external_wrench_ is the reported external wrench in the base frame O (O_F_ext_hat_K),
  // using the reaction-sign convention: a push in +x is reported as a measured external
  // force of -x. K_F_ext_hat_K expresses the same wrench in the end-effector / stiffness
  // frame K. With an identity tool K coincides with the flange EE, but the flange
  // orientation differs from the base orientation at a non-trivial arm pose, so force and
  // torque must each be rotated into K by O_R_EE^T. O_R_EE is the rotation block of the
  // synthetic O_T_EE (column-major 4x4). tcp_wrench_ carries the same stiffness-frame
  // wrench, since the TCP force/torque interfaces represent the wrench at the TCP.
  const Eigen::Map<const Eigen::Matrix<double, 4, 4>> o_T_ee(robot_state.O_T_EE.data());
  const Eigen::Matrix3d o_R_ee = o_T_ee.topLeftCorner<3, 3>();
  const Eigen::Vector3d force_stiffness = o_R_ee.transpose() * external_wrench_.head<3>();
  const Eigen::Vector3d torque_stiffness = o_R_ee.transpose() * external_wrench_.tail<3>();
  for (std::size_t i = 0; i < 3; ++i) {
    const auto index = static_cast<int>(i);
    robot_state.O_F_ext_hat_K[i] = external_wrench_[index];
    robot_state.O_F_ext_hat_K[i + 3] = external_wrench_[index + 3];
    robot_state.K_F_ext_hat_K[i] = force_stiffness[index];
    robot_state.K_F_ext_hat_K[i + 3] = torque_stiffness[index];
    tcp_wrench_[i] = force_stiffness[index];
    tcp_wrench_[i + 3] = torque_stiffness[index];
  }
}

auto FrankaGazeboHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    -> hardware_interface::return_type {
  const auto result = wrapped_system_->write(time, period);
  if (result != hardware_interface::return_type::OK || effort_entities_.empty()) {
    return result;
  }

  for (std::size_t i = 0; i < configuration_entities_.size(); ++i) {
    const auto* position =
        ecm_->Component<sim::components::JointPosition>(configuration_entities_[i]);
    joint_positions_[i] =
        (position != nullptr && !position->Data().empty()) ? position->Data()[0] : 0.0;
  }

  // The gravity computation indexes fixed-size buffers and calls into pinocchio.
  // A single bad cycle must degrade to "no compensation added this step" rather
  // than throwing into the Gazebo system-update loop and tearing down the server.
  try {
    gravity_model_.computeGravityTorque(joint_positions_, gravity_torques_);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR_THROTTLE(model_node_->get_logger(), *model_node_->get_clock(), 1000,
                          "Gravity compensation skipped this cycle: %s", exception.what());
    return result;
  }

  for (std::size_t i = 0; i < effort_entities_.size(); ++i) {
    auto* effort_command = ecm_->Component<sim::components::JointForceCmd>(effort_entities_[i]);
    // JointForceCmd exists only once an effort controller has claimed the interface; a
    // nullptr/empty here means no claimant this cycle. In that case gz_ros2_control's own
    // velocity-zero hold keeps the joint up, so dropping the gravity torque is safe.
    if (effort_command != nullptr && !effort_command->Data().empty()) {
      effort_command->Data()[0] += gravity_torques_[i];
    }
  }

  return result;
}

}  // namespace franka_gazebo_hardware

PLUGINLIB_EXPORT_CLASS(franka_gazebo_hardware::FrankaGazeboHardwareInterface,
                       gz_ros2_control::GazeboSimSystemInterface)
