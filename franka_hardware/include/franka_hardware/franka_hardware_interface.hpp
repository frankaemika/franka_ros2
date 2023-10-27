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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/franka_executor.hpp"
#include "franka_hardware/franka_param_service_server.hpp"
#include "franka_hardware/robot.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware {

class FrankaHardwareInterface : public hardware_interface::SystemInterface {
 public:
  explicit FrankaHardwareInterface(std::shared_ptr<Robot> robot);
  FrankaHardwareInterface() = default;
  virtual ~FrankaHardwareInterface() = default;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  static const size_t kNumberOfJoints = 7;

 private:
  std::shared_ptr<Robot> robot_;
  std::shared_ptr<FrankaParamServiceServer> node_;
  std::shared_ptr<FrankaExecutor> executor_;

  // Initialize joint position commands in the first pass
  bool first_pass{true};

  std::array<double, kNumberOfJoints> hw_commands_{0, 0, 0, 0, 0, 0, 0};

  // Robot joint states
  std::array<double, kNumberOfJoints> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_efforts_{0, 0, 0, 0, 0, 0, 0};

  /**
   * Desired Cartesian velocity with respect to the o-frame
   * "base frame O" with (vx, vy, vz)\ in [m/s] and
   * (wx, wy, wz) in [rad/s].
   */
  std::array<std::string, 6> hw_cartesian_velocities_names_{"vx", "vy", "vz", "wx", "wy", "wz"};
  std::array<double, 6> hw_cartesian_velocities_{0, 0, 0, 0, 0, 0};

  /**
   * Elbow configuration.
   *
   * The values of the array are:
   *  - elbow[0]: Position of the 3rd joint in \f$[rad]\f$.
   *  - elbow[1]: Flip direction of the elbow (4th joint):
   *    - +1 if \f$q_4 > \alpha\f$
   *    - 0 if \f$q_4 == \alpha \f$
   *    - -1 if \f$q_4 < \alpha \f$
   *    .
   *    with \f$\alpha = -0.467002423653011\f$ \f$rad\f$
   */
  std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};
  std::array<double, 2> hw_elbow_command_{0, 0};

  const std::string k_HW_IF_CARTESIAN_VELOCITY = "cartesian_velocity";
  const std::string k_HW_IF_ELBOW_COMMAND = "elbow_command";
  const std::string k_joint_velocity_interface = "/velocity";

  franka::RobotState hw_franka_robot_state_;
  franka::RobotState* hw_franka_robot_state_addr_ = &hw_franka_robot_state_;
  Model* hw_franka_model_ptr_ = nullptr;

  bool effort_interface_claimed_ = false;
  bool effort_interface_running_ = false;

  bool velocity_joint_interface_claimed_ = false;
  bool velocity_joint_interface_running_ = false;

  bool velocity_cartesian_interface_claimed_ = false;
  bool velocity_cartesian_interface_running_ = false;

  bool elbow_command_interface_claimed_ = false;
  bool elbow_command_interface_running_ = false;

  static rclcpp::Logger getLogger();

  const std::string k_robot_name{"panda"};
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
};
}  // namespace franka_hardware