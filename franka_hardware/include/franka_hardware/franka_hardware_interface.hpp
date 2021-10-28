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
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <franka_hardware/robot.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

class FrankaHardwareInterface
    : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
 public:
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;
  static const int kNumberOfJoints = 7;

 private:
  std::unique_ptr<Robot> robot_;
  std::array<double, kNumberOfJoints> hw_commands_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_efforts_{0, 0, 0, 0, 0, 0, 0};
  bool effort_interface_claimed_ = false;
  bool effort_interface_running_ = false;
  static rclcpp::Logger getLogger();
};
}  // namespace franka_hardware