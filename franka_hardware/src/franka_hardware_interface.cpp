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

#include <franka_hardware/franka_hardware_interface.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

FrankaHardwareInterface::FrankaHardwareInterface(std::unique_ptr<Robot> robot)
    : robot_{std::move(robot)} {}

std::vector<StateInterface> FrankaHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }
  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_.at(i)));
  }
  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_->initializeContinuousReading();
  hw_commands_.fill(0);
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  const auto kState = robot_->read();
  hw_positions_ = kState.q;
  hw_velocities_ = kState.dq;
  hw_efforts_ = kState.tau_J;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (std::any_of(hw_commands_.begin(), hw_commands_.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }

  robot_->write(hw_commands_);
  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  if (!robot_) {
    std::string robot_ip;
    try {
      robot_ip = info_.hardware_parameters.at("robot_ip");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' ! set");
      return CallbackReturn::ERROR;
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
      robot_ = std::make_unique<Robot>(robot_ip, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could ! connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  }
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (!effort_interface_running_ && effort_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeTorqueControl();
    effort_interface_running_ = true;
  } else if (effort_interface_running_ && !effort_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeContinuousReading();
    effort_interface_running_ = false;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  auto is_effort_interface = [](const std::string& interface) {
    return interface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos;
  };

  int64_t num_stop_effort_interfaces =
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), is_effort_interface);
  if (num_stop_effort_interfaces == kNumberOfJoints) {
    effort_interface_claimed_ = false;
  } else if (num_stop_effort_interfaces != 0) {
    RCLCPP_FATAL(this->getLogger(), "Expected %ld effort interfaces to stop, but got %ld instead.",
                 kNumberOfJoints, num_stop_effort_interfaces);
    std::string error_string = "Invalid number of effort interfaces to stop. Expected ";
    error_string += std::to_string(kNumberOfJoints);
    throw std::invalid_argument(error_string);
  }

  int64_t num_start_effort_interfaces =
      std::count_if(start_interfaces.begin(), start_interfaces.end(), is_effort_interface);
  if (num_start_effort_interfaces == kNumberOfJoints) {
    effort_interface_claimed_ = true;
  } else if (num_start_effort_interfaces != 0) {
    RCLCPP_FATAL(this->getLogger(), "Expected %ld effort interfaces to start, but got %ld instead.",
                 kNumberOfJoints, num_start_effort_interfaces);
    std::string error_string = "Invalid number of effort interfaces to start. Expected ";
    error_string += std::to_string(kNumberOfJoints);
    throw std::invalid_argument(error_string);
  }
  return hardware_interface::return_type::OK;
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)