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

#include <franka_hardware/robot.hpp>

#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <franka/rate_limiting.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly "
        "recommended! Information about how to set up a real-time kernel can be found here: "
        "https://frankaemika.github.io/docs/"
        "installation_linux.html#setting-up-the-real-time-kernel");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}

void Robot::writeOnce(const std::array<double, 7>& efforts) {
  auto torque_command = franka::Torques(efforts);
  torque_command.tau_J =
      franka::limitRate(franka::kMaxTorqueRate, torque_command.tau_J, last_desired_torque_);
  last_desired_torque_ = torque_command.tau_J;

  active_control_->writeOnce(torque_command);
}

franka::RobotState Robot::readOnce() {
  const auto [robot_state, _] = active_control_->readOnce();
  return robot_state;
}

franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  active_control_.reset();
}

void Robot::initializeReadWriteInterface() {
  active_control_ = robot_->startTorqueControl();
}

Robot::~Robot() {
  stopRobot();
}

}  // namespace franka_hardware
