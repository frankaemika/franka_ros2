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

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip) {
  tau_command_.fill(0.);
  robot_ = std::make_unique<franka::Robot>(robot_ip);
}

void Robot::write(const std::array<double, 7>& efforts) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}

void Robot::stopTorqueControl() {
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    finish_ = true;
  }
  control_thread_->join();
}

void Robot::initializeTorqueControl() {
  const auto kTorqueControl = [this]() {
    robot_->control([this](const franka::RobotState& state, const franka::Duration& /*period*/) {
      {
        std::lock_guard<std::mutex> lock(read_mutex_);
        current_state_ = state;
      }
      std::lock_guard<std::mutex> lock(write_mutex_);
      franka::Torques out(tau_command_);
      out.motion_finished = finish_;
      return out;
    });
  };
  control_thread_ = std::make_unique<std::thread>(kTorqueControl);
}

Robot::~Robot() {
  stopTorqueControl();
}
}  // namespace franka_hardware
