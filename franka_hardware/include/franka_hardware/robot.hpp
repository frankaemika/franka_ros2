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

#include <future>
#include <iostream>

#include <franka/robot.h>

namespace franka_hardware {

class Robot {
 public:
  explicit Robot(const std::string& robot_ip);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  void initializeTorqueControl();
  void stopTorqueControl();
  franka::RobotState read();

  void write(const std::array<double, 7>& efforts);  // NOLINT(readability-magic-numbers)

 private:
  std::unique_ptr<std::thread> control_thread_;
  std::unique_ptr<franka::Robot> robot_;
  std::mutex read_mutex_;
  std::mutex write_mutex_;
  bool finish_ = false;
  franka::RobotState current_state_;
  std::array<double, 7> tau_command_{};  // NOLINT(readability-magic-numbers)
 public:
  virtual ~Robot();
};
}  // namespace franka_hardware
