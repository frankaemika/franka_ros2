// Copyright (c) 2023 Franka Emika GmbH
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

#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

class FrankaExecutor : public rclcpp::executors::MultiThreadedExecutor {
 public:
  // Create an instance and start the internal thread
  FrankaExecutor();
  FrankaExecutor(const FrankaExecutor&) = delete;
  FrankaExecutor(FrankaExecutor&&) = delete;

  FrankaExecutor& operator=(const FrankaExecutor&) = delete;
  FrankaExecutor& operator=(FrankaExecutor&&) = delete;

  // Stops the internal executor and joins with the internal thread
  ~FrankaExecutor() override;

 private:
  std::thread executor_spin_;

  // Executor thread starts spining the multithreadedExecutor
  void run();

  // Cancel any spinning ROS executor
  void shutdown();
};
}  // namespace franka_hardware
