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

#include "franka_hardware/franka_executor.hpp"

namespace franka_hardware {
using namespace std::chrono_literals;

FrankaExecutor::FrankaExecutor() : executor_spin_([this] { run(); }) {
  while (!this->spinning) {
    // Need to wait until the executor starts spinning
    std::this_thread::sleep_for(100ms);
  }
}

FrankaExecutor::~FrankaExecutor() {
  // if the executor still spinning cancel it
  this->shutdown();
  executor_spin_.join();
}

void FrankaExecutor::run() {
  // spin the executor
  spin();
}

void FrankaExecutor::shutdown() {
  if (this->spinning) {
    this->cancel();
  }
}

}  // namespace franka_hardware
