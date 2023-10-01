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

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadFrankaRobotStateBroadcaster, load_controller) {
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(std::make_unique<hardware_interface::ResourceManager>(
                                               ros2_control_test_assets::minimal_robot_urdf),
                                           executor, "test_controller_manager");

  auto controller =
      cm.load_controller("test_franka_robot_state_broadcaster",
                         "franka_robot_state_broadcaster/FrankaRobotStateBroadcaster");
  ASSERT_NE(controller.get(), nullptr);
  rclcpp::shutdown();
}
