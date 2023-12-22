// Copyright 2023 Franka Robotics GmbH.
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

#include "gtest/gtest.h"

#include "franka_example_controllers/move_to_start_example_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;

class MoveToStartExampleControllerTest : public ::testing::Test {
 public:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

  void SetUpController();

 protected:
  std::unique_ptr<franka_example_controllers::MoveToStartExampleController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                                                 "joint5", "joint6", "joint7"};
  std::vector<double> joint_commands_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // The joint goal pose for the move to start controller
  std::vector<double> joint_q_state_ = {0, -0.785398, 0, -2.35619, 0, 1.5708, 0.785398};
  std::vector<double> joint_dq_state_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> K_gains_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> D_gains_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  CommandInterface joint_1_pos_cmd_{joint_names_[0], HW_IF_EFFORT, &joint_commands_[0]};
  CommandInterface joint_2_pos_cmd_{joint_names_[1], HW_IF_EFFORT, &joint_commands_[1]};
  CommandInterface joint_3_pos_cmd_{joint_names_[2], HW_IF_EFFORT, &joint_commands_[2]};
  CommandInterface joint_4_pos_cmd_{joint_names_[3], HW_IF_EFFORT, &joint_commands_[3]};
  CommandInterface joint_5_pos_cmd_{joint_names_[4], HW_IF_EFFORT, &joint_commands_[4]};
  CommandInterface joint_6_pos_cmd_{joint_names_[5], HW_IF_EFFORT, &joint_commands_[5]};
  CommandInterface joint_7_pos_cmd_{joint_names_[6], HW_IF_EFFORT, &joint_commands_[6]};

  // Position state interfaces
  StateInterface joint_1_pos_state_{joint_names_[0], HW_IF_POSITION, &joint_q_state_[0]};
  StateInterface joint_2_pos_state_{joint_names_[1], HW_IF_POSITION, &joint_q_state_[1]};
  StateInterface joint_3_pos_state_{joint_names_[2], HW_IF_POSITION, &joint_q_state_[2]};
  StateInterface joint_4_pos_state_{joint_names_[3], HW_IF_POSITION, &joint_q_state_[3]};
  StateInterface joint_5_pos_state_{joint_names_[4], HW_IF_POSITION, &joint_q_state_[4]};
  StateInterface joint_6_pos_state_{joint_names_[5], HW_IF_POSITION, &joint_q_state_[5]};
  StateInterface joint_7_pos_state_{joint_names_[6], HW_IF_POSITION, &joint_q_state_[6]};

  // Velocity state interfaces
  StateInterface joint_1_vel_state_{joint_names_[0], HW_IF_VELOCITY, &joint_dq_state_[0]};
  StateInterface joint_2_vel_state_{joint_names_[1], HW_IF_VELOCITY, &joint_dq_state_[1]};
  StateInterface joint_3_vel_state_{joint_names_[2], HW_IF_VELOCITY, &joint_dq_state_[2]};
  StateInterface joint_4_vel_state_{joint_names_[3], HW_IF_VELOCITY, &joint_dq_state_[3]};
  StateInterface joint_5_vel_state_{joint_names_[4], HW_IF_VELOCITY, &joint_dq_state_[4]};
  StateInterface joint_6_vel_state_{joint_names_[5], HW_IF_VELOCITY, &joint_dq_state_[5]};
  StateInterface joint_7_vel_state_{joint_names_[6], HW_IF_VELOCITY, &joint_dq_state_[6]};
};
