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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "gmock/gmock.h"

class FrankaCartesianPoseTestFriend
    : public franka_semantic_components::FrankaCartesianPoseInterface {
 public:
  explicit FrankaCartesianPoseTestFriend(const bool elbow_activate)
      : franka_semantic_components::FrankaCartesianPoseInterface(elbow_activate) {}

  virtual ~FrankaCartesianPoseTestFriend() = default;
};

class FrankaCartesianPoseTest : public ::testing::Test {
 public:
  void constructFrankaCartesianPoseInterface(bool elbow_active);
  void TearDown();

 private:
  void setUpHWCommandInterfaces(bool elbow_active);
  void setUpHWStateInterfaces(bool elbow_active);

 protected:
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>>
      pose_command_interfaces_container, elbow_command_interfaces_container;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> pose_state_interfaces_container,
      elbow_state_interfaces_container;
  std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};

  std::array<double, 16> hw_cartesian_pose_{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  std::array<double, 2> hw_elbow_command_{0.0, 0.0};

  std::array<double, 16> initial_cartesian_pose_{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 2> initial_elbow_state_{1.0, 1.0};

  std::unique_ptr<FrankaCartesianPoseTestFriend> franka_cartesian_command_friend;
  std::vector<hardware_interface::LoanedCommandInterface> temp_command_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  const std::string cartesian_pose_command_interface_name_{"cartesian_pose"};
  const std::string elbow_command_interface_name_{"elbow_command"};
  const std::string cartesian_initial_pose_state_interface_name_{"initial_cartesian_pose"};
  const std::string initial_elbow_state_interface_name_{"initial_elbow_state"};
};
