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
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "franka/exception.h"
#include "test_utils.hpp"

class FrankaCartesianCommandInterfaceTest
    : public ::testing::TestWithParam<std::tuple<std::vector<std::string>, std::string>> {
 protected:
  const std::vector<std::string> k_hw_cartesian_pose_names{
      "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};
  const std::vector<std::string> k_hw_cartesian_velocities_names{"vx", "vy", "vz",
                                                                 "wx", "wy", "wz"};
  const std::vector<std::string> k_hw_elbow_command_names{"joint_3_position", "joint_4_sign"};

  const std::string k_joint_name{"joint"};
  const size_t k_number_of_joints{7};

  const std::string k_cartesian_velocity_command_interface_name{"cartesian_velocity"};
  const std::string k_cartesian_pose_command_interface_name{"cartesian_pose"};
  const std::string k_elbow_command_interface_name{"elbow_command"};
};

INSTANTIATE_TEST_SUITE_P(
    FrankaCartesianCommandTest,
    FrankaCartesianCommandInterfaceTest,
    ::testing::Values(std::make_tuple(std::vector<std::string>{"vx", "vy", "vz", "wx", "wy", "wz"},
                                      "cartesian_velocity"),
                      std::make_tuple(std::vector<std::string>{"0", "1", "2", "3", "4", "5", "6",
                                                               "7", "8", "9", "10", "11", "12",
                                                               "13", "14", "15"},
                                      "cartesian_pose"),
                      std::make_tuple(std::vector<std::string>{"joint_3_position", "joint_4_sign"},
                                      "elbow_command")));
