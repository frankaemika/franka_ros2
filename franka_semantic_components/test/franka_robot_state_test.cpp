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

#include "franka_robot_state_test.hpp"

#include <memory>
#include <string>
#include <vector>

void FrankaRobotStateTest::TearDown() {
  franka_state_friend.reset(nullptr);
}

void FrankaRobotStateTest::SetUp() {
  full_interface_names.reserve(size);
  full_interface_names.emplace_back(robot_name + "/" + franka_state_interface_name);
  franka_state_friend =
      std::make_unique<FrankaRobotStateTestFriend>(robot_name + "/" + franka_state_interface_name);

  std::vector<std::string> interface_names = franka_state_friend->get_state_interface_names();

  robot_state.q = joint_angles;
  robot_state.q_d = joint_velocities;
  robot_state.robot_mode = robot_mode;

  hardware_interface::StateInterface franka_hw_state{
      robot_name, franka_state_interface_name, reinterpret_cast<double*>(&robot_state_address)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  temp_state_interfaces.reserve(size);

  temp_state_interfaces.emplace_back(franka_hw_state);
  franka_state_friend->assign_loaned_state_interfaces(temp_state_interfaces);
  ASSERT_TRUE(franka_state_friend->get_values_as_message(franka_robot_state_msg));
}

TEST_F(FrankaRobotStateTest, validate_state_names_and_size) {
  ASSERT_EQ(franka_state_friend->name_, robot_name + "/" + franka_state_interface_name);

  ASSERT_EQ(franka_state_friend->interface_names_.size(), size);
  ASSERT_EQ(franka_state_friend->state_interfaces_.capacity(), size);

  ASSERT_TRUE(std::equal(franka_state_friend->interface_names_.begin(),
                         franka_state_friend->interface_names_.end(), full_interface_names.begin(),
                         full_interface_names.end()));
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), size);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest, robot_state_ptr_uncasted_correctly) {
  ASSERT_EQ(franka_state_friend->robot_state_ptr, robot_state_address);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}

TEST_F(FrankaRobotStateTest,
       given_franka_semantic_state_initialized_when_message_returned_expect_correct_values) {
  ASSERT_EQ(joint_angles, franka_robot_state_msg.q);
  ASSERT_EQ(joint_velocities, franka_robot_state_msg.q_d);
  ASSERT_EQ(franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED,
            franka_robot_state_msg.robot_mode);
  franka_state_friend->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend->state_interfaces_.size(), 0u);
}