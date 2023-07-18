
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

#include "franka_state_test.hpp"

#include <memory>
#include <string>
#include <vector>

void FrankaStateTest::TearDown() {
  franka_state_friend_.reset(nullptr);
}

void FrankaStateTest::SetUp() {
  full_interface_names_.reserve(size_);
  full_interface_names_.emplace_back(robot_name_ + "/" + franka_state_interface_name_);
  franka_state_friend_ =
      std::make_unique<FrankaStateTestFriend>(robot_name_ + "/" + franka_state_interface_name_);

  std::vector<std::string> interface_names = franka_state_friend_->get_state_interface_names();

  robot_state_.q = joint_angles_;
  robot_state_.q_d = joint_velocities_;
  robot_state_.robot_mode = robot_mode_;

  hardware_interface::StateInterface franka_hw_state{
      robot_name_, franka_state_interface_name_, reinterpret_cast<double*>(&robot_state_address_)};
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  temp_state_interfaces.reserve(size_);

  temp_state_interfaces.emplace_back(franka_hw_state);
  franka_state_friend_->assign_loaned_state_interfaces(temp_state_interfaces);
  ASSERT_TRUE(franka_state_friend_->get_values_as_message(franka_robot_state_msg_));
}

TEST_F(FrankaStateTest, validate_state_names_and_size) {
  ASSERT_EQ(franka_state_friend_->name_, robot_name_ + "/" + franka_state_interface_name_);

  ASSERT_EQ(franka_state_friend_->interface_names_.size(), size_);
  ASSERT_EQ(franka_state_friend_->state_interfaces_.capacity(), size_);

  ASSERT_TRUE(std::equal(franka_state_friend_->interface_names_.begin(),
                         franka_state_friend_->interface_names_.end(),
                         full_interface_names_.begin(), full_interface_names_.end()));
  ASSERT_EQ(franka_state_friend_->state_interfaces_.size(), size_);
  franka_state_friend_->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend_->state_interfaces_.size(), 0u);
}

TEST_F(FrankaStateTest, robot_state_ptr_uncasted_correctly) {
  ASSERT_EQ(franka_state_friend_->robot_state_ptr_, robot_state_address_);
  franka_state_friend_->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend_->state_interfaces_.size(), 0u);
}

TEST_F(FrankaStateTest,
       given_franka_semantic_state_initialized_when_message_returned_expect_correct_values) {
  ASSERT_EQ(joint_angles_, franka_robot_state_msg_.q);
  ASSERT_EQ(joint_velocities_, franka_robot_state_msg_.q_d);
  ASSERT_EQ(franka_msgs::msg::FrankaState::ROBOT_MODE_USER_STOPPED,
            franka_robot_state_msg_.robot_mode);
  franka_state_friend_->release_interfaces();
  // validate the count of state_interfaces_
  ASSERT_EQ(franka_state_friend_->state_interfaces_.size(), 0u);
}