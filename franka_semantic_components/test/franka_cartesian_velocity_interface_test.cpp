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

#include "franka_cartesian_velocity_interface_test.hpp"

#include <memory>
#include <string>
#include <vector>

void FrankaCartesianVelocityTest::TearDown() {
  franka_cartesian_command_friend.reset(nullptr);
}

void FrankaCartesianVelocityTest::setUpHWCommandInterfaces() {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianVelocityTestFriend>();
  temp_command_interfaces.clear();

  for (auto i = 0U; i < hw_cartesian_velocities_names_.size(); i++) {
    velocity_command_interfaces_container.push_back(
        std::make_shared<hardware_interface::CommandInterface>(hardware_interface::CommandInterface{
            hw_cartesian_velocities_names_[i], cartesian_velocity_command_interface_name_,
            &hw_cartesian_velocities_.at(i)}));
  }
  for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
    elbow_command_interfaces_container.push_back(
        std::make_shared<hardware_interface::CommandInterface>(hardware_interface::CommandInterface{
            hw_elbow_command_names_[i], elbow_command_interface_name_, &hw_elbow_command_.at(i)}));
  }

  for (auto& velocity_command_interface : velocity_command_interfaces_container) {
    temp_command_interfaces.emplace_back(*velocity_command_interface.get());
  }

  for (auto& elbow_command_interface : elbow_command_interfaces_container) {
    temp_command_interfaces.emplace_back(*elbow_command_interface.get());
  }

  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);
}

TEST_F(FrankaCartesianVelocityTest,
       given_correct_interfaces_set_velocity_with_elbow_command_expect_successful) {
  setUpHWCommandInterfaces();
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  std::array<double, 2> new_elbow_command{0.0, 2.0};

  auto success =
      franka_cartesian_command_friend->setCommand(new_hw_cartesian_velocities, new_elbow_command);

  ASSERT_TRUE(success);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities);
  ASSERT_EQ(hw_elbow_command_, new_elbow_command);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianVelocityTest,
    given_elbow_and_cartesian_velocity_claimed_when_set_velocity_command_without_elbow_expect_failure) {
  setUpHWCommandInterfaces();
  std::array<double, 6> new_hw_cartesian_velocities_{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  std::array<double, 2> default_zero_elbow_command{0.0, 0.0};

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_velocities_);

  ASSERT_FALSE(success);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities_);
  ASSERT_EQ(hw_elbow_command_, default_zero_elbow_command);

  franka_cartesian_command_friend->release_interfaces();
}

// TODO(baris) Update this such that when franka_cartesian velocity command interface is activate
// it does not activate elbow
TEST_F(
    FrankaCartesianVelocityTest,
    given_only_cartesian_velocity_claimed_without_elbow_when_set_velocity_command_cartesian_vel_expect_successful) {
  setUpHWCommandInterfaces();
  std::array<double, 6> new_hw_cartesian_velocities_{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  std::array<double, 2> default_zero_elbow_command{0.0, 0.0};

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_velocities_);

  ASSERT_TRUE(success);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities_);
  ASSERT_EQ(hw_elbow_command_, default_zero_elbow_command);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianVelocityTest,
       given_incorrect_command_interfaces_set_velocity_expect_unsuccesful) {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianVelocityTestFriend>();
  hardware_interface::CommandInterface dummy_command_interface{"dummy", "dummy_cartesian_velocity",
                                                               &hw_elbow_command_.at(0)};
  std::array<double, 6> new_hw_cartesian_velocities_{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};

  temp_command_interfaces.emplace_back(dummy_command_interface);
  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);

  auto success = franka_cartesian_command_friend->setCommand(new_hw_cartesian_velocities_);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}