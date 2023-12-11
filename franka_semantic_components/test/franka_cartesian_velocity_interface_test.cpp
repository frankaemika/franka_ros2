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

#include "franka_cartesian_velocity_interface_test.hpp"
#include "../src/translation_utils.hpp"

#include <memory>
#include <string>
#include <vector>

void FrankaCartesianVelocityTest::TearDown() {
  franka_cartesian_command_friend.reset(nullptr);
}

void FrankaCartesianVelocityTest::setUpInterfaces(bool elbow_activate) {
  franka_cartesian_command_friend =
      std::make_unique<FrankaCartesianVelocityTestFriend>(elbow_activate);
  setUpHWStateInterfaces(elbow_activate);
  setUpHWCommandInterfaces(elbow_activate);
}

void FrankaCartesianVelocityTest::setUpHWStateInterfaces(bool elbow_activate) {
  temp_state_interfaces.clear();

  if (elbow_activate) {
    for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
      elbow_state_interfaces_container.push_back(
          std::make_shared<hardware_interface::StateInterface>(hardware_interface::StateInterface{
              hw_elbow_command_names_[i], elbow_initial_state_interface_name_,
              &initial_elbow_state_.at(i)}));
    }

    for (auto& elbow_state_interface : elbow_state_interfaces_container) {
      temp_state_interfaces.emplace_back(*elbow_state_interface.get());
    }

    franka_cartesian_command_friend->assign_loaned_state_interfaces(temp_state_interfaces);
  }
}

void FrankaCartesianVelocityTest::setUpHWCommandInterfaces(bool elbow_activate) {
  temp_command_interfaces.clear();

  for (auto i = 0U; i < hw_cartesian_velocities_names_.size(); i++) {
    velocity_command_interfaces_container.push_back(
        std::make_shared<hardware_interface::CommandInterface>(hardware_interface::CommandInterface{
            hw_cartesian_velocities_names_[i], cartesian_velocity_command_interface_name_,
            &hw_cartesian_velocities_.at(i)}));
  }

  if (elbow_activate) {
    for (auto i = 0U; i < hw_elbow_command_names_.size(); i++) {
      elbow_command_interfaces_container.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              hardware_interface::CommandInterface{hw_elbow_command_names_[i],
                                                   elbow_command_interface_name_,
                                                   &hw_elbow_command_.at(i)}));
    }
    for (auto& elbow_command_interface : elbow_command_interfaces_container) {
      temp_command_interfaces.emplace_back(*elbow_command_interface.get());
    }
  }

  for (auto& velocity_command_interface : velocity_command_interfaces_container) {
    temp_command_interfaces.emplace_back(*velocity_command_interface.get());
  }

  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);
}

TEST_F(FrankaCartesianVelocityTest,
       given_correct_interfaces_set_velocity_with_elbow_command_expect_successful) {
  setUpInterfaces(true);
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);
  std::array<double, 2> new_elbow_command{0.0, 2.0};

  auto success = franka_cartesian_command_friend->setCommand(
      cartesian_linear_velocity, cartesian_angular_velocity, new_elbow_command);

  ASSERT_TRUE(success);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities);
  ASSERT_EQ(hw_elbow_command_, new_elbow_command);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianVelocityTest,
       given_correct_interfaces_when_get_elbow_command_values_called_expect_successful) {
  setUpInterfaces(true);
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);
  std::array<double, 2> new_elbow_command{0.0, 2.0};

  auto success_set_command = franka_cartesian_command_friend->setCommand(
      cartesian_linear_velocity, cartesian_angular_velocity, new_elbow_command);

  ASSERT_TRUE(success_set_command);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities);
  ASSERT_EQ(hw_elbow_command_, new_elbow_command);

  std::array<double, 2> elbow_configuration;
  elbow_configuration = franka_cartesian_command_friend->getCommandedElbowConfiguration();

  ASSERT_EQ(new_elbow_command, elbow_configuration);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianVelocityTest,
       given_elbow_is_not_activated_when_elbow_command_get_value_is_called_expect_throw) {
  setUpInterfaces(false);
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);

  auto success_set_command = franka_cartesian_command_friend->setCommand(
      cartesian_linear_velocity, cartesian_angular_velocity);

  ASSERT_TRUE(success_set_command);

  ASSERT_EQ(hw_cartesian_velocities_, new_hw_cartesian_velocities);

  ASSERT_THROW(franka_cartesian_command_friend->getCommandedElbowConfiguration(),
               std::runtime_error);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianVelocityTest,
    given_elbow_and_cartesian_velocity_claimed_when_set_velocity_command_without_elbow_expect_failure) {
  setUpInterfaces(true);
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);

  auto success = franka_cartesian_command_friend->setCommand(cartesian_linear_velocity,
                                                             cartesian_angular_velocity);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(
    FrankaCartesianVelocityTest,
    given_only_cartesian_velocity_claimed_without_elbow_when_set_velocity_command_cartesian_vel_expect_successful) {
  setUpInterfaces(false);
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);
  std::array<double, 2> default_zero_elbow_command{0.0, 0.0};

  auto success = franka_cartesian_command_friend->setCommand(
      cartesian_linear_velocity, cartesian_angular_velocity, default_zero_elbow_command);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianVelocityTest,
       given_incorrect_command_interfaces_set_velocity_expect_unsuccesful) {
  franka_cartesian_command_friend = std::make_unique<FrankaCartesianVelocityTestFriend>(true);
  hardware_interface::CommandInterface dummy_command_interface{"dummy", "dummy_cartesian_velocity",
                                                               &hw_elbow_command_.at(0)};
  std::array<double, 6> new_hw_cartesian_velocities{0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  Eigen::Vector3d cartesian_linear_velocity(new_hw_cartesian_velocities[0],
                                            new_hw_cartesian_velocities[1],
                                            new_hw_cartesian_velocities[2]);
  Eigen::Vector3d cartesian_angular_velocity(new_hw_cartesian_velocities[3],
                                             new_hw_cartesian_velocities[4],
                                             new_hw_cartesian_velocities[5]);

  temp_command_interfaces.emplace_back(dummy_command_interface);
  franka_cartesian_command_friend->assign_loaned_command_interfaces(temp_command_interfaces);

  auto success = franka_cartesian_command_friend->setCommand(cartesian_linear_velocity,
                                                             cartesian_angular_velocity);

  ASSERT_FALSE(success);

  franka_cartesian_command_friend->release_interfaces();
}

TEST_F(FrankaCartesianVelocityTest,
       given_correct_interface_when_initial_elbow_state_requested_expect_correct) {
  setUpInterfaces(true);

  auto received_elbow_state = franka_cartesian_command_friend->getInitialElbowConfiguration();
  ASSERT_EQ(received_elbow_state, initial_elbow_state_);
}

TEST_F(FrankaCartesianVelocityTest,
       given_correct_interface_when_initial_elbow_state_requested_expect_throw) {
  setUpInterfaces(false);

  ASSERT_THROW(franka_cartesian_command_friend->getInitialElbowConfiguration(), std::runtime_error);
}
