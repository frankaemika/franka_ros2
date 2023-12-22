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

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_move_to_start_example_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

const double k_EPS = 1e-5;

void MoveToStartExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void MoveToStartExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void MoveToStartExampleControllerTest::SetUp() {
  controller_ = std::make_unique<franka_example_controllers::MoveToStartExampleController>();
}

void MoveToStartExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
}

void MoveToStartExampleControllerTest::SetUpController() {
  const auto result = controller_->init("test_move_to_start_example");
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<LoanedCommandInterface> command_ifs;
  std::vector<LoanedStateInterface> state_ifs;

  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_2_pos_cmd_);
  command_ifs.emplace_back(joint_3_pos_cmd_);
  command_ifs.emplace_back(joint_4_pos_cmd_);
  command_ifs.emplace_back(joint_5_pos_cmd_);
  command_ifs.emplace_back(joint_6_pos_cmd_);
  command_ifs.emplace_back(joint_7_pos_cmd_);

  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  state_ifs.emplace_back(joint_2_pos_state_);
  state_ifs.emplace_back(joint_2_vel_state_);
  state_ifs.emplace_back(joint_3_pos_state_);
  state_ifs.emplace_back(joint_3_vel_state_);
  state_ifs.emplace_back(joint_4_pos_state_);
  state_ifs.emplace_back(joint_4_vel_state_);
  state_ifs.emplace_back(joint_5_pos_state_);
  state_ifs.emplace_back(joint_5_vel_state_);
  state_ifs.emplace_back(joint_6_pos_state_);
  state_ifs.emplace_back(joint_6_vel_state_);
  state_ifs.emplace_back(joint_7_pos_state_);
  state_ifs.emplace_back(joint_7_vel_state_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(MoveToStartExampleControllerTest, controller_gains_not_set_failure) {
  SetUpController();

  // Failure due to not set K, D gains
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", std::vector<double>()});

  // Failure due empty k gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_damping_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", std::vector<double>()});

  // Failure due to empty d gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, correct_controller_gains_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_activate_expect_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_update_expect_ok) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_1_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_2_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_3_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_4_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_5_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_6_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_7_pos_cmd_.get_value(), 0.0, k_EPS);
}
