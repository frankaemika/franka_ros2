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

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "franka_example_controllers/gravity_compensation_example_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::LoanedCommandInterface;

class TestGravityCompensationExample : public ::testing::Test {
 public:
  static void SetUpTestSuite();
  static void TearDownTestSuite();

  void SetUp();
  void TearDown();

  void SetUpController();

 protected:
  std::unique_ptr<franka_example_controllers::GravityCompensationExampleController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                                                 "joint5", "joint6", "joint7"};
  std::vector<double> joint_commands_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  CommandInterface joint_1_pos_cmd_{joint_names_[0], HW_IF_EFFORT, &joint_commands_[0]};
  CommandInterface joint_2_pos_cmd_{joint_names_[1], HW_IF_EFFORT, &joint_commands_[1]};
  CommandInterface joint_3_pos_cmd_{joint_names_[2], HW_IF_EFFORT, &joint_commands_[2]};
  CommandInterface joint_4_pos_cmd_{joint_names_[3], HW_IF_EFFORT, &joint_commands_[3]};
  CommandInterface joint_5_pos_cmd_{joint_names_[4], HW_IF_EFFORT, &joint_commands_[4]};
  CommandInterface joint_6_pos_cmd_{joint_names_[5], HW_IF_EFFORT, &joint_commands_[5]};
  CommandInterface joint_7_pos_cmd_{joint_names_[6], HW_IF_EFFORT, &joint_commands_[6]};
};

void TestGravityCompensationExample::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void TestGravityCompensationExample::TearDownTestSuite() {
  rclcpp::shutdown();
}

void TestGravityCompensationExample::SetUp() {
  controller_ =
      std::make_unique<franka_example_controllers::GravityCompensationExampleController>();
}

void TestGravityCompensationExample::TearDown() {
  controller_.reset(nullptr);
}

void TestGravityCompensationExample::SetUpController() {
  const auto result = controller_->init("test_gravitiy_compensation_example");
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_2_pos_cmd_);
  command_ifs.emplace_back(joint_3_pos_cmd_);
  command_ifs.emplace_back(joint_4_pos_cmd_);
  command_ifs.emplace_back(joint_5_pos_cmd_);
  command_ifs.emplace_back(joint_6_pos_cmd_);
  command_ifs.emplace_back(joint_7_pos_cmd_);

  controller_->assign_interfaces(std::move(command_ifs), {});
}

TEST_F(TestGravityCompensationExample, JointsParameterNotSet) {
  GTEST_SKIP() << "Skipping joint parameters not set test";
  SetUpController();

  // TODO(baris) configure must fail, 'joints' parameter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestGravityCompensationExample, JointsParameterIsEmpty) {
  GTEST_SKIP() << "Skipping joints parameter is empty test";
  SetUpController();
  controller_->get_node()->set_parameter({"joints", std::vector<std::string>()});

  // Should return ERROR!!
  // TODO(baris) params_.joints can't be empty add a check
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestGravityCompensationExample, given_correct_number_of_joints_configure_returns_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestGravityCompensationExample, given_joints_and_interface_when_update_expect_zero_values) {
  SetUpController();
  controller_->get_node()->set_parameter({"joints", joint_names_});

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  // update successful though no command has been send yet
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);

  // check joint commands are updated to zero torque value
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_2_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_3_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_4_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_5_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_6_pos_cmd_.get_value(), 0.0);
  ASSERT_EQ(joint_7_pos_cmd_.get_value(), 0.0);
}
