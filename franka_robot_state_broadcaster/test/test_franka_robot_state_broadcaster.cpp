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

#include "controller_interface/controller_interface.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace franka_robot_state_broadcaster;

class TestFrankaRobotStateBroadcaster : public ::testing::Test {
 protected:
  void SetUp() override {
    broadcaster_ = std::make_unique<FrankaRobotStateBroadcaster>();
    broadcaster_->init("test_broadcaster");
  }

  std::unique_ptr<FrankaRobotStateBroadcaster> broadcaster_;
};

TEST_F(TestFrankaRobotStateBroadcaster, test_init_return_success) {
  EXPECT_EQ(broadcaster_->on_init(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_configure_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_activate_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_deactivate_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_without_franka_state_interface_returns_error) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::ERROR);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_with_franka_state_returns_success) {
  // Todo(anyone)
  GTEST_SKIP() << "Realtime publisher lock behaviour is not deterministic";
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::OK);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}