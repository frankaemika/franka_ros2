#include <gmock/gmock.h>

#include "controller_interface/controller_interface.hpp"
#include "franka_state_broadcaster/franka_state_broadcaster.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace franka_state_broadcaster;

class TestFrankaStateBroadcaster : public ::testing::Test {
 protected:
  void SetUp() override {
    broadcaster_ = std::make_unique<FrankaStateBroadcaster>();
    broadcaster_->init("test_broadcaster");
  }

  std::unique_ptr<FrankaStateBroadcaster> broadcaster_;
};

TEST_F(TestFrankaStateBroadcaster, TestInitReturnSuccess) {
  EXPECT_EQ(broadcaster_->on_init(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaStateBroadcaster, TestConfigureReturnSuccess) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaStateBroadcaster, TestActivateReturnSuccess) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaStateBroadcaster, TestDeactivateReturnSuccess) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaStateBroadcaster, TestUpdateWithoutFrankaStateInterfaceReturnsError) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::ERROR);
}

TEST_F(TestFrankaStateBroadcaster, TestUpdateWithFrankaStateInterfaceReturnsSuccess) {
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