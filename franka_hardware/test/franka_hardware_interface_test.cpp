#include <gmock/gmock.h>
#include <exception>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/robot.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

const std::string k_position_controller{"position"};
const std::string k_velocity_controller{"velocity"};
const std::string k_effort_controller{"effort"};
const std::string k_joint_name{"joint"};
const size_t k_number_of_joints{7};

class MockRobot : public franka_hardware::Robot {
 public:
  MOCK_METHOD(void, initializeContinuousReading, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(void, initializeTorqueControl, (), (override));
  MOCK_METHOD(franka::RobotState, read, (), (override));
};

auto createHardwareInfo() -> hardware_interface::HardwareInfo {
  hardware_interface::HardwareInfo info;
  std::unordered_map<std::string, std::string> hw_params;
  hw_params["robot_ip"] = "dummy_ip";

  info.hardware_parameters = hw_params;
  hardware_interface::InterfaceInfo command_interface, effort_state_interface,
      position_state_interface, velocity_state_interface;

  effort_state_interface.name = hardware_interface::HW_IF_EFFORT;
  position_state_interface.name = hardware_interface::HW_IF_POSITION;
  velocity_state_interface.name = hardware_interface::HW_IF_VELOCITY;

  std::vector<hardware_interface::InterfaceInfo> state_interfaces = {
      position_state_interface, velocity_state_interface, effort_state_interface};

  for (auto i = 0U; i < k_number_of_joints; i++) {
    hardware_interface::ComponentInfo joint;

    joint.name = k_joint_name + std::to_string(i + 1);

    command_interface.name = k_effort_controller;

    joint.command_interfaces.push_back(command_interface);
    joint.state_interfaces = state_interfaces;

    info.joints.push_back(joint);
  }

  return info;
}

TEST(FrankaHardwareInterfaceTest, when_on_init_called_expect_success) {
  auto mock_robot = std::make_unique<MockRobot>();
  const hardware_interface::HardwareInfo info = createHardwareInfo();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));
  auto return_type = franka_hardware_interface.on_init(info);

  EXPECT_EQ(return_type,
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST(FrankaHardwareInterfaceTest, given_that_the_robot_interfaces_set_when_read_called_return_ok) {
  franka::RobotState robot_state;

  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, read()).WillOnce(testing::Return(robot_state));
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  auto time = rclcpp::Time(0, 0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  EXPECT_EQ(return_type, hardware_interface::return_type::OK);
}

TEST(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_return_zero_values_and_correct_interface_names) {
  franka::RobotState robot_state;
  const size_t state_interface_size = 21;  // position, effort and velocity states for every joint
  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, read()).WillOnce(testing::Return(robot_state));
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  EXPECT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = franka_hardware_interface.export_state_interfaces();
  size_t joint_index = 0;
  for (size_t i = 0; i < states.size(); i++) {
    if (i % 3 == 0) {
      joint_index++;
    }
    const std::string joint_name = k_joint_name + std::to_string(joint_index);
    if (i % 3 == 0) {
      EXPECT_EQ(states[i].get_name(), joint_name + "/" + k_position_controller);
    } else if (i % 3 == 1) {
      EXPECT_EQ(states[i].get_name(), joint_name + "/" + k_velocity_controller);
    } else {
      EXPECT_EQ(states[i].get_name(), joint_name + "/" + k_effort_controller);
    }
    EXPECT_EQ(states[i].get_value(), 0.0);
  }
  EXPECT_EQ(states.size(), state_interface_size);
}

TEST(FrankaHardwareInterfaceTest,
     when_prepare_command_mode_interface_for_stop_effort_interfaces_expect_ok) {
  auto mock_robot = std::make_unique<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + k_effort_controller);
  }
  std::vector<std::string> start_interface = {};
  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_start_interface_number_expect_throw) {
  auto mock_robot = std::make_unique<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + k_effort_controller);
  }
  std::vector<std::string> start_interface = {"fr3_joint1/effort"};
  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST(FrankaHardwareInterfaceTest,
     when_prepare_command_mode_interface_for_start_effort_interfaces_expect_ok) {
  auto mock_robot = std::make_unique<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_effort_controller);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_stop_interface_number_expect_throw) {
  auto mock_robot = std::make_unique<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_effort_controller);
  }

  std::vector<std::string> stop_interface = {"fr3_joint1/effort"};

  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST(FrankaHardwareIntefaceTest, when_write_called_expect_ok) {
  auto mock_robot = std::make_unique<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  EXPECT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST(FrankaHardwareInterfaceTest, when_on_activate_called_expect_success) {
  franka::RobotState robot_state;

  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, read()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*mock_robot, initializeContinuousReading());

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  EXPECT_EQ(franka_hardware_interface.on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST(FrankaHardwareInterfaceTest, when_on_deactivate_called_expect_success) {
  franka::RobotState robot_state;

  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, stopRobot());

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  EXPECT_EQ(franka_hardware_interface.on_deactivate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST(FrankaHardwareInterfaceTest,
     given_start_effort_interface_prepared_when_perform_comamnd_mode_switch_called_expect_ok) {
  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, stopRobot());
  EXPECT_CALL(*mock_robot, initializeTorqueControl());

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_effort_controller);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST(FrankaHardwareInterfaceTest,
     given_that_effort_control_started_perform_command_mode_switch_stop_expect_ok) {
  auto mock_robot = std::make_unique<MockRobot>();
  EXPECT_CALL(*mock_robot, stopRobot()).Times(2).WillRepeatedly(testing::Return());
  EXPECT_CALL(*mock_robot, initializeTorqueControl());
  EXPECT_CALL(*mock_robot, initializeContinuousReading());

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_effort_controller);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + k_effort_controller);
  }

  start_interface.clear();

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}