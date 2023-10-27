#include "franka_hardware_cartesian_command_interface_test.hpp"

using ::testing::_;

TEST_F(FrankaCartesianCommandInterfaceTest, cartesian_command_interface_number_is_setup_correctly) {
  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto command_interfaces = franka_hardware_interface.export_command_interfaces();

  // cartesian velocity plus elbow command interfaces
  const auto number_cartesian_velocity_command_interface =
      k_hw_cartesian_velocities_names.size() + k_hw_elbow_command_names.size();

  EXPECT_EQ(command_interfaces.size(), number_cartesian_velocity_command_interface);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    given_correct_number_of_start_cartesian_velocity_interface_when_prepare_command_mode_interface_is_called_expect_success) {
  auto [command_interfaces, command_interface_name] = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  std::vector<std::string> stop_interface = {};
  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  start_interface.clear();
  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    given_correct_number_of_start_cartesian_velocity_and_elbow_command_interface_when_prepare_command_mode_interface_is_called_expect_success) {
  auto [command_interfaces, command_interface_name] = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  std::vector<std::string> stop_interface = {};
  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  start_interface.clear();

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }
  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_start_interface_number_expect_throw) {
  auto [command_interfaces, command_interface_name] = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }

  start_interface = {"fr3_joint1/" + command_interface_name};

  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_stop_interface_number_expect_throw) {
  auto [command_interfaces, command_interface_name] = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  stop_interface = {"fr3_joint1/" + command_interface_name};

  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_read_is_not_called_when_write_is_called_expec_robot_writeOnce_is_not_called) {
  auto mock_robot = std::make_shared<MockRobot>();

  EXPECT_CALL(*mock_robot, stopRobot());
  EXPECT_CALL(*mock_robot, initializeCartesianVelocityInterface());

  EXPECT_CALL(*mock_robot, writeOnce(_, _)).Times(0);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  EXPECT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_cartesian_velocity_and_elbow_set_when_read_and_write_called_expect_success) {
  auto mock_robot = std::make_shared<MockRobot>();

  EXPECT_CALL(*mock_robot, stopRobot());
  EXPECT_CALL(*mock_robot, initializeCartesianVelocityInterface());

  EXPECT_CALL(*mock_robot, readOnce()).Times(1);
  EXPECT_CALL(*mock_robot, writeOnce(_, _)).Times(1);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  EXPECT_EQ(franka_hardware_interface.read(time, duration), hardware_interface::return_type::OK);
  EXPECT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_cartesian_velocity_is_claimed_when_perform_mode_switch_is_called_expect_success) {
  auto mock_robot = std::make_shared<MockRobot>();

  EXPECT_CALL(*mock_robot, stopRobot());
  EXPECT_CALL(*mock_robot, initializeCartesianVelocityInterface());

  // Only cartesian velocity command
  EXPECT_CALL(*mock_robot, writeOnce(std::array<double, 6>{}));

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  EXPECT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_only_elbow_command_is_claimed_withoud_cartesian_velocity_when_perform_mode_switch_is_called_expect_error) {
  auto mock_robot = std::make_shared<MockRobot>();

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  EXPECT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  EXPECT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::ERROR);
}