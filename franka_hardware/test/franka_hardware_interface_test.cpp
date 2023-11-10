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
#include <exception>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include <franka_hardware/robot.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "franka/exception.h"

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include "test_utils.hpp"

const std::string k_position_controller{"position"};
const std::string k_velocity_controller{"velocity"};
const std::string k_effort_controller{"effort"};
const std::string k_joint_name{"joint"};
const size_t k_number_of_joints{7};
const double k_EPS{1e-5};

using namespace std::chrono_literals;

class FrankaHardwareInterfaceTest : public ::testing::TestWithParam<std::string> {};

auto createHardwareInfo() -> hardware_interface::HardwareInfo {
  hardware_interface::HardwareInfo info;
  std::unordered_map<std::string, std::string> hw_params;
  hw_params["robot_ip"] = "dummy_ip";

  info.hardware_parameters = hw_params;
  hardware_interface::InterfaceInfo command_effort_interface, command_velocity_interface,
      command_position_interface, effort_state_interface, position_state_interface,
      velocity_state_interface;

  effort_state_interface.name = hardware_interface::HW_IF_EFFORT;
  position_state_interface.name = hardware_interface::HW_IF_POSITION;
  velocity_state_interface.name = hardware_interface::HW_IF_VELOCITY;

  std::vector<hardware_interface::InterfaceInfo> state_interfaces = {
      position_state_interface, velocity_state_interface, effort_state_interface};

  command_effort_interface.name = k_effort_controller;
  command_velocity_interface.name = k_velocity_controller;
  command_position_interface.name = k_position_controller;

  for (auto i = 0U; i < k_number_of_joints; i++) {
    hardware_interface::ComponentInfo joint;

    joint.name = k_joint_name + std::to_string(i + 1);

    joint.command_interfaces.push_back(command_effort_interface);
    joint.command_interfaces.push_back(command_velocity_interface);
    joint.command_interfaces.push_back(command_position_interface);

    joint.state_interfaces = state_interfaces;

    info.joints.push_back(joint);
  }

  return info;
}

template <typename service_client_type,
          typename service_request_type,
          typename service_response_type>
void get_param_service_response(
    std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
    const std::string& service_name,
    service_response_type& response) {
  auto mock_robot = std::make_shared<MockRobot>();
  mock_function(mock_robot);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);

  auto node = rclcpp::Node::make_shared("test_node");

  auto client = node->create_client<service_client_type>(service_name);
  auto request = std::make_shared<service_request_type>();

  RCLCPP_INFO(node->get_logger(), "created request");

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }

  // Response will be checked on the calling function. Because we can't use ASSERT in a non-void
  // function
  response = *result.get();
}

TEST_F(FrankaHardwareInterfaceTest, when_on_init_called_expect_success) {
  auto mock_robot = std::make_shared<MockRobot>();
  const hardware_interface::HardwareInfo info = createHardwareInfo();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);
  auto return_type = franka_hardware_interface.on_init(info);

  ASSERT_EQ(return_type,
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(FrankaHardwareInterfaceTest,
       given_that_the_robot_interfaces_set_when_read_called_return_ok) {
  franka::RobotState robot_state;
  MockModel mock_model;
  MockModel* model_address = &mock_model;

  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*mock_robot, getModel()).WillOnce(testing::Return(model_address));

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  auto time = rclcpp::Time(0, 0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_return_zero_values_and_correct_interface_names) {
  franka::RobotState robot_state;
  const size_t state_interface_size = 23;  // position, effort and velocity states for
                                           // every joint + robot state and model
  auto mock_robot = std::make_shared<MockRobot>();
  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*mock_robot, getModel()).WillOnce(testing::Return(model_address));
  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = franka_hardware_interface.export_state_interfaces();
  size_t joint_index = 0;

  // Get all the states except the last two reserved for robot state
  for (size_t i = 0; i < states.size() - 2; i++) {
    if (i % 3 == 0) {
      joint_index++;
    }
    const std::string joint_name = k_joint_name + std::to_string(joint_index);
    if (i % 3 == 0) {
      ASSERT_EQ(states[i].get_name(), joint_name + "/" + k_position_controller);
    } else if (i % 3 == 1) {
      ASSERT_EQ(states[i].get_name(), joint_name + "/" + k_velocity_controller);
    } else {
      ASSERT_EQ(states[i].get_name(), joint_name + "/" + k_effort_controller);
    }
    ASSERT_EQ(states[i].get_value(), 0.0);
  }

  ASSERT_EQ(states.size(), state_interface_size);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_interface_robot_model_interface_exists) {
  franka::RobotState robot_state;
  const size_t state_interface_size = 23;  // position, effort and velocity states for
                                           // every joint + robot state and model
  auto mock_robot = std::make_shared<MockRobot>();

  MockModel mock_model;
  MockModel* model_address = &mock_model;
  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));

  EXPECT_CALL(*mock_robot, getModel()).WillOnce(testing::Return(model_address));
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = franka_hardware_interface.export_state_interfaces();
  ASSERT_EQ(states[state_interface_size - 1].get_name(),
            "panda/robot_model");  // Last state interface is the robot model state
  EXPECT_NEAR(states[state_interface_size - 1].get_value(),
              *reinterpret_cast<double*>(&model_address),
              k_EPS);  // testing that the casted mock_model ptr
                       // is correctly pushed to state interface
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_that_the_robot_interfaces_are_set_when_call_export_state_interface_robot_state_interface_exists) {
  const size_t state_interface_size = 23;  // position, effort and velocity states for
                                           // every joint + robot state and model
  auto mock_robot = std::make_shared<MockRobot>();

  franka::RobotState robot_state;
  franka::RobotState* robot_state_address = &robot_state;

  MockModel mock_model;
  MockModel* model_address = &mock_model;
  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*mock_robot, getModel()).WillOnce(testing::Return(model_address));
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  auto return_type = franka_hardware_interface.read(time, duration);
  ASSERT_EQ(return_type, hardware_interface::return_type::OK);
  auto states = franka_hardware_interface.export_state_interfaces();
  ASSERT_EQ(states[state_interface_size - 2].get_name(),
            "panda/robot_state");  // Last state interface is the robot model state
  EXPECT_NEAR(states[state_interface_size - 2].get_value(),
              *reinterpret_cast<double*>(&robot_state_address),
              k_EPS);  // testing that the casted robot state ptr
                       // is correctly pushed to state interface
}

TEST_P(FrankaHardwareInterfaceTest,
       when_prepare_command_mode_interface_for_stop_effort_interfaces_expect_ok) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }
  std::vector<std::string> start_interface = {};
  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_start_interface_number_expect_throw) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> stop_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }
  std::vector<std::string> start_interface = {"fr3_joint1/effort"};
  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST_P(FrankaHardwareInterfaceTest,
       when_prepare_command_mode_interface_for_start_effort_interfaces_expect_ok) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaHardwareInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_stop_interface_number_expect_throw) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }

  start_interface = {"fr3_joint1/effort"};

  EXPECT_THROW(
      franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
      std::invalid_argument);
}

TEST_P(FrankaHardwareInterfaceTest, when_write_called_expect_ok) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, writeOnce(std::array<double, 7>{}));

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  if (command_interface == k_position_controller) {
    ASSERT_EQ(franka_hardware_interface.read(time, duration), hardware_interface::return_type::OK);
  }
  ASSERT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST_F(FrankaHardwareInterfaceTest, when_write_called_with_inifite_command_expect_error) {
  auto mock_robot = std::make_shared<MockRobot>();
  franka::RobotState robot_state;
  robot_state.q_d = std::array<double, 7>{std::numeric_limits<double>::infinity()};

  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*mock_robot, writeOnce(std::array<double, 7>{})).Times(0);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_position_controller);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(franka_hardware_interface.read(time, duration), hardware_interface::return_type::OK);
  ASSERT_EQ(franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::ERROR);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_position_joint_command_interface_initialized_if_write_called_without_read_expect_write_once_not_to_called) {
  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, writeOnce(std::array<double, 7>{})).Times(0);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(std::move(mock_robot));

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + k_position_controller);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(franka_hardware_interface.write(time, duration), hardware_interface::return_type::OK);
}

TEST_F(FrankaHardwareInterfaceTest, when_on_activate_called_expect_success) {
  franka::RobotState robot_state;

  MockModel mock_model;
  MockModel* model_address = &mock_model;

  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*mock_robot, getModel()).WillOnce(testing::Return(model_address));

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  ASSERT_EQ(franka_hardware_interface.on_activate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(FrankaHardwareInterfaceTest, when_on_deactivate_called_expect_success) {
  franka::RobotState robot_state;

  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, stopRobot());

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  ASSERT_EQ(franka_hardware_interface.on_deactivate(rclcpp_lifecycle::State()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_P(FrankaHardwareInterfaceTest,
       given_start_effort_interface_prepared_when_perform_command_mode_switch_called_expect_ok) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();

  if (command_interface == k_effort_controller) {
    EXPECT_CALL(*mock_robot, initializeTorqueInterface());
  } else if (command_interface == k_velocity_controller) {
    EXPECT_CALL(*mock_robot, initializeJointVelocityInterface());
  } else if (command_interface == k_position_controller) {
    EXPECT_CALL(*mock_robot, initializeJointPositionInterface());
  }

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(FrankaHardwareInterfaceTest,
       given_that_effort_control_started_perform_command_mode_switch_stop_expect_ok) {
  std::string command_interface = GetParam();

  auto mock_robot = std::make_shared<MockRobot>();
  EXPECT_CALL(*mock_robot, stopRobot()).Times(2);

  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    start_interface.push_back(joint_name + "/" + command_interface);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  for (size_t i = 0; i < hardware_info.joints.size(); i++) {
    const std::string joint_name = k_joint_name + std::to_string(i);
    stop_interface.push_back(joint_name + "/" + command_interface);
  }

  start_interface.clear();

  ASSERT_EQ(franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);

  ASSERT_EQ(franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_joint_stiffness_service_called_expect_robot_set_joint_stiffness_to_be_called) {
  auto expect_call_set_joint_stiffness = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setJointStiffness(testing::_)).Times(1);
  };
  franka_msgs::srv::SetJointStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetJointStiffness,
                             franka_msgs::srv::SetJointStiffness::Request,
                             franka_msgs::srv::SetJointStiffness::Response>(
      expect_call_set_joint_stiffness, "service_server/set_joint_stiffness", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_joint_cartesian_service_called_expect_robot_set_joint_cartesian_to_be_called) {
  auto expect_call_set_cartesian_stiffness = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setCartesianStiffness(testing::_)).Times(1);
  };
  franka_msgs::srv::SetCartesianStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetCartesianStiffness,
                             franka_msgs::srv::SetCartesianStiffness::Request,
                             franka_msgs::srv::SetCartesianStiffness::Response>(
      expect_call_set_cartesian_stiffness, "service_server/set_cartesian_stiffness", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_load_service_called_expect_robot_set_load_to_be_called) {
  auto expect_call_set_load = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setLoad(testing::_)).Times(1);
  };
  franka_msgs::srv::SetLoad::Response response;
  get_param_service_response<franka_msgs::srv::SetLoad, franka_msgs::srv::SetLoad::Request,
                             franka_msgs::srv::SetLoad::Response>(
      expect_call_set_load, "service_server/set_load", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_tcp_frame_service_called_expect_robot_set_tcp_frame_to_be_called) {
  auto expect_call_set_tcp_frame = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setTCPFrame(testing::_)).Times(1);
  };
  franka_msgs::srv::SetTCPFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetTCPFrame, franka_msgs::srv::SetTCPFrame::Request,
                             franka_msgs::srv::SetTCPFrame::Response>(
      expect_call_set_tcp_frame, "service_server/set_tcp_frame", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_stiffness_frame_service_called_expect_robot_set_stiffness_frame_to_be_called) {
  auto expect_call_set_stiffness_frame = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setStiffnessFrame(testing::_)).Times(1);
  };
  franka_msgs::srv::SetStiffnessFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetStiffnessFrame,
                             franka_msgs::srv::SetStiffnessFrame::Request,
                             franka_msgs::srv::SetStiffnessFrame::Response>(
      expect_call_set_stiffness_frame, "service_server/set_stiffness_frame", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_force_torque_collision_behavior_service_called_expect_same_function_in_robot_class_to_be_called) {
  auto expect_call_set_force_torque_collision_behavior =
      [&](std::shared_ptr<MockRobot> mock_robot) {
        EXPECT_CALL(*mock_robot, setForceTorqueCollisionBehavior(testing::_)).Times(1);
      };
  franka_msgs::srv::SetForceTorqueCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetForceTorqueCollisionBehavior,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Request,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Response>(
      expect_call_set_force_torque_collision_behavior,
      "service_server/set_force_torque_collision_behavior", response);

  ASSERT_TRUE(response.success);
}

TEST_F(
    FrankaHardwareInterfaceTest,
    given_param_service_server_setup_when_set_full_collision_behavior_service_called_expect_same_function_in_robot_class_to_be_called) {
  auto expect_call_set_full_collision_behavior = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setFullCollisionBehavior(testing::_)).Times(1);
  };
  franka_msgs::srv::SetFullCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetFullCollisionBehavior,
                             franka_msgs::srv::SetFullCollisionBehavior::Request,
                             franka_msgs::srv::SetFullCollisionBehavior::Response>(
      expect_call_set_full_collision_behavior, "service_server/set_full_collision_behavior",
      response);

  ASSERT_TRUE(response.success);
}

TEST_F(FrankaHardwareInterfaceTest, set_joint_stiffness_throws_error) {
  auto set_joint_stiffness_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setJointStiffness(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetJointStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetJointStiffness,
                             franka_msgs::srv::SetJointStiffness::Request,
                             franka_msgs::srv::SetJointStiffness::Response>(
      set_joint_stiffness_mock_throw, "service_server/set_joint_stiffness", response);

  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_cartesian_stiffness_throws_error) {
  auto set_cartesian_stiffness_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setCartesianStiffness(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetCartesianStiffness::Response response;
  get_param_service_response<franka_msgs::srv::SetCartesianStiffness,
                             franka_msgs::srv::SetCartesianStiffness::Request,
                             franka_msgs::srv::SetCartesianStiffness::Response>(
      set_cartesian_stiffness_mock_throw, "service_server/set_cartesian_stiffness", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_load_throws_error) {
  auto set_load_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setLoad(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetLoad::Response response;
  get_param_service_response<franka_msgs::srv::SetLoad, franka_msgs::srv::SetLoad::Request,
                             franka_msgs::srv::SetLoad::Response>(
      set_load_mock_throw, "service_server/set_load", response);

  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_EE_frame_throws_error) {
  auto set_tcp_frame_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setTCPFrame(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetTCPFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetTCPFrame, franka_msgs::srv::SetTCPFrame::Request,
                             franka_msgs::srv::SetTCPFrame::Response>(
      set_tcp_frame_mock_throw, "service_server/set_tcp_frame", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_K_frame_throws_error) {
  auto set_stiffness_frame_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setStiffnessFrame(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetStiffnessFrame::Response response;
  get_param_service_response<franka_msgs::srv::SetStiffnessFrame,
                             franka_msgs::srv::SetStiffnessFrame::Request,
                             franka_msgs::srv::SetStiffnessFrame::Response>(
      set_stiffness_frame_mock_throw, "service_server/set_stiffness_frame", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_force_torque_collision_behavior_throws_error) {
  auto set_force_torque_collision_behavior_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setForceTorqueCollisionBehavior(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };

  franka_msgs::srv::SetForceTorqueCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetForceTorqueCollisionBehavior,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Request,
                             franka_msgs::srv::SetForceTorqueCollisionBehavior::Response>(
      set_force_torque_collision_behavior_mock_throw,
      "service_server/set_force_torque_collision_behavior", response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

TEST_F(FrankaHardwareInterfaceTest, set_full_collision_behavior_throws_error) {
  auto set_full_collision_behavior_mock_throw = [&](std::shared_ptr<MockRobot> mock_robot) {
    EXPECT_CALL(*mock_robot, setFullCollisionBehavior(testing::_))
        .Times(1)
        .WillRepeatedly(testing::Throw((franka::NetworkException(""))));
  };
  franka_msgs::srv::SetFullCollisionBehavior::Response response;
  get_param_service_response<franka_msgs::srv::SetFullCollisionBehavior,
                             franka_msgs::srv::SetFullCollisionBehavior::Request,
                             franka_msgs::srv::SetFullCollisionBehavior::Response>(
      set_full_collision_behavior_mock_throw, "service_server/set_full_collision_behavior",
      response);
  ASSERT_FALSE(response.success);
  ASSERT_EQ(response.error, "network exception error");
}

int main(int argc, char** argv) {
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

INSTANTIATE_TEST_SUITE_P(FrankaHardwareTests,
                         FrankaHardwareInterfaceTest,
                         ::testing::Values(k_velocity_controller,
                                           k_effort_controller,
                                           k_position_controller));