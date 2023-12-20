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

#include <gmock/gmock.h>
#include <exception>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include <franka_hardware/robot.hpp>

class MockModel : public franka_hardware::Model {};

class MockRobot : public franka_hardware::Robot {
 public:
  MOCK_METHOD(void, initializeJointPositionInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianVelocityInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianPoseInterface, (), (override));
  MOCK_METHOD(void, initializeTorqueInterface, (), (override));
  MOCK_METHOD(void, initializeJointVelocityInterface, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(MockModel*, getModel, (), (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 7>&)efforts), (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 6>&)cartesian_velocity), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::array<double, 6>&)cartesian_velocity,
               (const std::array<double, 2>&)elbow_command),
              (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 16>&)cartesian_pose), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::array<double, 16>&)cartesian_pose,
               (const std::array<double, 2>&)elbow_command),
              (override));
  MOCK_METHOD(void,
              setJointStiffness,
              (const franka_msgs::srv::SetJointStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setCartesianStiffness,
              (const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, setLoad, (const franka_msgs::srv::SetLoad::Request::SharedPtr&), (override));
  MOCK_METHOD(void,
              setTCPFrame,
              (const franka_msgs::srv::SetTCPFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setStiffnessFrame,
              (const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setForceTorqueCollisionBehavior,
              (const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setFullCollisionBehavior,
              (const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&),
              (override));
};
