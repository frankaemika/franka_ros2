#include <gmock/gmock.h>
#include <algorithm>
#include <functional>

#include <franka/active_control_base.h>
#include <franka/robot.h>
#include "franka_hardware/robot.hpp"
#include "test_utils.hpp"

#pragma once

const double k_EPS = 1e-5;

class MockActiveControl : public franka::ActiveControlBase {
 public:
  MOCK_METHOD((std::pair<franka::RobotState, franka::Duration>), readOnce, (), (override));
  MOCK_METHOD(void, writeOnce, (const franka::Torques&), (override));
  MOCK_METHOD(void,
              writeOnce,
              (const franka::JointPositions&, const std::optional<const franka::Torques>&),
              (override));
  MOCK_METHOD(void,
              writeOnce,
              (const franka::JointVelocities&, const std::optional<const franka::Torques>&),
              (override));
  MOCK_METHOD(void,
              writeOnce,
              (const franka::CartesianPose&, const std::optional<const franka::Torques>&),
              (override));
  MOCK_METHOD(void,
              writeOnce,
              (const franka::CartesianVelocities&, const std::optional<const franka::Torques>&),
              (override));
  MOCK_METHOD(void, writeOnce, (const franka::JointPositions&), (override));
  MOCK_METHOD(void, writeOnce, (const franka::JointVelocities&), (override));
  MOCK_METHOD(void, writeOnce, (const franka::CartesianPose&), (override));
  MOCK_METHOD(void, writeOnce, (const franka::CartesianVelocities&), (override));
};

class MockFrankaRobot : public franka::Robot {
 public:
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(std::unique_ptr<franka::ActiveControlBase>, startTorqueControl, (), (override));
  MOCK_METHOD(std::unique_ptr<franka::ActiveControlBase>,
              startJointVelocityControl,
              (const research_interface::robot::Move::ControllerMode&),
              (override));
  MOCK_METHOD(std::unique_ptr<franka::ActiveControlBase>,
              startJointPositionControl,
              (const research_interface::robot::Move::ControllerMode&),
              (override));
  MOCK_METHOD(std::unique_ptr<franka::ActiveControlBase>,
              startCartesianPoseControl,
              (const research_interface::robot::Move::ControllerMode&),
              (override));
  MOCK_METHOD(std::unique_ptr<franka::ActiveControlBase>,
              startCartesianVelocityControl,
              (const research_interface::robot::Move::ControllerMode&),
              (override));
};

namespace franka {

template <typename T>
bool compareWithTolerance(const T& lhs, const T& rhs) {
  return std::equal(lhs.begin(), lhs.end(), rhs.begin(),
                    [](double lhs_element, double rhs_element) {
                      return std::abs(lhs_element - rhs_element) < k_EPS;
                    });
}

bool operator==(const CartesianVelocities& lhs, const CartesianVelocities& rhs) {
  return compareWithTolerance(lhs.O_dP_EE, rhs.O_dP_EE);
}

bool operator==(const JointPositions& lhs, const JointPositions& rhs) {
  return compareWithTolerance(lhs.q, rhs.q);
}

bool operator==(const JointVelocities& lhs, const JointVelocities& rhs) {
  return compareWithTolerance(lhs.dq, rhs.dq);
}

bool operator==(const Torques& lhs, const Torques& rhs) {
  return compareWithTolerance(lhs.tau_J, rhs.tau_J);
}
}  // namespace franka

class FrankaRobotTests : public ::testing::Test {
 protected:
  std::unique_ptr<MockFrankaRobot> mock_libfranka_robot;
  std::unique_ptr<MockModel> mock_model;
  std::unique_ptr<MockActiveControl> mock_active_control;

  template <typename RobotInitFunction, typename ControlType, typename RawControlInputType>
  void testWriteOnce(RobotInitFunction initFunction,
                     std::function<void()> expectCallFunction,
                     const RawControlInputType& control_input,
                     const ControlType& expected_active_control_input) {
    EXPECT_CALL(*mock_active_control, writeOnce(expected_active_control_input));
    expectCallFunction();
    franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

    (robot.*initFunction)();
    robot.writeOnce(control_input);
  }

  void SetUp() override {
    mock_libfranka_robot = std::make_unique<MockFrankaRobot>();
    mock_model = std::make_unique<MockModel>();
    mock_active_control = std::make_unique<MockActiveControl>();
  }
  void TearDown() override {
    mock_libfranka_robot.reset();
    mock_model.reset();
    mock_active_control.reset();
  }
};