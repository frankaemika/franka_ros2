#include "franka_robot_test.hpp"

TEST_F(FrankaRobotTests, whenInitializeTorqueInterfaceCalled_thenStartTorqueControlCalled) {
  EXPECT_CALL(*mock_libfranka_robot, startTorqueControl()).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeTorqueInterface();
}

TEST_F(FrankaRobotTests, whenInitializeJointVelocityInterfaceCalled_thenStartJointVelocityControl) {
  EXPECT_CALL(*mock_libfranka_robot, startJointVelocityControl(testing::_)).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeJointVelocityInterface();
}

TEST_F(FrankaRobotTests,
       whenInitializeJointtPositionInterfaceCalled_thenStartJointPositionControl) {
  EXPECT_CALL(*mock_libfranka_robot, startJointPositionControl(testing::_)).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeJointPositionInterface();
}

TEST_F(FrankaRobotTests,
       whenInitializeCartesianVelocityInterfaceCalled_thenStartCartesianVelocityControl) {
  EXPECT_CALL(*mock_libfranka_robot, startCartesianVelocityControl(testing::_)).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianVelocityInterface();
}

TEST_F(FrankaRobotTests,
       givenCartesianVelocityControlIsStarted_whenReadOnceIsCalled_expectCorrectRobotState) {
  franka::RobotState robot_state;
  franka::Duration duration;
  robot_state.q_d = std::array<double, 7>{1, 2, 3, 1, 2, 3, 1};
  auto active_control_read_return_tuple = std::make_pair(robot_state, duration);

  EXPECT_CALL(*mock_active_control, readOnce())
      .WillOnce(testing::Return(active_control_read_return_tuple));
  EXPECT_CALL(*mock_libfranka_robot, startCartesianVelocityControl(testing::_))
      .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianVelocityInterface();
  auto actual_state = robot.readOnce();
  ASSERT_EQ(robot_state.q_d, actual_state.q_d);
}

TEST_F(FrankaRobotTests,
       givenJointControlIsNotStarted_whenReadOnceIsCalled_expectCorrectRobotState) {
  franka::RobotState robot_state;
  robot_state.q_d = std::array<double, 7>{1, 2, 3, 1, 2, 3, 1};

  EXPECT_CALL(*mock_libfranka_robot, readOnce()).WillOnce(testing::Return(robot_state));

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));
  auto actual_state = robot.readOnce();

  ASSERT_EQ(robot_state.q_d, actual_state.q_d);
}

TEST_F(
    FrankaRobotTests,
    givenCartesianVelocityControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::array<double, 6>& cartesian_velocities{1, 0, 0, 0, 0, 0};
  const franka::CartesianVelocities expected_cartesian_velocities(cartesian_velocities);

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startCartesianVelocityControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testWriteOnce<void (franka_hardware::Robot::*)(), franka::CartesianVelocities,
                std::array<double, 6>>(
      &franka_hardware::Robot::initializeCartesianVelocityInterface, expectCallFunction,
      cartesian_velocities, expected_cartesian_velocities);
}

TEST_F(
    FrankaRobotTests,
    givenJointPositionControlIsControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::array<double, 7>& joint_positions{1, 0, 0, 0, 0, 0, 0};
  const franka::JointPositions expected_joint_positions(joint_positions);

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startJointPositionControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testWriteOnce<void (franka_hardware::Robot::*)(), franka::JointPositions, std::array<double, 7>>(
      &franka_hardware::Robot::initializeJointPositionInterface, expectCallFunction,
      joint_positions, expected_joint_positions);
}

TEST_F(
    FrankaRobotTests,
    givenJointVelocityControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::array<double, 7>& joint_velocities{1, 0, 0, 0, 0, 0, 0};
  const franka::JointVelocities expected_joint_velocities(joint_velocities);

  const auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startJointVelocityControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testWriteOnce<void (franka_hardware::Robot::*)(), franka::JointVelocities, std::array<double, 7>>(
      &franka_hardware::Robot::initializeJointVelocityInterface, expectCallFunction,
      joint_velocities, expected_joint_velocities);
}

TEST_F(FrankaRobotTests,
       givenEffortControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::array<double, 7>& joint_torques{1, 0, 0, 0, 0, 0, 0};
  // Torque rate limiter defaulted to active
  const franka::Torques expected_joint_torques{0.999999, 0, 0, 0, 0, 0, 0};

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startTorqueControl())
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testWriteOnce<void (franka_hardware::Robot::*)(), franka::Torques, std::array<double, 7>>(
      &franka_hardware::Robot::initializeTorqueInterface, expectCallFunction, joint_torques,
      expected_joint_torques);
}

TEST_F(FrankaRobotTests,
       givenControlIsNotStart_whenWriteOnceIsCalled_expectRuntimeExceptionToBeThrown) {
  const std::array<double, 7>& joint_torques{1, 0, 0, 0, 0, 0, 0};
  const franka::Torques joint_torques_franka(joint_torques);

  const std::array<double, 6>& cartesian_velocities{1, 0, 0, 0, 0, 0};
  const franka::CartesianVelocities cartesian_franka(cartesian_velocities);

  EXPECT_CALL(*mock_active_control, writeOnce(joint_torques_franka)).Times(0);
  EXPECT_CALL(*mock_active_control, writeOnce(cartesian_franka)).Times(0);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  EXPECT_THROW(robot.writeOnce(joint_torques), std::runtime_error);
  EXPECT_THROW(robot.writeOnce(cartesian_velocities), std::runtime_error);
}
