// Copyright (c) 2021 Franka Emika GmbH
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

#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka_hardware/model.hpp>

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include <rclcpp/logger.hpp>

namespace franka_hardware {

class Robot {
 public:
  /**
   * Connects to the robot. This method can block for up to one minute if the robot is not
   * responding. An exception will be thrown if the connection cannot be established.
   *
   * @param[in] robot_ip IP address or hostname of the robot.
   * @param[im] logger ROS Logger to print eventual warnings.
   */
  explicit Robot(const std::string& robot_ip, const rclcpp::Logger& logger);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  /// Stops the currently running loop and closes the connection with the robot.
  virtual ~Robot();

  /// Starts the active control for torque control
  virtual void initializeTorqueInterface();

  /// Starts the active control for joint velocity control
  virtual void initializeJointVelocityInterface();

  /// Starts the active control for joint velocity control
  virtual void initializeCartesianVelocityInterface();

  /// stops the read continous communication with the connected robot
  virtual void stopRobot();

  /**
   * Get the current robot state
   * @return current robot state.
   */
  virtual franka::RobotState readOnce();

  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  virtual franka_hardware::Model* getModel();

  /**
   * This function will automatically propagate the received hardware active command
   * interface
   * @param[in] joint_hardware_command joint hardware command either efforts or velocities
   */
  virtual void writeOnce(const std::array<double, 7>& joint_hardware_command);

  /**
   * Cartesian velocity command
   * @param[in] cartesian_velocity_command cartesian level velocity command in format
   *  [vx, vy, vz, wx, wy, wz]
   */
  virtual void writeOnce(const std::array<double, 6>& cartesian_velocity_command);

  /**
   * Cartesian velocity command with elbow command
   * @param[in] cartesian_velocity_command cartesian level velocity command in format
   *  [vx, vy, vz, wx, wy, wz]
   * @param[in] elbow_command elbow command representing joint3_position in rad and joint4 sign
   */
  virtual void writeOnce(const std::array<double, 6>& cartesian_velocity_command,
                         const std::array<double, 2>& elbow_command);

  /**
   * Sets the impedance for each joint in the internal controller.
   *
   * User-provided torques are not affected by this setting.
   *
   * @param[in] franka_msgs::srv::SetJointStiffness::Request::SharedPtr requests with JointStiffness
   * values
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req);

  /**
   * Sets the Cartesian stiffness (for x, y, z, roll, pitch, yaw) in the internal
   * controller.
   *
   * The values set using Robot::SetCartesianStiffness are used in the direction of the
   * stiffness frame, which can be set with Robot::setK.
   *
   * Inputs received by the torque controller are not affected by this setting.
   *
   * @param[in] franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr request
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req);

  /**
   * Sets dynamic parameters of a payload.
   *
   * @note
   * This is not for setting end effector parameters, which have to be set in the administrator's
   * interface.
   *
   * @param[in] franka_msgs::srv::SetLoad::Request::SharedPtr request
   *
   * @throw CommandException if the Control reports an error.
   * @throw
   */
  virtual void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req);

  /**
   * Sets the transformation \f$^{NE}T_{EE}\f$ from nominal end effector to end effector frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] franka_msgs::srv::SetTCPFrame::Request::SharedPtr req
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req);

  /**
   * Sets the transformation \f$^{EE}T_K\f$ from end effector frame to stiffness frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr req.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   */
  virtual void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req);

  /**
   * Changes the collision behavior.
   *
   * Set common torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr req
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req);

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr request msg
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req);

 protected:
  Robot() = default;

 private:
  /**
   * Get the current robot state, when the controller is active
   * @return current robot state.
   */
  virtual franka::RobotState readOnceActiveControl();

  /**
   * The robot will use these torques until a different set of torques are commanded.
   * @param[in] efforts torque command for each joint.
   */
  virtual void writeOnceEfforts(const std::array<double, 7>& efforts);

  /**
   * The robot will use these velocities until a different set of velocities are commanded.
   * @param[in] joint_velocities joint velocity command.
   */
  virtual void writeOnceJointVelocities(const std::array<double, 7>& joint_velocities);

  /**
   * @brief Checks if control loop is activated for active control.
   *
   * @return true when active control started either with effort or velocity command.
   * @return false when active control is not started.
   */
  virtual bool isControlLoopActive();

  std::mutex write_mutex_;
  std::mutex control_mutex_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::ActiveControl> active_control_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<Model> franka_hardware_model_;

  std::array<double, 7> last_desired_torque_ = {0, 0, 0, 0, 0, 0, 0};

  bool effort_interface_active_{false};
  bool joint_velocity_interface_active_{false};
  bool cartesian_velocity_interface_active_{false};

  bool velocity_command_rate_limit_active_{false};
  bool cartesian_velocity_command_rate_limit_active_{false};
  bool cartesian_velocity_low_pass_filter_active{false};
  double low_pass_filter_cut_off_freq{1000.0};

  franka::RobotState current_state_;
};
}  // namespace franka_hardware
