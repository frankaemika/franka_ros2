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

  /// Starts a read / write communication with the connected robot
  virtual void initializeReadWriteInterface();

  /// stops the read / write communication with the connected robot
  virtual void stopRobot();

  /**
   * Get the current robot state in a thread-safe way.
   * @return current robot state.
   */
  virtual franka::RobotState readOnce();

  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  virtual franka_hardware::Model* getModel();

  /**
   * Sends new desired torque commands to the control loop in a thread-safe way.
   * The robot will use these torques until a different set of torques are commanded.
   * @param[in] efforts torque command for each joint.
   */
  virtual void writeOnce(const std::array<double, 7>& efforts);

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
  std::mutex write_mutex_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::ActiveControl> active_control_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<Model> franka_hardware_model_;

  std::array<double, 7> last_desired_torque_ = {0, 0, 0, 0, 0, 0, 0};
};
}  // namespace franka_hardware
