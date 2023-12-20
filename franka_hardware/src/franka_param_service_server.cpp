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

#include "franka_hardware/franka_param_service_server.hpp"

namespace franka_hardware {

FrankaParamServiceServer::FrankaParamServiceServer(const rclcpp::NodeOptions& options,
                                                   std::shared_ptr<Robot> robot)
    : rclcpp::Node("service_server", options), robot_(std::move(robot)) {
  set_joint_stiffness_service_ = create_service<franka_msgs::srv::SetJointStiffness>(
      "~/set_joint_stiffness",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setJointStiffnessCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  set_cartesian_stiffness_service_ = create_service<franka_msgs::srv::SetCartesianStiffness>(
      "~/set_cartesian_stiffness",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setCartesianStiffnessCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  set_tcp_frame_service_ = create_service<franka_msgs::srv::SetTCPFrame>(
      "~/set_tcp_frame",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setTCPFrameCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  set_stiffness_frame_service_ = create_service<franka_msgs::srv::SetStiffnessFrame>(
      "~/set_stiffness_frame",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setStiffnessFrameCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  set_force_torque_collision_behavior_service_ =
      create_service<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
          "~/set_force_torque_collision_behavior",
          std::bind(  // NOLINT [modernize-avoid-bind]
              &FrankaParamServiceServer::setForceTorqueCollisionBehaviorCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  set_full_collision_behavior_service_ = create_service<franka_msgs::srv::SetFullCollisionBehavior>(
      "~/set_full_collision_behavior",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setFullCollisionBehaviorCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  set_load_service_ = create_service<franka_msgs::srv::SetLoad>(
      "~/set_load",
      std::bind(  // NOLINT [modernize-avoid-bind]
          &FrankaParamServiceServer::setLoadCallback, this, std::placeholders::_1,
          std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Service started");
}

void FrankaParamServiceServer::setJointStiffnessCallback(
    const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request,
    const franka_msgs::srv::SetJointStiffness::Response::SharedPtr& response) {
  auto set_joint_stiffness_function =
      [&](const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) {
        robot_->setJointStiffness(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetJointStiffness::Request::SharedPtr,
                       franka_msgs::srv::SetJointStiffness::Response::SharedPtr>(
      set_joint_stiffness_function, request, response);
}

void FrankaParamServiceServer::setCartesianStiffnessCallback(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request,
    const franka_msgs::srv::SetCartesianStiffness::Response::SharedPtr& response) {
  auto set_cartesian_stiffness_function =
      [&](const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) {
        robot_->setCartesianStiffness(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr,
                       franka_msgs::srv::SetCartesianStiffness::Response::SharedPtr>(
      set_cartesian_stiffness_function, request, response);
}

void FrankaParamServiceServer::setTCPFrameCallback(
    const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request,
    const franka_msgs::srv::SetTCPFrame::Response::SharedPtr& response) {
  auto set_tcp_frame_function =
      [&](const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) {
        robot_->setTCPFrame(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetTCPFrame::Request::SharedPtr,
                       franka_msgs::srv::SetTCPFrame::Response::SharedPtr>(set_tcp_frame_function,
                                                                           request, response);
}

void FrankaParamServiceServer::setStiffnessFrameCallback(
    const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request,
    const franka_msgs::srv::SetStiffnessFrame::Response::SharedPtr& response) {
  auto set_stiffness_frame_function =
      [&](const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) {
        robot_->setStiffnessFrame(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr,
                       franka_msgs::srv::SetStiffnessFrame::Response::SharedPtr>(
      set_stiffness_frame_function, request, response);
}

void FrankaParamServiceServer::setForceTorqueCollisionBehaviorCallback(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request,
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr& response) {
  auto set_force_torque_collision_behavior_function =
      [&](const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request) {
        robot_->setForceTorqueCollisionBehavior(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr,
                       franka_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr>(
      set_force_torque_collision_behavior_function, request, response);
}

void FrankaParamServiceServer::setFullCollisionBehaviorCallback(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request,
    const franka_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr& response) {
  auto set_full_collision_behavior_function =
      [&](const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) {
        robot_->setFullCollisionBehavior(request);
      };
  setGenericRobotParam<franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr,
                       franka_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr>(
      set_full_collision_behavior_function, request, response);
}

void FrankaParamServiceServer::setLoadCallback(
    const franka_msgs::srv::SetLoad::Request::SharedPtr& request,
    const franka_msgs::srv::SetLoad::Response::SharedPtr& response) {
  auto set_load_function = [&](const franka_msgs::srv::SetLoad::Request::SharedPtr& request) {
    robot_->setLoad(request);
  };
  setGenericRobotParam<franka_msgs::srv::SetLoad::Request::SharedPtr,
                       franka_msgs::srv::SetLoad::Response::SharedPtr>(set_load_function, request,
                                                                       response);
}

}  // namespace franka_hardware
