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

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_gripper/GripperActionServer.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <utility>

namespace franka_gripper {
GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
    : Node("franka_gripper_node", options) {
  this->declare_parameter("robot_ip");
  this->declare_parameter("default_grasp_epsilon.inner", k_default_grasp_epsilon);
  this->declare_parameter("default_grasp_epsilon.outer", k_default_grasp_epsilon);
  this->declare_parameter("default_speed", k_default_speed);
  this->declare_parameter("joint_names");
  this->declare_parameter("state_publish_rate", k_default_state_publish_rate);
  this->declare_parameter("feedback_publish_rate", k_default_feedback_publish_rate);
  std::string robot_ip;
  if (not this->get_parameter<std::string>("robot_ip", robot_ip)) {
    RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_ip' not set");
  }

  this->default_speed_ = this->get_parameter("default_speed").as_double();
  this->default_epsilon_inner_ = this->get_parameter("default_grasp_epsilon.inner").as_double();
  this->default_epsilon_outer_ = this->get_parameter("default_grasp_epsilon.outer").as_double();
  if (not this->get_parameter("joint_names", this->joint_names_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'joint_names' not set");
    this->joint_names_ = {"", ""};
  }

  if (this->joint_names_.size() != 2) {
    RCLCPP_FATAL(this->get_logger(),
                 "Parameter 'joint_names' needs exactly two arguments, got %d instead",
                 this->joint_names_.size());
    throw std::invalid_argument("Parameter 'joint_names' has wrong number of arguments");
  }

  const double kStatePublishRate = this->get_parameter("state_publish_rate").as_double();
  const double kFeedbackPublishRate = this->get_parameter("feedback_publish_rate").as_double();
  this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();

  this->gripper_ = std::make_unique<franka::Gripper>(robot_ip);
  current_gripper_state_ = gripper_->readOnce();
  const auto kHomingTask = Task::kHoming;
  this->stop_service_ =  // NOLINTNEXTLINE
      create_service<Trigger>("stop", [=](std::shared_ptr<Trigger::Request> /*request*/,
                                          std::shared_ptr<Trigger::Response> response) {  // NOLINT
        return stopServiceCallback(std::move(response));                                  // NOLINT
      });

  this->homing_server_ = rclcpp_action::create_server<Homing>(
      this, "homing", [=](auto /*uuid*/, auto /*goal*/) { return handleGoal(kHomingTask); },
      [=](const auto& /*goal_handle*/) { return handleCancel(kHomingTask); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { executeHoming(goal_handle); }}.detach();
      });
  const auto kMoveTask = Task::kMove;
  this->move_server_ = rclcpp_action::create_server<Move>(
      this, "move", [=](auto /*uuid*/, auto /*goal*/) { return handleGoal(kMoveTask); },
      [=](const auto& /*goal_handle*/) { return handleCancel(kMoveTask); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { executeMove(goal_handle); }}.detach();
      });

  const auto kGraspTask = Task::kGrasp;
  this->grasp_server_ = rclcpp_action::create_server<Grasp>(
      this, "grasp", [=](auto /*uuid*/, auto /*goal*/) { return handleGoal(kGraspTask); },
      [=](const auto& /*goal_handle*/) { return handleCancel(kGraspTask); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { executeGrasp(goal_handle); }}.detach();
      });

  const auto kGripperCommandTask = Task::kGripperCommand;
  this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "gripper_action",
      [=](auto /*uuid*/, auto /*goal*/) { return handleGoal(kGripperCommandTask); },
      [=](const auto& /*goal_handle*/) { return handleCancel(kGripperCommandTask); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { prepareAndExecuteGripperCommand(goal_handle); }}.detach();
      });

  this->joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  this->timer_ = this->create_wall_timer(rclcpp::WallRate(kStatePublishRate).period(),
                                         [&]() { return publishGripperState(); });
}

rclcpp_action::CancelResponse GripperActionServer::handleCancel(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received request to handleCancel %s", getTaskName(task).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperActionServer::handleGoal(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received %s request", getTaskName(task).c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperActionServer::executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle) {
  const auto kCommand = [=]() { return gripper_->homing(); };
  executeCommand(goal_handle, Task::kHoming, kCommand);
}

void GripperActionServer::executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle) {
  auto command = [=]() {
    const auto kGoal = goal_handle->get_goal();
    return gripper_->move(kGoal->width, kGoal->speed);
  };
  executeCommand(goal_handle, Task::kMove, command);
}

void GripperActionServer::executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
  auto command = [=]() {
    const auto kGoal = goal_handle->get_goal();
    return gripper_->grasp(kGoal->width, kGoal->speed, kGoal->force, kGoal->epsilon.inner,
                           kGoal->epsilon.outer);
  };
  executeCommand(goal_handle, Task::kGrasp, command);
}

void GripperActionServer::prepareAndExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
  const auto kGoal = goal_handle->get_goal();
  const double kTargetWidth = 2 * kGoal->command.position;

  std::unique_lock<std::mutex> guard(gripper_state_mutex_);
  constexpr double kSamePositionThreshold = 1e-4;
  auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
  const double kCurrentWidth = current_gripper_state_.width;
  if (kTargetWidth > current_gripper_state_.max_width or kTargetWidth < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "GripperServer: Commanding out of range width! max_width = %f command = %f",
                 current_gripper_state_.max_width, kTargetWidth);
    goal_handle->abort(result);
    return;
  }
  if (std::abs(kTargetWidth - kCurrentWidth) < kSamePositionThreshold) {
    result->effort = 0;
    result->position = kCurrentWidth;
    result->reached_goal = true;
    result->stalled = false;
    goal_handle->succeed(result);
    return;
  }
  guard.unlock();
  auto command = [=]() {
    if (kTargetWidth >= kCurrentWidth) {
      return gripper_->move(kTargetWidth, default_speed_);
    }
    return gripper_->grasp(kTargetWidth, default_speed_, kGoal->command.max_effort,
                           default_epsilon_inner_, default_epsilon_outer_);
  };

  executeGripperCommand(goal_handle, command);
}

void GripperActionServer::executeGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    const std::function<bool()>& command_lambda) {
  const auto kTaskName = getTaskName(Task::kGripperCommand);
  RCLCPP_INFO(this->get_logger(), "Gripper %s...", kTaskName.c_str());

  auto command_execution_thread = [=]() {
    auto result = std::make_shared<GripperCommand::Result>();
    try {
      result->reached_goal = command_lambda();
    } catch (const franka::Exception& e) {
      result->reached_goal = false;
      RCLCPP_ERROR(this->get_logger(), e.what());
    }
    return result;
  };

  std::future<std::shared_ptr<typename GripperCommand ::Result>> result_future =
      std::async(std::launch::async, command_execution_thread);

  while (not resultIsReady(result_future, future_wait_timeout_)) {
    if (goal_handle->is_canceling()) {
      gripper_->stop();
      result_future.wait();
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "Gripper %s canceled", kTaskName.c_str());
      goal_handle->canceled(result);
      return;
    }
    publishGripperCommandFeedback(goal_handle);
  }
  const auto kResult = result_future.get();
  std::lock_guard<std::mutex> guard(gripper_state_mutex_);
  kResult->position = current_gripper_state_.width / 2;  // todo this was not done in franka_ros
  kResult->effort = 0.;
  if (kResult->reached_goal) {
    RCLCPP_INFO(get_logger(), "Gripper %s succeeded", kTaskName.c_str());
    goal_handle->succeed(kResult);
  } else {
    RCLCPP_INFO(get_logger(), "Gripper %s failed", kTaskName.c_str());
    goal_handle->abort(kResult);
  }
}

void GripperActionServer::stopServiceCallback(const std::shared_ptr<Trigger::Response>& response) {
  RCLCPP_INFO(this->get_logger(), "Stopping gripper_...");
  auto action_result = generateCommandExecutionThread<Homing>([=]() { return gripper_->stop(); })();
  response->success = action_result->success;
  response->message = action_result->error;
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Gripper stopped");
  } else {
    RCLCPP_INFO(this->get_logger(), "Gripper could not be stopped");
  }
  if (not response->message.empty()) {
    RCLCPP_ERROR(this->get_logger(), response->message.c_str());
  }
}

void GripperActionServer::publishGripperState() {
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  try {
    current_gripper_state_ = gripper_->readOnce();
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.name.push_back(this->joint_names_[0]);
  joint_states.name.push_back(this->joint_names_[1]);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.velocity.push_back(0.0);
  joint_states.velocity.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states_publisher_->publish(joint_states);
}
}  // namespace franka_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::GripperActionServer)  // NOLINT