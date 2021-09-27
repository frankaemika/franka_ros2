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

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace franka_gripper {

template <typename T>
bool resultIsReady(std::future<T>& t, std::chrono::nanoseconds future_wait_timeout) {
  return t.wait_for(future_wait_timeout) == std::future_status::ready;
}

class GripperActionServer : public rclcpp::Node {
 public:
  using Homing = franka_msgs::action::Homing;
  using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

  using Move = franka_msgs::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  using Grasp = franka_msgs::action::Grasp;
  using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

  using Trigger = std_srvs::srv::Trigger;

  explicit GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  enum class Task { kHoming, kMove, kGrasp, kGripperCommand };
  static std::string getTaskName(Task task) {
    switch (task) {
      case Task::kHoming:
        return {"Homing"};
      case Task::kMove:
        return {"Moving"};
      case Task::kGrasp:
        return {"Grasping"};
      case Task::kGripperCommand:
        return {"GripperCommand"};
      default:
        throw std::invalid_argument("getTaskName is not implemented for this case");
    }
  };

  const double k_default_grasp_epsilon = 0.005;
  const double k_default_speed = 0.1;
  const double k_default_state_publish_rate = 30.;
  const double k_default_feedback_publish_rate = 10.;

  std::unique_ptr<franka::Gripper> gripper_;
  rclcpp_action::Server<Homing>::SharedPtr homing_server_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  std::mutex gripper_state_mutex_;
  franka::GripperState current_gripper_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double default_speed_;
  double default_epsilon_inner_;
  double default_epsilon_outer_;
  std::vector<std::string> joint_names_;
  std::chrono::nanoseconds future_wait_timeout_{0};

  void publishGripperState();
  void stopServiceCallback(const std::shared_ptr<Trigger::Response>& response);

  rclcpp_action::CancelResponse handleCancel(Task task);
  rclcpp_action::GoalResponse handleGoal(Task task);

  void executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle);
  void executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle);
  void executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);

  void executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                             const std::function<bool()>& command_lambda);
  void prepareAndExecuteGripperCommand(
      const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

  template <typename T>
  auto generateCommandExecutionThread(const std::function<bool()>& command_lambda)
      -> std::function<std::shared_ptr<typename T::Result>()> {
    return [command_lambda, this]() {
      auto result = std::make_shared<typename T::Result>();
      try {
        result->success = command_lambda();
      } catch (const franka::Exception& e) {
        result->success = false;
        result->error = e.what();
      }
      return result;
    };
  }

  template <typename T>
  void executeCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle,
                      Task task,
                      const std::function<bool()>& command_lambda) {
    const auto kTaskName = getTaskName(task);
    RCLCPP_INFO(this->get_logger(), "Gripper %s...", kTaskName.c_str());

    auto command_execution_thread = generateCommandExecutionThread<T>(command_lambda);

    std::future<std::shared_ptr<typename T::Result>> result_future =
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
      publishGripperWidthFeedback(goal_handle);
    }
    const auto kResult = result_future.get();
    if (kResult->success) {
      RCLCPP_INFO(get_logger(), "Gripper %s succeeded", kTaskName.c_str());
      goal_handle->succeed(kResult);
    } else {
      RCLCPP_INFO(get_logger(), "Gripper %s failed", kTaskName.c_str());
      goal_handle->abort(kResult);
    }
  }

  template <typename T>
  void publishGripperWidthFeedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    auto gripper_feedback = std::make_shared<typename T::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    gripper_feedback->current_width = current_gripper_state_.width;
    goal_handle->publish_feedback(gripper_feedback);
  }

  void publishGripperCommandFeedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
    auto gripper_feedback = std::make_shared<GripperCommand::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    gripper_feedback->position =
        current_gripper_state_.width / 2;  // todo this was not done in franka_ros
    gripper_feedback->effort = 0.;
    goal_handle->publish_feedback(gripper_feedback);
  }
};
}  // namespace franka_gripper
