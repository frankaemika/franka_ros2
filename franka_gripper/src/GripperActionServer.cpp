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

#include "franka_gripper/GripperActionServer.hpp"
#include <utility>
using namespace std::chrono_literals;
franka_gripper::GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
    : Node("franka_gripper_node", options) {
  this->declare_parameter("robot_ip");
  this->declare_parameter("default_epsilon_inner", 0.005);
  this->declare_parameter("default_epsilon_outer", 0.005);
  this->declare_parameter("default_speed", 0.1);
  this->declare_parameter("joint_names");
  this->declare_parameter("state_publish_rate", 30);
  this->declare_parameter("feedback_publish_rate", 10);
  std::string robot_ip;
  if (not this->get_parameter<std::string>("robot_ip", robot_ip)) {
    RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_ip' not set");
  }

  this->default_speed_ = this->get_parameter("default_speed").as_double();
  this->default_epsilon_inner_ = this->get_parameter("default_epsilon_inner").as_double();
  this->default_epsilon_outer_ = this->get_parameter("default_epsilon_outer").as_double();
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

  const double state_publish_rate = (double)this->get_parameter("state_publish_rate").as_int();
  const double wait_time_rate = (double)this->get_parameter("feedback_publish_rate").as_int();
  this->future_wait_timeout_ = rclcpp::WallRate(wait_time_rate).period();

  this->gripper_ = std::make_unique<franka::Gripper>(robot_ip);
  current_gripper_state_ = gripper_->readOnce();
  const auto homing_task = Task::Homing;
  this->stop_service_ = create_service<Trigger>(
      "stop",
      [=](std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        return stop_service_callback(std::move(response));
      });

  this->homing_server_ = rclcpp_action::create_server<Homing>(
      this, "homing", [=](auto uuid, auto goal) { return handle_goal(homing_task); },
      [=](const auto& goal_handle) { return handle_cancel(homing_task); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { execute_homing(goal_handle); }}.detach();
      });
  const auto move_task = Task::Move;
  this->move_server_ = rclcpp_action::create_server<Move>(
      this, "move", [=](auto uuid, auto goal) { return handle_goal(move_task); },
      [=](const auto& goal_handle) { return handle_cancel(move_task); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { execute_move(goal_handle); }}.detach();
      });

  const auto grasp_task = Task::Grasp;
  this->grasp_server_ = rclcpp_action::create_server<Grasp>(
      this, "grasp", [=](auto uuid, auto goal) { return handle_goal(grasp_task); },
      [=](const auto& goal_handle) { return handle_cancel(grasp_task); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { execute_grasp(goal_handle); }}.detach();
      });

  const auto gripper_command_task = Task::GripperCommand;
  this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "gripper_action",
      [=](auto uuid, auto goal) { return handle_goal(gripper_command_task); },
      [=](const auto& goal_handle) { return handle_cancel(gripper_command_task); },
      [=](const auto& goal_handle) {
        return std::thread{[=]() { prepare_and_execute_gripper_command(goal_handle); }}.detach();
      });

  this->joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "gripper_state", rclcpp::SensorDataQoS());
  this->timer_ = this->create_wall_timer(rclcpp::WallRate(state_publish_rate).period(),
                                         [&]() { return publish_gripper_state(); });
}

rclcpp_action::CancelResponse franka_gripper::GripperActionServer::handle_cancel(
    franka_gripper::Task task) {
  RCLCPP_INFO(this->get_logger(), "Received request to handle_cancel %s",
              get_task_name(task).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::GoalResponse franka_gripper::GripperActionServer::handle_goal(
    franka_gripper::Task task) {
  RCLCPP_INFO(this->get_logger(), "Received %s request", get_task_name(task).c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void franka_gripper::GripperActionServer::execute_homing(
    const std::shared_ptr<GoalHandleHoming>& goal_handle) {
  const auto command = [=]() { return gripper_->homing(); };
  execute_command(goal_handle, Task::Homing, command);
}
void franka_gripper::GripperActionServer::execute_move(
    const std::shared_ptr<GoalHandleMove>& goal_handle) {
  auto command = [=]() {
    const auto goal = goal_handle->get_goal();
    return gripper_->move(goal->width, goal->speed);
  };
  execute_command(goal_handle, Task::Move, command);
}
void franka_gripper::GripperActionServer::execute_grasp(
    const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
  auto command = [=]() {
    const auto goal = goal_handle->get_goal();
    return gripper_->grasp(goal->width, goal->speed, goal->force, goal->epsilon.inner,
                           goal->epsilon.outer);
  };
  execute_command(goal_handle, Task::Grasp, command);
}

void franka_gripper::GripperActionServer::prepare_and_execute_gripper_command(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  const double target_width = 2 * goal->command.position;

  std::unique_lock<std::mutex> guard(gripper_state_mutex_);
  constexpr double kSamePositionThreshold = 1e-4;
  auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
  const double current_width = current_gripper_state_.width;
  if (target_width > current_gripper_state_.max_width or target_width < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "GripperServer: Commanding out of range width! max_width = %f command = %f",
                 current_gripper_state_.max_width, target_width);
    goal_handle->abort(result);
    return;
  } else if (std::abs(target_width - current_width) < kSamePositionThreshold) {
    result->effort = 0;
    result->position = current_width;
    result->reached_goal = true;
    result->stalled = false;
    goal_handle->succeed(result);
    return;
  }
  guard.unlock();
  auto command = [=]() {
    if (target_width >= current_width) {
      return gripper_->move(target_width, default_speed_);
    } else {
      return gripper_->grasp(target_width, default_speed_, goal->command.max_effort,
                             default_epsilon_inner_, default_epsilon_outer_);
    }
  };

  execute_gripper_command(goal_handle, command);
}

void franka_gripper::GripperActionServer::execute_gripper_command(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    const std::function<bool()>& command_lambda) {
  const auto task_name = get_task_name(Task::GripperCommand);
  RCLCPP_INFO(this->get_logger(), "Gripper %s...", task_name.c_str());

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

  while (not result_is_ready(result_future, future_wait_timeout_)) {
    if (goal_handle->is_canceling()) {
      gripper_->stop();
      result_future.wait();
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "Gripper %s canceled", task_name.c_str());
      goal_handle->canceled(result);
      return;
    }
    publish_gripper_command_feedback(goal_handle);
  }
  const auto result = result_future.get();
  std::lock_guard<std::mutex> guard(gripper_state_mutex_);
  result->position = current_gripper_state_.width / 2;  // todo this was not done in franka_ros
  result->effort = 0.;
  if (result->reached_goal) {
    RCLCPP_INFO(get_logger(), "Gripper %s succeeded", task_name.c_str());
    goal_handle->succeed(result);
  } else {
    RCLCPP_INFO(get_logger(), "Gripper %s failed", task_name.c_str());
    goal_handle->abort(result);
  }
}
void franka_gripper::GripperActionServer::stop_service_callback(
    std::shared_ptr<Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Stopping gripper_...");
  auto action_result =
      generate_command_execution_thread<Homing>([=]() { return gripper_->stop(); })();
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
void franka_gripper::GripperActionServer::publish_gripper_state() {
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
  joint_states.position.push_back(current_gripper_state_.width * 0.5);
  joint_states.position.push_back(current_gripper_state_.width * 0.5);
  joint_states.velocity.push_back(0.0);
  joint_states.velocity.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states_publisher_->publish(joint_states);
}

RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::GripperActionServer)