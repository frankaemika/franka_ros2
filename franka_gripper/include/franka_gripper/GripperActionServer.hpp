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

#ifndef FRANKA_GRIPPER_GRIPPERACTIONSERVER_HPP
#define FRANKA_GRIPPER_GRIPPERACTIONSERVER_HPP

#include <functional>
#include <memory>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include "control_msgs/action/gripper_command.hpp"
#include "franka/exception.h"
#include "franka/gripper.h"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/homing.hpp"
#include "franka_msgs/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

const int64_t FUTURE_STEP_TIME_MILLISECONDS = 50;
template <typename T>
bool result_is_ready(std::future<T>& t) {
  return t.wait_for(std::chrono::milliseconds(FUTURE_STEP_TIME_MILLISECONDS)) ==
         std::future_status::ready;
}

namespace franka_gripper {
enum class Task { Homing, Move, Grasp, GripperCommand };
std::string get_task_name(Task task) {
  switch (task) {
    case Task::Homing:
      return {"Homing"};
    case Task::Move:
      return {"Moving"};
    case Task::Grasp:
      return {"Grasping"};
    case Task::GripperCommand:
      return {"GripperCommand"};
  }
};
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

  void publish_gripper_state();
  void stop_service_callback(std::shared_ptr<Trigger::Response> response);

  rclcpp_action::CancelResponse handle_cancel(Task task);
  rclcpp_action::GoalResponse handle_goal(Task task);

  void execute_homing(const std::shared_ptr<GoalHandleHoming>& goal_handle);
  void execute_move(const std::shared_ptr<GoalHandleMove>& goal_handle);
  void execute_grasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);

  void execute_gripper_command(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                               const std::function<bool()>& command_lambda);
  void prepare_and_execute_gripper_command(
      const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

  template <typename T>
  std::function<std::shared_ptr<typename T::Result>()> generate_command_execution_thread(
      const std::function<bool()>& command_lambda) {
    return [=]() {
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
  void execute_command(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle,
                       Task task,
                       const std::function<bool()>& command_lambda) {
    const auto task_name = get_task_name(task);
    RCLCPP_INFO(this->get_logger(), "Gripper %s...", task_name.c_str());

    auto command_execution_thread = generate_command_execution_thread<T>(command_lambda);

    std::future<std::shared_ptr<typename T::Result>> result_future =
        std::async(std::launch::async, command_execution_thread);

    while (not result_is_ready(result_future)) {
      if (goal_handle->is_canceling()) {
        gripper_->stop();
        result_future.wait();
        auto result = result_future.get();
        RCLCPP_INFO(get_logger(), "Gripper %s canceled", task_name.c_str());
        goal_handle->canceled(result);
        return;
      }
      publish_gripper_width_feedback(goal_handle);
    }
    const auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO(get_logger(), "Gripper %s succeeded", task_name.c_str());
      goal_handle->succeed(result);
    } else {
      RCLCPP_INFO(get_logger(), "Gripper %s failed", task_name.c_str());
      goal_handle->abort(result);
    }
  }

  template <typename T>
  void publish_gripper_width_feedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    auto gripper_feedback = std::make_shared<typename T::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    gripper_feedback->current_width = current_gripper_state_.width;
    goal_handle->publish_feedback(gripper_feedback);
  }

  void publish_gripper_command_feedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
    auto gripper_feedback = std::make_shared<GripperCommand ::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    gripper_feedback->position =
        current_gripper_state_.width / 2;  // todo this was not done in franka_ros
    gripper_feedback->effort = 0.;
    goal_handle->publish_feedback(gripper_feedback);
  }
};
}  // namespace franka_gripper

#endif  // FRANKA_GRIPPER_GRIPPERACTIONSERVER_HPP
