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

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_robot_state_broadcaster/async_buffer.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster_parameters.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

namespace franka_robot_state_broadcaster {

/**
 * Publishes the full Franka robot state and a set of convenience topics.
 *
 * The controller update() path copies robot state from the hardware
 * RealtimeThreadSafeBox, builds the ROS message into an AsyncBuffer slot, and
 * returns. A dedicated publish thread drains that buffer and performs all DDS
 * publishes, so the publish cost does not sit between robot state arrival
 * and command egress.
 *
 * Keep this controller synchronous (is_async: false) so it shares the controller
 * manager thread with model-based controllers that also read the state box.
 */
class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  explicit FrankaRobotStateBroadcaster(
      std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state = nullptr);

  ~FrankaRobotStateBroadcaster() override;

  // SCHED_FIFO priority for the publish thread. Stay below the CM RT loop (97)
  // and below typical PREEMPT_RT NIC IRQ threads (50) so publishing cannot delay
  // packet delivery or control.
  static constexpr int kPublishThreadPriority = 30;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::string state_interface_name_{"robot_state"};
  rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_stamped_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      last_desired_pose_stamped_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      desired_end_effector_twist_stamped_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      external_wrench_in_base_frame_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      external_wrench_in_stiffness_frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr external_joint_torques_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr measured_joint_states_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_states_publisher_;

  const std::string kCurrentPoseTopic = "~/current_pose";
  const std::string kLastDesiredPoseTopic = "~/last_desired_pose";
  const std::string kDesiredEETwist = "~/desired_end_effector_twist";
  const std::string kMeasuredJointStates = "~/measured_joint_states";
  const std::string kExternalWrenchInStiffnessFrame = "~/external_wrench_in_stiffness_frame";
  const std::string kExternalWrenchInBaseFrame = "~/external_wrench_in_base_frame";
  const std::string kExternalJointTorques = "~/external_joint_torques";
  const std::string kDesiredJointStates = "~/desired_joint_states";

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  AsyncBuffer<franka_msgs::msg::FrankaRobotState> state_buffer_;

  std::thread publish_thread_;
  std::atomic<bool> is_publish_thread_running_{false};
  std::mutex publish_mutex_;
  std::condition_variable publish_cv_;
  std::atomic<bool> data_ready_{false};

  // Convenience topics publish every N-th fresh sample on the publish thread.
  int convenience_publish_skip_{1};

  void startPublishThread();
  void stopPublishThread();
  void publishRunner();
};
}  // namespace franka_robot_state_broadcaster
