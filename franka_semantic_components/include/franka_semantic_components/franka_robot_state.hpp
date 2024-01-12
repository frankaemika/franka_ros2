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

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "urdf/model.h"

#include "franka/robot_state.h"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaRobotState
    : public semantic_components::SemanticComponentInterface<franka_msgs::msg::FrankaRobotState> {
 public:
  explicit FrankaRobotState(const std::string& name, const std::string& robot_description);

  virtual ~FrankaRobotState() = default;

  /**
   * @param[in/out] message Initializes this message to contain the respective frame_id information
   */
  virtual auto initialize_robot_state_msg(franka_msgs::msg::FrankaRobotState& message) -> void;

  /**
   * Constructs and return a FrankaRobotState message from the current values.
   * \return FrankaRobotState message from values;
   */
  virtual auto get_values_as_message(franka_msgs::msg::FrankaRobotState& message) -> bool;

 protected:
  franka::RobotState* robot_state_ptr;

 private:
  std::string robot_description_;
  std::string robot_name_;
  const std::string state_interface_name_{"robot_state"};
  bool gripper_loaded_{false};
  size_t kEndEffectorLinkIndex{8};
  size_t kTotalAmountOfLinksWithoutEndEffector{8};
  size_t kTotalAmountOfJoints{8};
  // TODO(yazi_ba) update stiffness frame with the user defined transformation
  size_t kStiffnessLinkIndex{8};
  std::shared_ptr<urdf::Model> model_;
  std::vector<std::string> joint_names, link_names;

  /**
   * @brief Populate the link_name std::vector with the links from urdf object in order.
   *       The root link is the first element and tcp is the last element.
   *
   */
  auto set_links_from_urdf() -> void;

  /**
   * @brief Populate the joint_name std::vector with the joints from urdf object in order.
   *
   */
  auto set_joints_from_urdf() -> void;

  /**
   * @brief Recursively sets all child links from a link and assign them to the link_name
   *
   * @param link root link
   */
  auto set_child_links_recursively(const std::shared_ptr<const urdf::Link>& link) -> void;

  /**
   * @brief Check if gripper is loaded
   *        Checks if the robot_name + "_hand_tcp" frame exists
   *
   * @return true if gripper is loaded
   * @return false if gripper is not loaded
   */
  auto is_gripper_loaded() -> bool;
  /**
   * @brief Get the robot name from urdf object

   * @return std::string
   */
  auto get_robot_name_from_urdf() -> std::string;

  /**
   * @brief Get the link index from link name
   *
   * @param link_name
   * @return size_t
   */
  auto get_link_index(const std::string& link_name) -> size_t;
};

}  // namespace franka_semantic_components