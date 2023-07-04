#pragma once

#include <limits>
#include <string>
#include <vector>

#include "franka/robot_state.h"
#include "franka_msgs/msg/errors.hpp"
#include "franka_msgs/msg/franka_state.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaState
    : public semantic_components::SemanticComponentInterface<franka_msgs::msg::FrankaState> {
 public:
  explicit FrankaState(const std::string& name);

  virtual ~FrankaState() = default;

  const franka::RobotState& getRobotState() const noexcept { return *robot_state_ptr_; }

  /// Return Imu message with orientation, angular velocity and linear acceleration
  /**
   * Constructs and return a FrankaState message from the current values.
   * \return FrankaState message from values;
   */
  bool get_values_as_message(franka_msgs::msg::FrankaState& message);

 protected:
  franka::RobotState* robot_state_ptr_;
};

}  // namespace franka_semantic_components
