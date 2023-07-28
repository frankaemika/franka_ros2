// Copyright (c) 2023 Franka Emika GmbH
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

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include "franka/robot_state.h"
#include "franka_hardware/model.hpp"

#include "semantic_components/semantic_component_interface.hpp"

namespace franka_semantic_components {
class FrankaRobotModel
    : public semantic_components::SemanticComponentInterface<franka_hardware::Model> {
 public:
  /**
   * Creates an instance of a FrankaRobotModel.
   * @param[in] name The name of robot model state interface.
   */
  FrankaRobotModel(const std::string& franka_model_interface_name,
                   const std::string& franka_state_interface_name);
  FrankaRobotModel() = delete;

  virtual ~FrankaRobotModel() = default;

  /**
   * Calculates the 7x7 mass matrix from the current robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @throws Runtime error when state interfaces are not available.
   *
   * @see franka::Model::mass
   */
  std::array<double, 49> getMassMatrix() {
    if (!initialized) {
      initialize();
    }
    return robot_model->mass(*robot_state);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation) from the current robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @return Coriolis force vector.
   *
   * @throws Runtime error when state interfaces are not available.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 7> getCoriolisForceVector() {
    if (!initialized) {
      initialize();
    }
    return robot_model->coriolis(*robot_state);
  }

  /**
   * Calculates the gravity vector from the current robot state. Unit: \f$[Nm]\f$.
   *
   * @return Gravity vector.
   *
   * @throws Runtime error when state interfaces are not available
   * @see franka::Model::gravity
   */
  std::array<double, 7> getGravityForceVector() {
    if (!initialized) {
      initialize();
    }
    return robot_model->gravity(*robot_state);
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @throws Runtime error when state interfaces are not available
   * @see franka::Model::pose
   */
  std::array<double, 16> getPoseMatrix(const franka::Frame& frame) {
    if (!initialized) {
      initialize();
    }
    return robot_model->pose(frame, *robot_state);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to the given frame.
   *
   * BodyJacobian relates joint velocities to the end-effoctor twist expressed in the the given
   * frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the current robot state.
   *
   * Jacobian matrix is in the form | linear components  |
   *                                | angular components |
   *
   * E.g.
   *
   * To calculate the Jacobian of frame kJoint1 w.r.t kJoint1
   *
   * getBodyJacobian: (frame = kJoint1) will return kJoint1_J_kJoint1(6x7)
   *
   * kJoint1_J_kJoint1 can be used to calculate the twist in the Joint1 by multiplying
   * with the joint velocities.
   *
   * \f$^{1}{\mathcal{V}_{1}} = \, ^{1}{\mathcal{J}_{1}} * dq\f$
   *
   * Similarly, given desired joint twist in the kJoint1 frame, pseudoinverse of body jacobian can
   * be used to retrieve the desired joint velocity to command.
   *
   * \f$ dq = \ ^{1}{\mathcal{J}^{\dagger}_{1}} * \, ^{1}{\mathcal{V}_{1}}\f$
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @throws Runtime error when state interfaces are not available.
   * @see franka::Model::bodyJacobian
   */
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame) {
    if (!initialized) {
      initialize();
    }
    return robot_model->bodyJacobian(frame, *robot_state);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base(zero) frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the current robot state.
   *
   * Jacobian matrix is in the form | linear components  |
   *                                | angular components |
   *
   * E.g.
   *
   * To calculate the Jacobian of frame kJoint1 w.r.t base frame
   *
   * getZeroJacobian: (frame: kJoint1) will return base_J_kJoint1(6x7)
   *
   * base_J_kJoint1 can be used to calculate the twist in the Joint1 by multiplying
   * with the joint velocities.
   *
   * \f$^{O}{\mathcal{V}_{1}} = \, ^{O}{\mathcal{J}_{1}} * dq\f$
   *
   * Similarly, given desired joint twist in the base frame, pseudoinverse of zero jacobian can be
   *used to retrieve the desired joint velocity to command.
   *
   *
   *\f$ dq = \, ^{O}{\mathcal{J}^{\dagger}_{1}} * \, ^{O}{\mathcal{V}_{1}}\f$
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @throws Runtime error when state interfaces are not available.
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame) {
    if (!initialized) {
      initialize();
    }
    return robot_model->zeroJacobian(frame, *robot_state);
  }

 protected:
  /**
   * Retrieve the robot state and robot model pointers from the hardware state interface
   *
   * @throws Runtime error when state interfaces are not available.
   */
  void initialize();

  bool initialized{false};
  franka_hardware::Model* robot_model;
  franka::RobotState* robot_state;

 private:
  const std::string arm_id_{"panda"};

  const std::string robot_state_interface_name_{"robot_state"};
  const std::string robot_model_interface_name_{"robot_model"};
};

}  // namespace franka_semantic_components
