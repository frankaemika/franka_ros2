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

#include <array>

#include <franka/model.h>

namespace franka_hardware {

/**
 * This class is a thin wrapper around a @ref franka::Model and delegates all calls to
 * that
 */
class Model {  // NOLINT(cppcoreguidelines-pro-type-member-init,
               // cppcoreguidelines-special-member-functions)
 public:
  /**
   * Create a new Model instance wrapped around a franka::Model
   */
  explicit Model(franka::Model* model) : model_(model) {}
  virtual ~Model() = default;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  [[nodiscard]] std::array<double, 16> pose(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const {
    return model_->pose(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const {
    return model_->bodyJacobian(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const {
    return model_->zeroJacobian(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  [[nodiscard]] std::array<double, 49> mass(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept {
    return model_->mass(q, I_total, m_total, F_x_Ctotal);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   */
  [[nodiscard]] std::array<double, 7> coriolis(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept {
    return model_->coriolis(q, dq, I_total, m_total, F_x_Ctotal);
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] std::array<double, 7> gravity(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const noexcept {
    return model_->gravity(q, m_total, F_x_Ctotal, gravity_earth);
  }
  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  [[nodiscard]] virtual std::array<double, 16> pose(franka::Frame frame,
                                                    const franka::RobotState& robot_state) const {
    return pose(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] virtual std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const franka::RobotState& robot_state) const {
    return bodyJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  [[nodiscard]] virtual std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const franka::RobotState& robot_state) const {
    return zeroJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  [[nodiscard]] virtual std::array<double, 49> mass(const franka::RobotState& robot_state) const {
    return mass(robot_state.q, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the Coriolis force vector should be calculated.
   *
   * @return Coriolis force vector.
   */
  [[nodiscard]] virtual std::array<double, 7> coriolis(
      const franka::RobotState& robot_state) const {
    return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total,
                    robot_state.F_x_Ctotal);
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$. Assumes default gravity vector of -9.81 m/s^2
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] std::array<double, 7> gravity(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal  // NOLINT(readability-identifier-naming)
  ) const {
    return gravity(q, m_total, F_x_Ctotal, {0, 0, -9.81});
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] virtual std::array<double, 7> gravity(
      const franka::RobotState& robot_state,
      const std::array<double, 3>& gravity_earth) const {
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, gravity_earth);
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$. Assumes default gravity vector of -9.81 m/s^2
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   *
   * @return Gravity vector.
   */
  [[nodiscard]] virtual std::array<double, 7> gravity(const franka::RobotState& robot_state) const {
#ifdef ENABLE_BASE_ACCELERATION
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, robot_state.O_ddP_O);
#else
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, {0, 0, -9.81});
#endif
  }

 protected:
  Model() = default;

 private:
  franka::Model* model_;
};

}  // namespace franka_hardware
