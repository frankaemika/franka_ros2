#include <cstddef>
#include <franka_example_controllers/cartesian_impedance_example_controller.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <Eigen/Eigen>
#include <std_msgs/msg/header.cpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::command_interface_configuration() const {
  /*
   * TODO:
   * Define command interfaces here
  */
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // one way to get command config interfaces
  for (int i = 1; i <= num_joints; i++) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/effort");
  }

  // get it from robot state semantic components
  config.names = franka_robot_state_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::state_interface_configuration() const {
  /*
   * TODO:
   * Define state interfaces
  */
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Creates state interface for the desired equilibrium position
  config.names = equilibrium_pose_d_->get_state_interface_names();

  // Creates state interface for everything published in robot state (joint
  // position, velocity)
  config.names.push_back(franka_robot_state_->get_state_interface_names());
  // for (int i = 1; i<=num_joints; i++) {
  //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/position");
  //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/velocity");
  // }

  // Create state interface to read robot model for computations involving
  // robot dynamics
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config
}

controller_interface::return_type update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period) {
  /*
   * Generates and writes the joint torques to the joints of the robot
   * TODO:
  */
  // get robot state
  // DOUBT: placeholder for getting complete robot state. Pass a franka state
  // message into this function?
  robot_state_ = std::make_unique<franka_msgs::msg::FrankaRobotState>();
  franka_robot_state_->get_values_as_message(robot_state_);

  // get coriolis
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolis();

  // get jacobian DOUBT: How to access the relevant end effector frame
  std::array<double, 42> Jacobian_array = franka_robot_model_->getZeroJacobian(
    franka::Frame::kEndEffector);

  // Convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(
    robot_state_.tau_J_d.data());

  // get current eef pose X_EE(drake)
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state_.O_T_EE.date()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // Get desired equilibrium pose
  // construct error
  // tranlation error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  // QUE: What is happening here? work it out
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen:: Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

  // transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control law
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoinverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  // TODO: implement cartesian_stiffness and cartesian dampness
  tau_task << jacobian,transpose() * 
   (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
    jacobian.transpose()* jacobian_transpose_pinv) *
    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
     (2.0 * sqrt(nullspace_stiffness_)) * dq);

  tau_d << tau_task + tau_nullspace + coriolis;

  // TODO: implement saturation of torque commanded
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // command the torques
  for (int i = 0; i < num_joints; i++) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  // update parameters
}

CallbackReturn CartesianImpedanceExampleController::on_init() {
  /*
   * TODO: Intialise some stuff
   * Intialize variables
   * Reserve memory
   * Declare node parameters
  */
}

CallbackReturn CartesianImpedanceExampleController::on_configure(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * TODO: Intitialise something here as well
   * This is where you instantiate objects
   * Read parameters here
  */
  arm_id_ = get_node()
}

CallbackReturn CartesianImpedanceExampleController::on_activate(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * TODO:
   * Assign loaned command and state interfaces
  */
  // state interfaces
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  equilibrium_pose_d_->assign_loaned_state_interfaces(state_interfaces_);

  // command interfaces
  franka_robot_state_->assign_loaned_command_interfaces(command_interfaces_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_deactivate(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * TODO:
   * De-assign loaned command and state interfaces
  */

  franka_robot_state_->release_interfaces();
  franka_robot_model_->release_interfaces();
  equilibrium_pose_d_->release_interfaces();

  return CallbackReturn::SUCCESS;
} 

void CartesianImpedanceExampleController::UpdateJointStates() {
  // update joint states

}

Eigen::Matrix<double, 7, 1> saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < num_joints; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
       tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

} // namespace franka_example_controllers

