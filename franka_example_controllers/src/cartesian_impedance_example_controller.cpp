#include <cstddef>
#include <franka_example_controllers/cartesian_impedance_example_controller.hpp>
#include <franka_example_controllers/pseudo_inversion.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <Eigen/Eigen>
#include <franka/robot_state.h>

// NOTE: Build Errors and interface questions
// TODO: Maybe get ros parameters in on_init(). ref:
// franka_robot_state_broadcaster
// TODO: Subscribe to the equilibrium pose topic and update target position and
// orientation of the controller. This is what the CIC will use to track.
// TODO: Monitor stiffness and damping parameters served as ROS params. Create
// a callback that changes these values. Ensure thread safety and prevent race
// conditions.

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::command_interface_configuration() const {
  /*
   * Define command interfaces here
   * NOTE: const word after function declaration makes it a compiler error to
   * change any data member of the class.
  */
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // DOUBT: Is there any other way to get command config
  for (int i = 1; i <= num_joints; i++) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  // get it from robot state semantic components
  // config.names = franka_robot_state_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::state_interface_configuration() const {
  /*
   * Define state interfaces
  */
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Creates state interface for the desired equilibrium position
  config.names = equilibrium_pose_d_->get_state_interface_names();

  // Creates state interface for everything published in robot state (joint
  // position, velocity)
  // config.names.push_back(franka_robot_state_->get_state_interface_names());
  for (const auto& franka_robot_state_name : franka_robot_state_->get_state_interface_names()) {
    config.names.push_back(franka_robot_state_name);
  }
  // for (int i = 1; i<=num_joints; i++) {
  //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/position");
  //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/velocity");
  // }

  // Create state interface to read robot model for computations involving
  // robot dynamics
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config;
}

controller_interface::return_type CartesianImpedanceExampleController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period) {
  /*
   * Generates and writes the joint torques to the joints of the robot
  */
  // get robot state
  // DOUBT: placeholder for getting complete robot state. Pass a franka state
  // message into this function?
  // DEBG: deleter after testing
  // robot_state_ =
  //   std::make_unique<franka_msgs::msg::FrankaRobotState>(
  //     franka_msgs::msg::FrankaRobotState());

  robot_state_ = franka_msgs::msg::FrankaRobotState();

  // TODO: Implement boolean check for if the robot state read has been
  // successfull or not
  franka_robot_state_->get_values_as_message(robot_state_);

  // get coriolis
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();

  // get jacobian DOUBT: How to access the relevant end effector frame
  std::array<double, 42> Jacobian_array = franka_robot_model_->getZeroJacobian(
    franka::Frame::kEndEffector);

  // Convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(Jacobian_array.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.measured_joint_state.position.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.measured_joint_state.velocity.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.dq.data());

  // DOUBT: Is this desired joint torque
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(
  //   robot_state_.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(
    robot_state_.desired_joint_state.effort.data());

  // get current eef pose X_EE(drake)
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state_.o_t_ee.pose.data()));
  // Eigen::Vector3d position(transform.translation());
  // Eigen::Quaterniond orientation(transform.rotation());
  Eigen::Vector3d position(
    robot_state_.o_t_ee.pose.position.x,
    robot_state_.o_t_ee.pose.position.y,
    robot_state_.o_t_ee.pose.position.z);
  Eigen::Quaterniond orientation(
    robot_state_.o_t_ee.pose.orientation.w,
    robot_state_.o_t_ee.pose.orientation.x,
    robot_state_.o_t_ee.pose.orientation.y,
    robot_state_.o_t_ee.pose.orientation.z);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = position;
  transform.rotate(orientation.toRotationMatrix());
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state_.o_t_ee.pose.data()));

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
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  // TODO: implement cartesian_stiffness and cartesian dampness
  tau_task << jacobian.transpose() * 
   (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
    jacobian.transpose()* jacobian_transpose_pinv) *
    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
     (2.0 * sqrt(nullspace_stiffness_)) * dq);

  tau_d << tau_task + tau_nullspace + coriolis;

  // saturate the commanded torque to joint limits
  tau_d << saturateTorqueRate(tau_d, tau_j_d);

  // command the torques
  for (int i = 0; i < num_joints; i++) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  // update parameters
  // fin
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceExampleController::on_init() {
  /*
   * Declare variables
   * Reserve memory
   * Declare node parameters
  */
  try {
    if (!get_node()->get_parameter("arm_id", arm_id_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }



  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_configure(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * TODO:
   * Initialise variables
   * Read parameters here
  */
  equilibrium_pose_d_ = 
    std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
    franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated));

  franka_robot_state_ =
    std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(arm_id_ + "/" + k_robot_state_interface_name));

  franka_robot_model_ =
    std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
        arm_id_ + "/" + k_robot_model_interface_name,
        arm_id_ + "/" + k_robot_state_interface_name));
  // arm_id_ = get_node()
  
  // initialise varrious variables
  init_robot_state_ = franka_msgs::msg::FrankaRobotState();
  franka_robot_state_->get_values_as_message(init_robot_state_);

  // retrieve intial joint state
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(init_robot_state_.measured_joint_state.position.data());
  Eigen::Vector3d position(
    robot_state_.o_t_ee.pose.position.x,
    robot_state_.o_t_ee.pose.position.y,
    robot_state_.o_t_ee.pose.position.z);
  Eigen::Quaterniond orientation(
    robot_state_.o_t_ee.pose.orientation.w,
    robot_state_.o_t_ee.pose.orientation.x,
    robot_state_.o_t_ee.pose.orientation.y,
    robot_state_.o_t_ee.pose.orientation.z);
  Eigen::Affine3d init_transform = Eigen::Affine3d::Identity();
  init_transform.translation() = position;
  init_transform.rotate(orientation.toRotationMatrix());
  q_d_nullspace_ = q_init;

  position_d_ = init_transform.translation();
  orientation_d_ = init_transform.rotation();
  position_d_target_ = init_transform.translation();
  orientation_d_target_ = init_transform.rotation();

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_activate(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * Assign loaned command and state interfaces
  */
  // state interfaces
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  equilibrium_pose_d_->assign_loaned_state_interfaces(state_interfaces_);

  // command interfaces
  // franka_robot_state_->assign_loaned_command_interfaces(command_interfaces_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_deactivate(
  const rclcpp_lifecycle::State& previous_state) {
  /*
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

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (int i = 0; i < num_joints; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
       tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

} // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)

