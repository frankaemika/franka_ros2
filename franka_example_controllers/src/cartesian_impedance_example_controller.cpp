#include <franka_example_controllers/cartesian_impedance_example_controller.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::command_interface_configuration() const {
  /*
   * TODO:
   * Define command interfaces here
  */
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; i++) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i), + "/effort");
  }
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

  // Get current robot state: needs state handle
  // get coriolis
  
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

} // namespace franka_example_controllers

