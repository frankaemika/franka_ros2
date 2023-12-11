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
  */
}

controller_interface::InterfaceConfiguration
  CartesianImpedanceExampleController::state_interface_configuration() const {
  /*
   * TODO:
  */
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
  */
}

CallbackReturn CartesianImpedanceExampleController::on_configure(
  const rclcpp_lifecycle::State& previous_state) {
  /*
   * TODO: Intitialise something here as well
   * This is where you instantiate objects
  */
}
}

