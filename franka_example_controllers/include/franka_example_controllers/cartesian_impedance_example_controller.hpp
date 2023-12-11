#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian empedance example controller
*/
class CartesianImpedanceExampleController : public controller_interface::ControllerInterface {
public:

    /*
     * Interface:
     * Takes in the equilibrium position of the spring-mass-damper system that
     * the impedance controller implements, computes the torques and then sends
     * the joint torques to the corresponding interface
     *
     * QUE: What is the Torque command interface?
     * QUE: How do you retrieve the robot state?
     * QUE: How do you retrieve the robot model?
     */

    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
    const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
    const override;

    /* The only thing that needs to be implemented for ros2_control interface
     */
    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    /* The following is all ROS2's lifecycle related stuff
     */
    CallbackReturn on_init() override;
    CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    /* What Do I need for this controller
     *
    */


private:
    std::string arm_id_;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};


};
} // namespace franka_example_controllers
