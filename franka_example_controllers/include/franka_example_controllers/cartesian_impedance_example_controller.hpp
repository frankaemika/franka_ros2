#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The Cartesian Impedance example controller
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

    using vector7d = Eigen::Matrix<double, 7, 1>;
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
    void UpdateJointStates();
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d)
    std::string arm_id_;
    const int num_joints = 7;

    // Interface for Panda model for dynamics and kinematics
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    // Interface for Desired equilibrium pose of the eef
    std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> equilibrium_pose_d_;

    // Interface for Robot state
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

    // message object that takes in robot state
    std::unique_ptr<franka_msgs::msg::FrankaRObotState> robot_state_;

    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};

    const double delta_tau_max_{1.0}


};
} // namespace franka_example_controllers
