#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclpp/rclpp.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian empedance example controller
*/
class CartesianImpedanceExampleController : public controller_interface::ControllerInterface {
public:

    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
    const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
    const override;

    controller_interface::return_type update(const rclpp::Time& time,
                                             const rclpp::Duration& period) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(
        const rclpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(
        const rclpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(
        const rclpp_lifecycle::State& previous_state) override;

};
}
