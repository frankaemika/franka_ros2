// Copyright (c) 2026 Franka Robotics GmbH
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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestLoadControllers : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void expectControllerLoads(const std::string& controller_name,
                             const std::string& plugin_name) {
    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::Logger logger = rclcpp::get_logger("load_controller");

    controller_manager::ControllerManager cm(
        std::make_unique<hardware_interface::ResourceManager>(
            ros2_control_test_assets::minimal_robot_urdf,
            std::make_shared<rclcpp::Clock>(), logger),
        executor, "test_cm_" + controller_name);

    auto response = cm.load_controller(controller_name, plugin_name);

    ASSERT_NE(response, nullptr);
  }
};

TEST_F(TestLoadControllers, LoadsGravityCompensationExampleController) {
  expectControllerLoads("test_gravity_compensation_example_controller",
                        "franka_example_controllers/"
                        "GravityCompensationExampleController");
}

TEST_F(TestLoadControllers, LoadsJointImpedanceExampleController) {
  expectControllerLoads("test_joint_impedance_example_controller",
                        "franka_example_controllers/"
                        "JointImpedanceExampleController");
}

TEST_F(TestLoadControllers, LoadsFr3DuoJointImpedanceExampleController) {
  expectControllerLoads("test_fr3_duo_joint_impedance_example_controller",
                        "franka_example_controllers/"
                        "Fr3DuoJointImpedanceExampleController");
}

TEST_F(TestLoadControllers, LoadsMobileFr3DuoJointImpedanceExampleController) {
  expectControllerLoads("test_mobile_fr3_duo_joint_impedance_example_controller",
                        "franka_example_controllers/"
                        "MobileFr3DuoJointImpedanceExampleController");
}

TEST_F(TestLoadControllers, LoadsJointVelocityExampleController) {
  expectControllerLoads("test_joint_velocity_example_controller",
                        "franka_example_controllers/JointVelocityExampleController");
}

TEST_F(TestLoadControllers, LoadsJointPositionExampleController) {
  expectControllerLoads("test_joint_position_example_controller",
                        "franka_example_controllers/JointPositionExampleController");
}

TEST_F(TestLoadControllers, LoadsCartesianVelocityExampleController) {
  expectControllerLoads("test_cartesian_velocity_example_controller",
                        "franka_example_controllers/CartesianVelocityExampleController");
}

TEST_F(TestLoadControllers, LoadsCartesianPoseExampleController) {
  expectControllerLoads("test_cartesian_pose_example_controller",
                        "franka_example_controllers/CartesianPoseExampleController");
}

TEST_F(TestLoadControllers, LoadsJointImpedanceWithIkExampleController) {
  expectControllerLoads("test_joint_impedance_with_ik_example_controller",
                        "franka_example_controllers/"
                        "JointImpedanceWithIKExampleController");
}

TEST_F(TestLoadControllers, LoadsCartesianElbowExampleController) {
  expectControllerLoads("test_cartesian_elbow_example_controller",
                        "franka_example_controllers/CartesianElbowExampleController");
}

TEST_F(TestLoadControllers, LoadsCartesianOrientationExampleController) {
  expectControllerLoads("test_cartesian_orientation_example_controller",
                        "franka_example_controllers/"
                        "CartesianOrientationExampleController");
}

TEST_F(TestLoadControllers, LoadsElbowExampleController) {
  expectControllerLoads("test_elbow_example_controller",
                        "franka_example_controllers/ElbowExampleController");
}

TEST_F(TestLoadControllers, LoadsMoveToStartExampleController) {
  expectControllerLoads("test_move_to_start_example_controller",
                        "franka_example_controllers/MoveToStartExampleController");
}

TEST_F(TestLoadControllers, LoadsModelExampleController) {
  expectControllerLoads("test_model_example_controller",
                        "franka_example_controllers/ModelExampleController");
}

TEST_F(TestLoadControllers, LoadsGripperExampleController) {
  expectControllerLoads("test_gripper_example_controller",
                        "franka_example_controllers/GripperExampleController");
}

TEST_F(TestLoadControllers, LoadsCartesianImpedanceExampleController) {
  expectControllerLoads("test_cartesian_impedance_example_controller",
                        "franka_example_controllers/"
                        "CartesianImpedanceExampleController");
}
