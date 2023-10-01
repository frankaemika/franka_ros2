#  Copyright (c) 2023 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import time
import unittest

from franka_msgs.action import Move  # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.action import ActionClient
import sensor_msgs.msg


def generate_test_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    arm_parameter_name = 'arm_id'
    joint_names_parameter_name = 'joint_names'
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    arm_id = LaunchConfiguration(arm_parameter_name)
    joint_names = LaunchConfiguration(joint_names_parameter_name)

    default_joint_name_postfix = '_finger_joint'
    arm_default_argument = [
        '[',
        arm_id,
        default_joint_name_postfix,
        '1',
        ',',
        arm_id,
        default_joint_name_postfix,
        '2',
        ']',
    ]

    franka_gripper_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_gripper'),
                        'launch',
                        'gripper.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            use_fake_hardware_parameter_name: use_fake_hardware,
            arm_parameter_name: arm_id,
            joint_names_parameter_name: joint_names,
        }.items(),
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    robot_ip_parameter_name, description='Hostname or IP address of the robot.'
                ),
                DeclareLaunchArgument(
                    use_fake_hardware_parameter_name,
                    default_value='false',
                    description=(
                        'Publish fake gripper joint states without connecting to a real gripper'
                    ),
                ),
                DeclareLaunchArgument(
                    arm_parameter_name,
                    default_value='panda',
                    description=(
                        'Name of the arm in the URDF file. This is used to generate the joint '
                        'names of the gripper.'
                    ),
                ),
                DeclareLaunchArgument(
                    joint_names_parameter_name,
                    default_value=arm_default_argument,
                    description='Names of the gripper joints in the URDF',
                ),
                franka_gripper_description,
                # Start test right away, no need to wait for anything
                ReadyToTest(),
            ],
        ),
        {
            'franka_gripper': franka_gripper_description,
        },
    )


# Test node
class TestStartJointPositions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.link_node = rclpy.create_node('gripper_test_link')  # type: ignore

    def tearDown(self):
        self.link_node.destroy_node()

    def test_gripper_position(self, launch_service, franka_gripper, proc_output):
        # max gripper width 0.08m
        self.gripper_positions_goal = [0.02, 0.05, 0.08]
        self.gripper_position = None
        self.gripper_speed = 1.0

        # decimal places in assertion
        ACCURACY = 2

        def _service_callback(msg):
            gripper_left_position = msg.position[0]
            gripper_right_position = msg.position[0]
            self.gripper_position = gripper_left_position + gripper_right_position

        sub = self.link_node.create_subscription(
            sensor_msgs.msg.JointState,
            '/panda_gripper/joint_states',
            _service_callback,
            10,
        )

        action_client = ActionClient(
            self.link_node, action_name='/panda_gripper/move', action_type=Move
        )

        while not action_client.wait_for_server(timeout_sec=1.0):
            self.link_node.get_logger().info('Action server not available, waiting...')

        goal_msg = Move.Goal()
        goal_msg.speed = self.gripper_speed

        for position in self.gripper_positions_goal:
            print('\n')
            self.link_node.get_logger().info(f'VALIDATING POSITION {position}:')
            goal_msg.width = position
            future = action_client.send_goal_async(goal_msg)

            """
            status 1 - accepted
            status 2 - executing
            status 3 - canceling
            status 4 - success
            status 5 - aborted
            status 6 - canceled
            """
            action_status = 0
            while action_status < 3:
                if future.done():
                    action_status = future.result().status  # type: ignore
                rclpy.spin_once(self.link_node)

            self.assertAlmostEqual(position, self.gripper_position, ACCURACY)  # type: ignore
            time.sleep(1)

        self.link_node.destroy_subscription(sub)
        action_client.destroy()
