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

from math import pi
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

from rcl_interfaces.srv import GetParameters
import rclpy
import sensor_msgs.msg


def generate_test_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    move_to_start_controller_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'move_to_start_example_controller.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            load_gripper_parameter_name: load_gripper,
            use_fake_hardware_parameter_name: use_fake_hardware,
            fake_sensor_commands_parameter_name: fake_sensor_commands,
            use_rviz_parameter_name: use_rviz,
        }.items(),
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    robot_ip_parameter_name, description='Hostname or IP address of the robot.'
                ),
                DeclareLaunchArgument(
                    use_rviz_parameter_name,
                    default_value='false',
                    description='Visualize the robot in Rviz',
                ),
                DeclareLaunchArgument(
                    use_fake_hardware_parameter_name,
                    default_value='false',
                    description='Use fake hardware',
                ),
                DeclareLaunchArgument(
                    fake_sensor_commands_parameter_name,
                    default_value='false',
                    description=(
                        'Fake sensor commands. Only valid when'
                        f' {use_fake_hardware_parameter_name} is true'
                    ),
                ),
                DeclareLaunchArgument(
                    load_gripper_parameter_name,
                    default_value='true',
                    description=(
                        'Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'
                    ),
                ),
                move_to_start_controller_description,
                # Start test right away, no need to wait for anything
                ReadyToTest(),
            ]
        ),
        {
            'move_to_start_controller': move_to_start_controller_description,
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
        self.link_node = rclpy.create_node('controller_test_link')  # type: ignore

    def tearDown(self):
        self.link_node.destroy_node()

    def test_start_joint_positions(self, launch_service, move_to_start_controller, proc_output):
        self.joint_positions_goal = [0, -pi / 4, 0, -3 * pi / 4, 0, pi / 2, pi / 4]
        self.joint_positions = []
        self.process_finished = False

        total_joint_no = 7

        # decimal places in assertion
        ACCURACY = 1

        def _service_callback(msg):
            for joint_no in range(total_joint_no):
                if len(self.joint_positions) <= joint_no:
                    self.joint_positions.append(msg.position[joint_no])
                else:
                    self.joint_positions[joint_no] = msg.position[joint_no]

        sub = self.link_node.create_subscription(
            sensor_msgs.msg.JointState,
            'joint_states',
            _service_callback,
            10,
        )

        client = self.link_node.create_client(
            GetParameters, '/move_to_start_example_controller/get_parameters'
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.link_node.get_logger().info('Service not available, waiting...')

        request = GetParameters.Request()
        request.names = {'process_finished'}

        while not self.process_finished or len(self.joint_positions) < total_joint_no:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.link_node, future)
            self.process_finished = future.result().values[0].bool_value  # type: ignore

        self.link_node.destroy_subscription(sub)
        self.link_node.destroy_client(client)

        self.assertAlmostEqual(self.joint_positions_goal[0], self.joint_positions[0], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[1], self.joint_positions[1], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[2], self.joint_positions[2], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[3], self.joint_positions[3], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[4], self.joint_positions[4], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[5], self.joint_positions[5], ACCURACY)
        self.assertAlmostEqual(self.joint_positions_goal[6], self.joint_positions[6], ACCURACY)
