#!/usr/bin/env python3

#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rclpy
from rclpy.node import Node
from time import sleep

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


class EquilibriumPosePublisher(Node):

    def __init__(self):
        super().__init__("equilibrium_pose_node")
        self.initial_pose_found = False
        self.marker_pose = PoseStamped()
        self.pose_pub = self.create_publisher(PoseStamped, "equilibrium_pose", 10)

        state_sub = self.create_subscription(FrankaState,
                                             "franka_state_controller/franka_states",
                                             self.franka_state_callback, 1)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.link_name = "panda_link0"  # rospy.get_param("~link_name")

        # self.fake_initial_pose()
        # Get initial pose for the interactive marker
        while not self.initial_pose_found:
            sleep(1)
        state_sub.destroy()

        self.server = InteractiveMarkerServer(self, "equilibrium_pose_marker")
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.link_name
        int_marker.scale = 0.3
        int_marker.name = "equilibrium_pose"
        int_marker.pose.position.z = 2.
        int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                                  "If you move the \nequilibrium "
                                  "pose the robot will follow it\n"
                                  "so be aware of potential collisions")
        int_marker.pose = self.marker_pose.pose

        # run pose publisher
        self.timer = self.create_timer(0.005, self.publisher_callback)

        # insert a box
        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 1.
        control.orientation.y = 0.
        control.orientation.z = 0.
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 1.
        control.orientation.y = 0.
        control.orientation.z = 0.
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 0.
        control.orientation.y = 1.
        control.orientation.z = 0.
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 0.
        control.orientation.y = 1.
        control.orientation.z = 0.
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 0.
        control.orientation.y = 0.
        control.orientation.z = 1.
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.
        control.orientation.x = 0.
        control.orientation.y = 0.
        control.orientation.z = 1.
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        self.server.insert(int_marker, feedback_callback=self.process_feedback)

        self.server.applyChanges()

    def fake_initial_pose(self):

        initial_quaternion = [1., 0., 0., 0.]
        self.marker_pose.pose.orientation.x = initial_quaternion[0]
        self.marker_pose.pose.orientation.y = initial_quaternion[1]
        self.marker_pose.pose.orientation.z = initial_quaternion[2]
        self.marker_pose.pose.orientation.w = initial_quaternion[3]
        self.marker_pose.pose.position.x = 0.
        self.marker_pose.pose.position.y = 0.
        self.marker_pose.pose.position.z = 1.
        self.initial_pose_found = True

    def publisher_callback(self):
        self.marker_pose.header.frame_id = self.link_name
        self.marker_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.marker_pose)

    def franka_state_callback(self, msg: FrankaState):
        self.marker_pose.pose.position.x = msg.o_t_ee.transform.translation.x
        self.marker_pose.pose.position.y = msg.o_t_ee.transform.translation.y
        self.marker_pose.pose.position.z = msg.o_t_ee.transform.translation.z
        self.marker_pose.pose.orientation = msg.o_t_ee.transform.rotation
        self.initial_pose_found = True

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                                         position_limits[0][1]]),
                                                    position_limits[0][0]])
            self.marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                                         position_limits[1][1]]),
                                                    position_limits[1][0]])
            self.marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                                         position_limits[2][1]]),
                                                    position_limits[2][0]])
            self.marker_pose.pose.orientation = feedback.pose.orientation
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = EquilibriumPosePublisher()

    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
