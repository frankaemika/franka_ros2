franka_robot_state_broadcaster
==============================

This package provides the read-only ``franka_robot_state_broadcaster`` controller.

Functionality
-------------

The broadcaster publishes the Franka robot state on ``~/robot_state``. Without a
namespace, this resolves to ``/franka_robot_state_broadcaster/robot_state``.

``update()`` runs on the controller manager thread: it reads the hardware
``RealtimeThreadSafeBox<franka::RobotState>``, builds the ROS message into an
``AsyncBuffer``, and returns. A dedicated publish thread drains that buffer and
performs all DDS publishes, so publish cost does not sit between robot state
arrival and command egress. Keep ``is_async: false`` (the default) so this
controller shares the CM thread with other readers of the same state box.

This controller is spawned by ``franka.launch.py`` in ``franka_bringup``. Therefore,
any setup that launches ``franka.launch.py`` also publishes the robot state topic.

Usage
-----

The robot state broadcaster is automatically started when you launch the robot using:

.. code-block:: shell

    ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

Parameters
----------

.. list-table::
    :header-rows: 1

    * - Name
      - Type
      - Default
      - Description
    * - ``robot_type``
      - string
      - ``fr3``
      - Robot type used to resolve the state interface names, for example ``fr3/...``.
    * - ``arm_prefix``
      - string
      - ``''``
      - Optional prefix added before ``robot_type`` when resolving state interface names in multi-arm setups.
    * - ``convenience_publish_rate``
      - int
      - ``1000``
      - Publish rate in Hz for convenience topics. The full robot state always publishes at the controller update rate (1 kHz). Set this lower, for example to ``100``, to reduce bandwidth for convenience topics while keeping the full state at 1 kHz.

Example in ``controllers.yaml``:

.. code-block:: yaml

    franka_robot_state_broadcaster:
      ros__parameters:
        convenience_publish_rate: 100

Published Topics
----------------

Full robot state (reliable QoS):

.. note::

    ``ros2 bag record`` adopts the QoS offered by the publisher, so recordings of these
    topics are best_effort as well and may omit samples if the writer cannot keep up.
    Requesting reliable on the subscriber side does not help, because that profile is
    incompatible with these publishers and would record nothing at all.

Full robot state:

* ``~/robot_state`` (franka_msgs/FrankaRobotState): Complete robot state, published at
  the controller update rate (1 kHz).

Convenience topics:

The following topics are published at the rate configured by
``convenience_publish_rate``.

* ``~/current_pose`` (geometry_msgs/PoseStamped): Measured end-effector pose in base frame.
* ``~/last_desired_pose`` (geometry_msgs/PoseStamped): Last desired end-effector pose.
* ``~/desired_end_effector_twist`` (geometry_msgs/TwistStamped): Desired end-effector twist.
* ``~/measured_joint_states`` (sensor_msgs/JointState): Measured joint positions, velocities, and torques.
* ``~/desired_joint_states`` (sensor_msgs/JointState): Desired joint positions, velocities, and torques.
* ``~/external_joint_torques`` (sensor_msgs/JointState): Estimated external joint torques.
* ``~/external_wrench_in_base_frame`` (geometry_msgs/WrenchStamped): Estimated external wrench in base frame.
* ``~/external_wrench_in_stiffness_frame`` (geometry_msgs/WrenchStamped): Estimated external wrench in stiffness frame.

.. important::

    Subscribers for these convenience topics must request **best_effort** reliability. The ``rclcpp`` and ``rclpy``
    defaults are **reliable**, which is **not compatible** with these publishers: the
    endpoints never match, no messages arrive, and nothing is reported as an error. This
    applies to custom nodes, to ``ros2 topic echo`` (use
    ``--qos-reliability best_effort``) and to rviz2 (set Reliability Policy to
    "Best Effort" in the display properties).

Integration
-----------

This broadcaster integrates with the
:doc:`franka_semantic_components <../../franka_semantic_components/doc/index>` package
to provide safe access to robot state information for controllers and other nodes.