franka_hardware
===============

.. important::
    Breaking changes as of 0.1.14 release: ``franka_hardware`` robot_state and robot_model will be prefixed by the ``robot_type``.

        - ``panda/robot_model  -> ${robot_type}/robot_model``
        - ``panda/robot_state  -> ${robot_type}/robot_state``

    There is no change with the state and command interfaces naming. They are prefixed with the joint names in the URDF.

Package Overview
----------------

This package contains the ``franka_hardware`` plugin needed for `ros2_control <https://control.ros.org/jazzy/index.html>`_.
The plugin is loaded from the URDF of the robot and passed to the controller manager via the robot description.

Hardware Interfaces
--------------------

The hardware plugin provides for each joint:

* a ``position state interface`` that contains the measured joint position.
* a ``velocity state interface`` that contains the measured joint velocity.
* an ``effort state interface`` that contains the measured link-side joint torques.
* an ``initial_position state interface`` that contains the initial joint position of the robot.
* an ``effort command interface`` that contains the desired joint torques without gravity.
* a  ``position command interface`` that contains the desired joint position.
* a  ``velocity command interface`` that contains the desired joint velocity.

Additional State Interfaces
---------------------------

In addition to joint interfaces, the hardware plugin provides:

* a ``franka_robot_state`` that contains the robot state information, `franka_robot_state <https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_msgs/msg/FrankaRobotState.msg>`_.
* a ``franka_robot_model_interface`` that contains the pointer to the model object.
* a ``ForceTorqueSensor`` (``<arm_prefix><robot_type>_tcp``) that exposes the estimated
  external wrench in the stiffness frame (``K_F_ext_hat_K`` from libfranka) as six state
  interfaces: ``force.x``, ``force.y``, ``force.z``, ``torque.x``, ``torque.y``, ``torque.z``.

.. important::
    ``franka_robot_state`` and ``franka_robot_model_interface`` state interfaces should not be used directly from hardware state interface.
    Rather, they should be utilized by the :doc:`franka_semantic_components <../../franka_semantic_components/doc/index>` interface.

    The ``ForceTorqueSensor`` interfaces follow the standard ``ros2_control`` sensor convention
    and can be consumed directly via the ``semantic_components::ForceTorqueSensor`` component
    in any controller (e.g. the admittance controller) without requiring a topic bridge.
    See the gravity compensation example controller for usage.

Configuration
-------------

The IP of the robot is read over a parameter from the URDF.

ros2_control Macro Library
--------------------------

This package owns the ``ros2_control`` xacro macro library used to declare hardware interfaces
for all Franka robot configurations. The macros live in ``franka_hardware/ros2_control/``:

* ``franka_ros2_control_macros.xacro`` — shared building blocks (``configure_arm_joints``,
  ``configure_finger_joint``, ``configure_steering_joint``, ``configure_driving_joint``,
  ``general_purpose_io``, ``cartesian_velocity_io``, ``cartesian_pose_loop``, etc.)
* ``franka_arm.ros2_control.xacro`` — single-arm configuration
* ``tmrv0_2.ros2_control.xacro`` — standalone TMR base

These are composed with ``franka_description`` robot models via thin wrappers in
``franka_bringup/urdf/`` to produce complete robot descriptions with hardware interfaces.

The gazebo-only dual-arm descriptions (``fr3_duo.ros2_control.xacro`` and
``mobile_fr3_duo.ros2_control.xacro``) live in ``franka_gazebo_bringup/urdf/``, the package
that owns and consumes them. They reuse the shared building blocks above via
``$(find franka_hardware)/ros2_control/franka_ros2_control_macros.xacro``.

Error Recovery
--------------

Previously, FCI errors caused the entire launch process to exit. A
``franka::ControlException`` can now be recovered in place without restarting
the ``ros2_control_node`` process.

Behavior on Error
^^^^^^^^^^^^^^^^^

When a ``franka::ControlException`` occurs (e.g. a reflex triggered by a
collision or a violated joint limit), the hardware plugin:

1. Logs and latches the fault.
2. Returns ``OK`` from ``read()`` without refreshing the robot state
   (exported interfaces keep the pre-fault / frozen sample).
3. Returns ``DEACTIVATE`` from the next ``write()``.

Observable sequence from that point:

4. One controller-manager update cycle may still publish that pre-fault /
   frozen sample after ``read()`` latches (broadcasters remain lifecycle-active).
5. The ``controller_manager`` then transitions the hardware component to the
   **inactive** state and runs its deactivation callback inline in the
   controller-manager update cycle. The stop blocks that update thread for
   approximately two seconds; controller updates and topic publication pause
   for that duration, and no new ``RobotState`` is produced or required.
6. After the block, inactive-state reads resume and publish the live reflex
   state.

Cycle-overrun warnings from the controller manager are therefore expected during
this one-time recovery step. Do not command the robot while it is transitioning.

The ``robot_state_broadcaster`` and ``joint_state_broadcaster`` remain
lifecycle-active across the reflex, so they do not need re-activation afterward.

After the approximately two-second braking stop completes, the hardware component
is inactive. The deactivation callback has cleared the fault latch and reset the
active control interface. ``ros2_control`` continues calling ``read()`` for
inactive hardware, so inactive-state reads resume and publish the live reflex
state: ``franka_msgs/msg/FrankaRobotState`` reports ``ROBOT_MODE_REFLEX`` in
``robot_mode`` and provides the active and motion-ending errors in
``current_errors`` and ``last_motion_errors``.

The ``ros2_control_node`` process **stays alive** — no restart is needed.

This recovery path applies only to ``franka::ControlException``. A
``franka::NetworkException`` still returns ``ERROR`` from the hardware component
and deactivates the state broadcasters.

Recovery Steps
^^^^^^^^^^^^^^

After an FCI error the following three steps must be performed **in order**:

**Step 1 — Clear the robot error**

Call the error recovery action exposed by the ``franka_hardware`` action server.
This invokes ``libfranka``'s ``automaticErrorRecovery()`` to reset the robot's
error state.

.. code-block:: bash

   # Single-arm (no arm prefix):
   ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}

   # Dual-arm (example for left arm with prefix "left_"):
   ros2 action send_goal /left/action_server/error_recovery franka_msgs/action/ErrorRecovery {}

**Step 2 — Re-activate the hardware component**

Transition the hardware component back to the *active* state so it reconnects
the control loop.

.. code-block:: bash

   # Single-arm:
   ros2 control set_hardware_component_state FrankaHardwareInterface active

   # Dual-arm (left arm):
   ros2 control set_hardware_component_state left_FrankaHardwareInterface active

.. tip::
   You can discover the hardware component name at runtime with:

   .. code-block:: bash

      ros2 control list_hardware_components

**Step 3 — Re-activate the command controller**

.. code-block:: bash

   ros2 control switch_controllers --activate <controller_name>

The ``robot_state_broadcaster`` and ``joint_state_broadcaster`` stay active across
the reflex and do not need to be re-activated. Re-activate only the command
controller that should resume commanding the robot.

.. warning::
   The order matters. The robot error must be cleared (Step 1) before the
   hardware component can re-activate (Step 2), and the hardware component must
   be active before controllers can be activated (Step 3). Attempting Step 2
   before Step 1 fails the activation and logs that the robot error must be cleared.

Action Server Topics
^^^^^^^^^^^^^^^^^^^^

The error recovery action topic follows the arm prefix configured in the URDF:

+-----------------------------+---------------------------------------------+
| Configuration               | Action topic                                |
+=============================+=============================================+
| Single-arm (no prefix)      | ``/action_server/error_recovery``            |
+-----------------------------+---------------------------------------------+
| Dual-arm, left (``left_``)  | ``/left/action_server/error_recovery``       |
+-----------------------------+---------------------------------------------+
| Dual-arm, right (``right_``)| ``/right/action_server/error_recovery``      |
+-----------------------------+---------------------------------------------+

Usage with Controllers
----------------------

Controllers can access these interfaces through the standard ros2_control framework. For examples of how to use these interfaces in practice, see the :doc:`franka_example_controllers <../../franka_example_controllers/doc/index>` package.