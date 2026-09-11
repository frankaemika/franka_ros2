^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* feat: host the ros2_control xacro macro library and per-robot ros2_control configs (moved from franka_description); composition wrappers added in franka_bringup
* feat: add ForceTorqueSensor to the ros2_control xacro exposing K_F_ext_hat_K (force.x/y/z, torque.x/y/z)
* fix: recover from a ``franka::ControlException`` without restarting the ROS 2 control node;
  keep state broadcasters lifecycle-active; one update cycle may publish the pre-fault /
  frozen sample after ``read()`` latches, then the controller-manager update thread blocks
  for the approximately two-second braking stop and topic publication pauses; after the
  block, inactive-state reads resume and publish the live reflex state; re-activate only
  the command controller, and stop the robot once during recovery

1.0.0 (2025-01-22)
------------------
