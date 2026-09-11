^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_gazebo_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* feat: new package holding the Gazebo (gz_ros2_control) system plugin
  ``franka_gazebo_hardware/GazeboGravityCompensationSystem``. The plugin computes
  pinocchio-based gravity torque and injects it on the effort-controlled Franka
  arm joints so the zero-torque example controllers behave as on the real robot
  instead of collapsing under their own weight. Split out of ``franka_gazebo_bringup``.
* feat: export the 16 ``<i>/cartesian_pose_state`` interfaces from the Gazebo hardware
  interface so Cartesian pose/impedance controllers can activate in simulation.
* feat: report the estimated external wrench in the stiffness frame K. ``K_F_ext_hat_K``
  and the ``_tcp`` force/torque interfaces are expressed in frame K, while
  ``O_F_ext_hat_K`` stays in the base frame. The wrench sign follows the reaction
  convention: a push in +x reads a measured external force of -x.
