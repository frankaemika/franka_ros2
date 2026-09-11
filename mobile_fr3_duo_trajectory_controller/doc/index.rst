mobile_fr3_duo_trajectory_controller
====================================

Package Overview
----------------

This package contains the controller used to execute trajectories on the Mobile FR3 Duo, both
in simulation and on real hardware.

Controllers
-----------

The package provides:

* ``mobile_fr3_duo_trajectory_controller``: a controller derived from the `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_. The key differences are:
    - tailored to control the full ``mobile_fr3_duo_v02`` via the joint torque command interface for the arms, and Cartesian velocity commands for the ``tmrv0_2``
    - intended to be used together with MoveIt 2. For more information and an example launch file, see the `franka_mobile_fr3_duo_moveit_config` package
    - provides a ``FollowJointTrajectory`` action server, but for the ``tmrv0_2``, it converts the velocities of the joints corresponding to the virtual planar joint used for planning to Cartesian velocity commands for the mobile base
    - the arms follow a simple joint impedance control law with damping on the filtered joint velocities. Configure this behavior with the ``k_gains`` and ``d_gains`` entries in the YAML file passed to the ``controller_manager``