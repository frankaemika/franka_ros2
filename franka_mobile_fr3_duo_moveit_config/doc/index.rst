franka_mobile_fr3_duo_moveit_config
===================================

This package contains the MoveIt 2 configuration for the Mobile FR3 Duo.

Move Group
----------

There are three move groups called ``left_arm``, ``right_arm``, and ``mobile_base``,
plus one group called ``full_body`` that combines them all. There is also a move group
for the ``spine`` that you can use for planning and visualization, but it does not yet
support motion on the real hardware or in simulation.


Configuration Files
-------------------

In the ``config`` folder you will find:

* Motion planning configuration for the Mobile FR3 Duo (``moveit_defaults.json``)
* Joint limits (``joint_limits.yaml``)
* Kinematics solver configuration (``kinematics.yaml``)
* ROS 2 controller configuration (``mobile_fr3_duo_controllers.yaml``)
* MoveIt controller configuration (``moveit_controllers.yaml``)

Planning groups and collisions are defined in ``mobile_fr3_duo.srdf.xacro`` in the
``franka_description`` package.

Usage
-----

.. code-block:: shell

    colcon build --packages-up-to franka_mobile_fr3_duo_moveit_config
    ros2 launch franka_mobile_fr3_duo_moveit_config moveit.launch.py # for the real robot
    ros2 launch franka_mobile_fr3_duo_moveit_config moveit.launch.py simulate_in_gazebo:=true # gazebo simulation

.. figure:: ../../docs/assets/motion_planning_in_rviz.png
    :align: center
    :figclass: align-center

    Move the marker-handles to define your goal state. Then click ``Plan & Execute`` and observe the robot move.