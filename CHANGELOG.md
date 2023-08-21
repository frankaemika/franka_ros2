# Changelog

## 0.1.1 - 2023-08-21

Requires libfranka >= 0.11.0, required ROS 2 Humble

* franka\_hardware: uses updated libfranka version providing the possibility to have the control loop on the ROS side

## 0.1.0 - 2023-07-28

Requires libfranka >= 0.10.0, required ROS 2 Humble

* franka\_bringup: franka_robot_state broadcaster added to franka.launch.py.
* franka\_example\_controllers: model printing read onyl controller implemented
* franka\_robot\_model: semantic component to access robot model parameters.
* franka\_msgs: franka robot state msg added
* franka\_robot\_state: broadcaster publishes robot state.

### Added

* CI tests in Jenkins.
* joint\_effort\_trajectory\_controller package that contains a version of the
 joint\_trajectory\_controller that can use the torque interface.
 [See this PR](https://github.com/ros-controls/ros2_controllers/pull/225)
* franka\_bringup package that contains various launch files to start controller examples or Moveit2.
* franka\_moveit\_config package that contains a minimal moveit config to control the robot.
* franka\_example\_controllers package that contains some example controllers to use.
* franka\_hardware package that contains a plugin to access the robot.
* franka\_msgs package that contains common message, service and action type definitions.
* franka\_description package that contains all meshes and xacro files.
* franka\_gripper package that offers action and service interfaces to use the Franka Hand gripper.

### Fixed

* franka\_hardware Fix the mismatched joint state interface type logger error message.