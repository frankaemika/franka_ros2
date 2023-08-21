cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "msg/Errors.msg"
  "msg/GraspEpsilon.msg"
  "msg/FrankaRobotState.msg"
  "srv/SetCartesianStiffness.srv"
  "srv/SetTCPFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointStiffness.srv"
  "srv/SetStiffnessFrame.srv"
  "srv/SetLoad.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()