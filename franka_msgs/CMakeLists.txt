cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ErrorRecovery.action"
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "action/Stop.action"
  "msg/Errors.msg"
  "msg/FrankaState.msg"
  "msg/GraspEpsilon.msg"
  "srv/SetCartesianImpedance.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetTransform.srv"
  "srv/SetLoad.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()