cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "action/Stop.action"
  "msg/Errors.msg"
  "msg/FrankaState.msg"
  "msg/GraspEpsilon.msg"
  "srv/SetCartesianImpedance.srv"
  "srv/SetCollisionBehavior.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetSimpleCollisionBehavior.srv"
  "srv/SetTransform.srv"
  "srv/SetLoad.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()