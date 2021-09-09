cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
  "srv/SetEEFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetKFrame.srv"
  "srv/SetLoad.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
