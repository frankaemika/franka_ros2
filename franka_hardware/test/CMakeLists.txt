find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Isolate ROS domain when tests are launched via colcon 
set(ROS_ISOLATED_TEST_RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")

add_library(franka_hardware_mocks INTERFACE)
target_include_directories(franka_hardware_mocks INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

ament_add_gmock(${PROJECT_NAME}_test
  franka_hardware_interface_test.cpp
  franka_action_server_test.cpp
  ptp_motion_test.cpp
  RUNNER "${ROS_ISOLATED_TEST_RUNNER}"
)

target_include_directories(${PROJECT_NAME}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  franka_hardware_mocks
)


ament_add_gmock(${PROJECT_NAME}_command_interface_test franka_hardware_cartesian_command_interface_test.cpp
  RUNNER "${ROS_ISOLATED_TEST_RUNNER}"
)
target_include_directories(${PROJECT_NAME}_command_interface_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(${PROJECT_NAME}_command_interface_test
  ${PROJECT_NAME}
  franka_hardware_mocks
)

ament_add_gmock(${PROJECT_NAME}_robot_test franka_robot_test.cpp
  RUNNER "${ROS_ISOLATED_TEST_RUNNER}"
)
target_include_directories(${PROJECT_NAME}_robot_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME}_robot_test
  ${PROJECT_NAME}
  franka_hardware_mocks
)

