cmake_minimum_required(VERSION 3.5)
project(franka_gripper)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Franka REQUIRED)

add_library(franka_gripper_node SHARED
        src/GripperActionServer.cpp)
target_link_libraries(franka_gripper_node Franka::Franka)
target_include_directories(franka_gripper_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


ament_target_dependencies(franka_gripper_node
        franka_msgs
        rclcpp
        rclcpp_action
        rclcpp_components
        Franka
        sensor_msgs
        std_srvs
        control_msgs
        )
rclcpp_components_register_node(franka_gripper_node PLUGIN "franka_gripper::GripperActionServer" EXECUTABLE gripper_action_server)
install(TARGETS
        franka_gripper_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)


if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
