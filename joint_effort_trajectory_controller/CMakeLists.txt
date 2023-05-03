cmake_minimum_required(VERSION 3.5)
project(joint_effort_trajectory_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

add_library(joint_effort_trajectory_controller SHARED
        src/joint_trajectory_controller.cpp
        src/trajectory.cpp
        )
target_include_directories(joint_effort_trajectory_controller PUBLIC include)
ament_target_dependencies(joint_effort_trajectory_controller
        angles
        builtin_interfaces
        controller_interface
        control_msgs
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        trajectory_msgs
        control_toolbox
        )

pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

install(DIRECTORY include/
        DESTINATION include
        )

install(TARGETS joint_effort_trajectory_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

ament_export_dependencies(
        controller_interface
        control_msgs
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        trajectory_msgs
)
ament_export_include_directories(
        include
)
ament_export_libraries(
        joint_effort_trajectory_controller
)
ament_package()
