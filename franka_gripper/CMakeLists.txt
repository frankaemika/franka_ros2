cmake_minimum_required(VERSION 3.5)
project(franka_gripper)

find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Franka REQUIRED)

add_library(gripper_server SHARED
        src/gripper_action_server.cpp)
target_link_libraries(gripper_server Franka::Franka)
target_include_directories(gripper_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gripper_server
        franka_msgs
        rclcpp
        rclcpp_action
        rclcpp_components
        Franka
        sensor_msgs
        std_srvs
        control_msgs)
rclcpp_components_register_node(gripper_server PLUGIN "franka_gripper::GripperActionServer" EXECUTABLE franka_gripper_node)
install(TARGETS
        gripper_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
endif()

ament_package()
