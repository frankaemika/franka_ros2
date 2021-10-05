cmake_minimum_required(VERSION 3.5)
project(franka_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Franka REQUIRED)


add_library(franka_hardware
        SHARED
        src/franka_hardware_interface.cpp
        src/robot.cpp include/franka_hardware/robot.hpp)
target_include_directories(
        franka_hardware
        PRIVATE
        include
)
ament_target_dependencies(
        franka_hardware
        hardware_interface
        Franka
        pluginlib
        rclcpp
        franka_msgs
)
pluginlib_export_plugin_description_file(hardware_interface franka_hardware.xml)


install(
        TARGETS franka_hardware
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)


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

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_hardware
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()
