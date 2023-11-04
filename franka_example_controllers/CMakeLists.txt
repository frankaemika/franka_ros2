cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(generate_parameter_library)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/gravity_compensation_example_controller.cpp
        src/joint_impedance_example_controller.cpp
        src/joint_velocity_example_controller.cpp
        src/joint_position_example_controller.cpp
        src/cartesian_velocity_example_controller.cpp
        src/elbow_example_controller.cpp
        src/model_example_controller.cpp
        src/move_to_start_example_controller.cpp
        src/motion_generator.cpp)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        franka_semantic_components
)

generate_parameter_library(franka_example_controllers_parameters src/model_example_controller_parameters.yaml)

target_link_libraries(${PROJECT_NAME} franka_example_controllers_parameters)

pluginlib_export_plugin_description_file(
        controller_interface franka_example_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
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
    find_package(ament_cmake_gmock REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
    ament_add_gmock(${PROJECT_NAME}_test
        test/test_load_gravity_compensation_controller.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE include)
    ament_target_dependencies(${PROJECT_NAME}_test
        controller_manager
        ros2_control_test_assets
    )

    ament_add_gmock(${PROJECT_NAME}_test_load_move_to_start
        test/test_load_move_to_start_example_controller.cpp)
    target_include_directories(${PROJECT_NAME}_test_load_move_to_start PRIVATE include)
    ament_target_dependencies(${PROJECT_NAME}_test_load_move_to_start
        controller_manager
        ros2_control_test_assets
    )


    ament_add_gmock(${PROJECT_NAME}_gravity_test test/test_gravity_compensation_example.cpp)
    target_include_directories(${PROJECT_NAME}_gravity_test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}_gravity_test ${PROJECT_NAME})

    ament_add_gmock(${PROJECT_NAME}_move_to_start_test test/test_move_to_start_example_controller.cpp)
    target_include_directories(${PROJECT_NAME}_move_to_start_test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}_move_to_start_test ${PROJECT_NAME})

    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
)
ament_package()
