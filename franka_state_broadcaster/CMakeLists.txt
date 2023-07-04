cmake_minimum_required(VERSION 3.16)
project(franka_state_broadcaster LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    control_msgs
    controller_interface
    franka_msgs
    franka_semantic_components
    generate_parameter_library
    pluginlib
    rclcpp_lifecycle
    rcutils
    realtime_tools
    sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(franka_state_broadcaster_parameters
                           src/franka_state_broadcaster_parameters.yaml)

add_library(franka_state_broadcaster SHARED src/franka_state_broadcaster.cpp)
target_compile_features(franka_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(
  franka_state_broadcaster
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/franka_state_broadcaster>)
target_link_libraries(franka_state_broadcaster
                      PUBLIC franka_state_broadcaster_parameters)
ament_target_dependencies(franka_state_broadcaster PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(franka_state_broadcaster
                           PRIVATE "FRANKA_STATE_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface
                                         franka_state_broadcaster.xml)

install(DIRECTORY include/ DESTINATION include/franka_state_broadcaster)
install(
  TARGETS franka_state_broadcaster franka_state_broadcaster_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_franka_state_broadcaster
    test/test_load_franka_state_broadcaster.cpp)
  target_link_libraries(test_load_franka_state_broadcaster
    franka_state_broadcaster)
  ament_target_dependencies(
    test_load_franka_state_broadcaster controller_manager hardware_interface
    ros2_control_test_assets)
endif()

ament_export_include_directories(include)
ament_export_libraries(franka_state_broadcaster franka_state_broadcaster_parameters)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
