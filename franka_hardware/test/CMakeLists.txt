find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}_test franka_hardware_interface_test.cpp)
target_include_directories(${PROJECT_NAME}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME}_test  ${PROJECT_NAME})


ament_add_gmock(${PROJECT_NAME}_command_interface_test franka_hardware_cartesian_command_interface_test.cpp)
target_include_directories(${PROJECT_NAME}_command_interface_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME}_command_interface_test  ${PROJECT_NAME})

ament_add_gmock(${PROJECT_NAME}_robot_test franka_robot_test.cpp)
target_include_directories(${PROJECT_NAME}_robot_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME}_robot_test  ${PROJECT_NAME})
