cmake_minimum_required(VERSION 3.8)
project(g1_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(g1_control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)

add_library(${PROJECT_NAME}
  src/common/joint_map.cpp
  src/common/safety_checker.cpp
  src/arm/arm_sdk_controller.cpp
  src/loco/loco_client_wrapper.cpp
  src/loco/loco_controller.cpp
  src/manager/robot_mode_manager.cpp
  src/manager/g1_controller_node.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../example/src/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  g1_control_msgs
  rclcpp
  std_msgs
  unitree_api
  unitree_hg
)

add_executable(g1_controller_main src/apps/g1_controller_main.cpp)
target_link_libraries(g1_controller_main ${PROJECT_NAME})
ament_target_dependencies(g1_controller_main
  g1_control_msgs
  rclcpp
)

add_executable(g1_arm_keyboard_delta_teleop
  src/apps/g1_arm_keyboard_delta_teleop.cpp
)
target_link_libraries(g1_arm_keyboard_delta_teleop ${PROJECT_NAME})
ament_target_dependencies(g1_arm_keyboard_delta_teleop
  g1_control_msgs
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}
  g1_arm_keyboard_delta_teleop
  g1_controller_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  builtin_interfaces
  g1_control_msgs
  rclcpp
  std_msgs
  unitree_api
  unitree_hg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
