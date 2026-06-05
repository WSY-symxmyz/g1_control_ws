cmake_minimum_required(VERSION 3.8)
project(g1_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointCommand.msg"
  "msg/ArmCommandDebug.msg"
  "msg/ArmJointState.msg"
  "msg/LocoCommand.msg"
  "msg/ControllerStatus.msg"
  "srv/SetArmPose.srv"
  "srv/SetLocoMode.srv"
  "srv/StopArm.srv"
  "srv/StopLoco.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
