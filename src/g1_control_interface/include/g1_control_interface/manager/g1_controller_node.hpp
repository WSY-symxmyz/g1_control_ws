#pragma once

#include "rclcpp/rclcpp.hpp"

#include "g1_control_msgs/msg/arm_joint_command.hpp"
#include "g1_control_msgs/msg/arm_joint_state.hpp"
#include "g1_control_msgs/msg/controller_status.hpp"
#include "g1_control_msgs/msg/loco_command.hpp"
#include "g1_control_msgs/srv/set_arm_pose.hpp"
#include "g1_control_msgs/srv/set_loco_mode.hpp"
#include "g1_control_msgs/srv/stop_arm.hpp"
#include "g1_control_msgs/srv/stop_loco.hpp"
#include "g1_control_interface/arm/arm_sdk_controller.hpp"
#include "g1_control_interface/loco/loco_controller.hpp"
#include "g1_control_interface/manager/robot_mode_manager.hpp"

namespace g1_control_interface {

// Main ROS 2 entry point for the unified control interface. The node owns both
// arm and locomotion controllers and exposes a stable topic/service API to
// teleoperation, scripted behaviors, or learning pipelines.
class G1ControllerNode : public rclcpp::Node {
 public:
  G1ControllerNode();

 private:
  void OnArmCommand(const g1_control_msgs::msg::ArmJointCommand::SharedPtr msg);
  void OnLocoCommand(const g1_control_msgs::msg::LocoCommand::SharedPtr msg);
  void PublishStatus();

  ArmSdkController arm_controller_;
  LocoController loco_controller_;
  RobotModeManager mode_manager_;

  rclcpp::Subscription<g1_control_msgs::msg::ArmJointCommand>::SharedPtr
      arm_command_sub_;
  rclcpp::Subscription<g1_control_msgs::msg::LocoCommand>::SharedPtr
      loco_command_sub_;
  rclcpp::Publisher<g1_control_msgs::msg::ArmJointState>::SharedPtr
      arm_state_pub_;
  rclcpp::Publisher<g1_control_msgs::msg::ControllerStatus>::SharedPtr
      status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Service<g1_control_msgs::srv::SetArmPose>::SharedPtr set_arm_pose_srv_;
  rclcpp::Service<g1_control_msgs::srv::SetLocoMode>::SharedPtr
      set_loco_mode_srv_;
  rclcpp::Service<g1_control_msgs::srv::StopArm>::SharedPtr stop_arm_srv_;
  rclcpp::Service<g1_control_msgs::srv::StopLoco>::SharedPtr stop_loco_srv_;
};

}  // namespace g1_control_interface
