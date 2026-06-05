#include "g1_control_interface/manager/g1_controller_node.hpp"

namespace g1_control_interface {

namespace {

ArmModel ParseArmModel(const std::string& value) {
  return value == "G1ARM7" ? ArmModel::kG1Arm7 : ArmModel::kG1Arm5;
}

}  // namespace

G1ControllerNode::G1ControllerNode()
    : Node("g1_controller_node"),
      arm_controller_(this,
                      ParseArmModel(this->declare_parameter<std::string>(
                          "arm_model", "G1ARM5")),
                      0.02),
      loco_controller_(this),
      mode_manager_(arm_controller_, loco_controller_) {
  arm_command_sub_ = create_subscription<g1_control_msgs::msg::ArmJointCommand>(
      "/g1/command/arm_joints", rclcpp::QoS(1).best_effort(),
      [this](const g1_control_msgs::msg::ArmJointCommand::SharedPtr msg) {
        OnArmCommand(msg);
      });

  loco_command_sub_ = create_subscription<g1_control_msgs::msg::LocoCommand>(
      "/g1/command/loco", 10,
      [this](const g1_control_msgs::msg::LocoCommand::SharedPtr msg) {
        OnLocoCommand(msg);
      });

  arm_state_pub_ = create_publisher<g1_control_msgs::msg::ArmJointState>(
      "/g1/state/arm", 10);
  status_pub_ =
      create_publisher<g1_control_msgs::msg::ControllerStatus>(
          "/g1/state/controller_status", 10);
  status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    PublishStatus();
  });

  set_arm_pose_srv_ = create_service<g1_control_msgs::srv::SetArmPose>(
      "/g1/arm/set_pose",
      [this](const std::shared_ptr<g1_control_msgs::srv::SetArmPose::Request> request,
             std::shared_ptr<g1_control_msgs::srv::SetArmPose::Response> response) {
        ArmCommand command;
        command.joint_names = request->joint_names;
        command.positions = request->positions;
        command.velocities = request->velocities;
        command.kp = request->kp;
        command.kd = request->kd;
        command.duration_sec = request->duration_sec;
        command.hold = request->hold;
        response->success =
            arm_controller_.set_joint_targets(command, response->message);
      });

  set_loco_mode_srv_ = create_service<g1_control_msgs::srv::SetLocoMode>(
      "/g1/loco/set_mode",
      [this](const std::shared_ptr<g1_control_msgs::srv::SetLocoMode::Request> request,
             std::shared_ptr<g1_control_msgs::srv::SetLocoMode::Response> response) {
        std::string message;
        if (request->enable && request->requested_fsm_id != 0 &&
            request->requested_fsm_id != 500) {
          response->success = false;
          response->message =
              "Only fsm_id=500 is supported by this unified locomotion interface";
        } else {
          response->success = request->enable
                                  ? loco_controller_.enable(message)
                                  : loco_controller_.disable(message);
          response->message = message;
        }
        std::string cached_message;
        loco_controller_.get_cached_fsm(response->fsm_id, response->fsm_mode,
                                        cached_message);
      });

  stop_arm_srv_ = create_service<g1_control_msgs::srv::StopArm>(
      "/g1/arm/stop",
      [this](const std::shared_ptr<g1_control_msgs::srv::StopArm::Request>,
             std::shared_ptr<g1_control_msgs::srv::StopArm::Response> response) {
        double duration_sec = 2.0;
        this->get_parameter("arm_release_duration_sec", duration_sec);
        response->success =
            arm_controller_.release_control(duration_sec, response->message);
      });

  stop_loco_srv_ = create_service<g1_control_msgs::srv::StopLoco>(
      "/g1/loco/stop",
      [this](const std::shared_ptr<g1_control_msgs::srv::StopLoco::Request>,
             std::shared_ptr<g1_control_msgs::srv::StopLoco::Response> response) {
        response->success = loco_controller_.stop(response->message);
      });
}

void G1ControllerNode::OnArmCommand(
    const g1_control_msgs::msg::ArmJointCommand::SharedPtr msg) {
  ArmCommand command;
  command.joint_names = msg->joint_names;
  command.positions = msg->positions;
  command.deltas = msg->deltas;
  command.velocities = msg->velocities;
  command.kp = msg->kp;
  command.kd = msg->kd;
  command.duration_sec = rclcpp::Duration(msg->duration).seconds();
  command.is_delta = msg->is_delta;
  command.hold = msg->hold;

  std::string message;
  if (!arm_controller_.stream_joint_targets(command, message)) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

void G1ControllerNode::OnLocoCommand(
    const g1_control_msgs::msg::LocoCommand::SharedPtr msg) {
  LocoCommand command;
  command.vx = msg->vx;
  command.vy = msg->vy;
  command.omega = msg->omega;
  command.duration_sec = rclcpp::Duration(msg->duration).seconds();
  command.continuous = msg->continuous;
  command.enable = msg->enable;

  std::string message;
  if (!loco_controller_.command_velocity(command, message)) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

void G1ControllerNode::PublishStatus() {
  const auto arm_state = arm_controller_.current_state();
  g1_control_msgs::msg::ArmJointState arm_msg;
  arm_msg.header.stamp = now();
  arm_msg.joint_names = arm_state.joint_names;
  arm_msg.positions = arm_state.positions;
  arm_msg.velocities = arm_state.velocities;
  arm_msg.has_state = arm_state.has_state;
  arm_state_pub_->publish(arm_msg);

  const auto snapshot = mode_manager_.snapshot();
  g1_control_msgs::msg::ControllerStatus msg;
  msg.header.stamp = now();
  msg.lowstate_online = snapshot.lowstate_online;
  msg.arm_control_active = snapshot.arm_control_active;
  msg.loco_control_active = snapshot.loco_control_active;
  msg.fsm_id = snapshot.fsm_id;
  msg.fsm_mode = snapshot.fsm_mode;
  msg.arm_model = snapshot.arm_model;
  msg.message = snapshot.message;
  status_pub_->publish(msg);
}

}  // namespace g1_control_interface
