#include "g1_control_interface/arm/arm_sdk_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "g1_control_interface/common/safety_checker.hpp"

namespace g1_control_interface {

namespace {

std::vector<double> Filled(size_t size, double value) {
  return std::vector<double>(size, value);
}

double Clamp(double value, double min_value, double max_value) {
  return std::clamp(value, min_value, max_value);
}

}  // namespace

ArmSdkController::ArmSdkController(rclcpp::Node* node, ArmModel model,
                                   double control_period_sec)
    : node_(node), arm_model_(model), joint_map_(model) {
  LoadParameters(control_period_sec);

  const auto joint_count = joint_map_.ordered_joint_names().size();
  current_positions_ = Filled(joint_count, 0.0);
  current_velocities_ = Filled(joint_count, 0.0);
  target_positions_ = Filled(joint_count, 0.0);
  commanded_positions_ = Filled(joint_count, 0.0);
  kp_ = Filled(joint_count, default_kp_);
  kd_ = Filled(joint_count, default_kd_);
  for (size_t i = 0; i < joint_map_.ordered_joint_names().size(); ++i) {
    const auto& name = joint_map_.ordered_joint_names()[i];
    if (name == "waist_yaw" || name == "waist_roll" ||
        name == "waist_pitch") {
      kp_[i] = default_kp_ * waist_kp_scale_;
      kd_[i] = default_kd_ * waist_kd_scale_;
    }
  }
  limits_ = BuildJointLimits();

  publisher_ =
      node_->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);

  const auto debug_qos = rclcpp::QoS(1).best_effort();
  debug_publisher_ =
      node_->create_publisher<g1_control_msgs::msg::ArmCommandDebug>(
          "/g1/debug/arm_command", debug_qos);

  subscription_ = node_->create_subscription<unitree_hg::msg::LowState>(
      "/lowstate", 10,
      [this](const unitree_hg::msg::LowState::SharedPtr msg) {
        OnLowState(msg);
      });

  control_thread_ = std::thread([this]() { ControlLoop(); });
}

ArmSdkController::~ArmSdkController() {
  stop_control_thread_.store(true);
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void ArmSdkController::LoadParameters(double control_period_sec) {
  const std::vector<std::string> default_arm5 = {
      "left_shoulder_pitch",  "left_shoulder_roll",  "left_shoulder_yaw",
      "left_elbow_pitch",    "left_elbow_roll",     "right_shoulder_pitch",
      "right_shoulder_roll", "right_shoulder_yaw",  "right_elbow_pitch",
      "right_elbow_roll",    "waist_yaw",           "waist_roll",
      "waist_pitch"};
  const std::vector<std::string> default_arm7 = {
      "left_shoulder_pitch",  "left_shoulder_roll",  "left_shoulder_yaw",
      "left_elbow",          "left_wrist_roll",     "left_wrist_pitch",
      "left_wrist_yaw",      "right_shoulder_pitch", "right_shoulder_roll",
      "right_shoulder_yaw",  "right_elbow",         "right_wrist_roll",
      "right_wrist_pitch",   "right_wrist_yaw",     "waist_yaw",
      "waist_roll",         "waist_pitch"};

  control_period_sec_ = node_->declare_parameter<double>(
      "arm_control_period_sec", control_period_sec);
  max_delta_per_command_rad_ = node_->declare_parameter<double>(
      "arm_servo_max_delta_per_command_rad", 0.05);
  max_abs_step_per_command_rad_ = node_->declare_parameter<double>(
      "arm_servo_max_abs_step_per_command_rad", 0.05);
  max_position_step_rad_ =
      node_->declare_parameter<double>("arm_max_position_step_rad", 0.01);
  lowstate_fresh_timeout_sec_ = node_->declare_parameter<double>(
      "arm_lowstate_fresh_timeout_sec", 0.10);
  release_duration_sec_ =
      node_->declare_parameter<double>("arm_release_duration_sec", 2.0);
  default_kp_ = node_->declare_parameter<double>("arm_default_kp", 60.0);
  default_kd_ = node_->declare_parameter<double>("arm_default_kd", 1.5);
  waist_kp_scale_ =
      node_->declare_parameter<double>("arm_waist_kp_scale", 4.0);
  waist_kd_scale_ =
      node_->declare_parameter<double>("arm_waist_kd_scale", 4.0);

  const auto default_names =
      arm_model_ == ArmModel::kG1Arm7 ? default_arm7 : default_arm5;
  const auto configured_names = node_->declare_parameter<std::vector<std::string>>(
      "arm_joint_names", default_names);
  joint_map_ = JointMap(configured_names);
}

std::vector<JointLimit> ArmSdkController::BuildJointLimits() const {
  const auto names = node_->declare_parameter<std::vector<std::string>>(
      "joint_limit_names", joint_map_.ordered_joint_names());
  const auto min_positions = node_->declare_parameter<std::vector<double>>(
      "joint_limit_min_positions", Filled(names.size(), -2.5));
  const auto max_positions = node_->declare_parameter<std::vector<double>>(
      "joint_limit_max_positions", Filled(names.size(), 2.5));
  const auto max_velocities = node_->declare_parameter<std::vector<double>>(
      "joint_limit_max_velocities", Filled(names.size(), 1.5));

  std::unordered_map<std::string, JointLimit> configured_limits;
  const size_t count =
      std::min(names.size(),
               std::min(min_positions.size(),
                        std::min(max_positions.size(), max_velocities.size())));
  for (size_t i = 0; i < count; ++i) {
    configured_limits.emplace(
        names[i],
        JointLimit{names[i], min_positions[i], max_positions[i],
                   max_velocities[i]});
  }

  std::vector<JointLimit> limits;
  limits.reserve(joint_map_.ordered_joint_names().size());
  for (const auto& joint_name : joint_map_.ordered_joint_names()) {
    const auto it = configured_limits.find(joint_name);
    if (it != configured_limits.end()) {
      limits.push_back(it->second);
    } else {
      limits.push_back(JointLimit{joint_name, -2.5, 2.5, 1.5});
    }
  }
  return limits;
}

bool ArmSdkController::has_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_state_;
}

bool ArmSdkController::control_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_ == Mode::kActiveHold || mode_ == Mode::kActiveMoving ||
         mode_ == Mode::kReleasing;
}

std::string ArmSdkController::arm_model_name() const {
  return joint_map_.arm_model_name();
}

bool ArmSdkController::set_joint_targets(const ArmCommand& command,
                                         std::string& message) {
  if (!ValidateCommandShape(command, false, message)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_state_) {
    message = "waiting for /lowstate before accepting arm set_pose";
    return false;
  }

  QueuedTarget target;
  target.positions = current_positions_;
  target.kp = BuildGainVector(command.joint_names, command.kp, kp_);
  target.kd = BuildGainVector(command.joint_names, command.kd, kd_);

  for (size_t i = 0; i < command.joint_names.size(); ++i) {
    size_t index = 0;
    if (!FindJointIndex(command.joint_names[i], index)) {
      message = "unknown joint: " + command.joint_names[i];
      return false;
    }
    target.positions[index] = ClampToJointLimit(index, command.positions[i]);
  }

  if (mode_ == Mode::kReady || active_source_ != CommandSource::kService) {
    while (!queued_targets_.empty()) {
      queued_targets_.pop();
    }
    target_positions_ = target.positions;
    commanded_positions_ = current_positions_;
    kp_ = target.kp;
    kd_ = target.kd;
    active_source_ = CommandSource::kService;
    StartControlLocked();
  } else {
    queued_targets_.push(target);
  }

  message = "arm set_pose target accepted";
  return true;
}

bool ArmSdkController::stream_joint_targets(const ArmCommand& command,
                                            std::string& message) {
  if (!ValidateCommandShape(command, true, message)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_state_) {
    message = "waiting for /lowstate before accepting arm servo command";
    return false;
  }

  while (!queued_targets_.empty()) {
    queued_targets_.pop();
  }
  active_source_ = CommandSource::kServo;
  StartControlLocked();

  kp_ = BuildGainVector(command.joint_names, command.kp, kp_);
  kd_ = BuildGainVector(command.joint_names, command.kd, kd_);
  const auto now = node_->now();
  const auto reference = ServoReferenceLocked(now);

  for (size_t i = 0; i < command.joint_names.size(); ++i) {
    size_t index = 0;
    if (!FindJointIndex(command.joint_names[i], index)) {
      message = "unknown joint: " + command.joint_names[i];
      return false;
    }

    double target = target_positions_[index];
    if (command.is_delta) {
      const double delta =
          Clamp(command.deltas[i], -max_delta_per_command_rad_,
                max_delta_per_command_rad_);
      target += delta;
    } else {
      const double diff = command.positions[i] - reference[index];
      const double safe_diff =
          Clamp(diff, -max_abs_step_per_command_rad_,
                max_abs_step_per_command_rad_);
      target = reference[index] + safe_diff;
    }
    target_positions_[index] = ClampToJointLimit(index, target);
  }

  mode_ = HasMotionErrorLocked(1e-6) ? Mode::kActiveMoving : Mode::kActiveHold;
  message = "arm servo target updated";
  return true;
}

bool ArmSdkController::hold_current_pose(std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_state_) {
    message = "waiting for /lowstate before holding arm pose";
    return false;
  }

  ResetTargetsToMeasuredLocked();
  active_source_ = CommandSource::kServo;
  mode_ = Mode::kActiveHold;
  release_weight_ = 1.0;
  message = "holding current arm pose";
  return true;
}

bool ArmSdkController::release_control(std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_state_) {
    mode_ = Mode::kWaitLowState;
    message = "waiting for /lowstate; arm control is not active";
    return true;
  }

  if (mode_ == Mode::kReady || mode_ == Mode::kWaitLowState) {
    release_weight_ = 0.0;
    mode_ = Mode::kReady;
    active_source_ = CommandSource::kNone;
    message = "arm control already released";
    return true;
  }

  while (!queued_targets_.empty()) {
    queued_targets_.pop();
  }
  target_positions_ = commanded_positions_;
  release_start_time_ = node_->now();
  release_weight_ = 1.0;
  mode_ = Mode::kReleasing;
  active_source_ = CommandSource::kNone;
  message = "releasing arm_sdk control";
  return true;
}

bool ArmSdkController::release_control(double duration_sec,
                                       std::string& message) {
  if (duration_sec > 0.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    release_duration_sec_ = duration_sec;
  }
  return release_control(message);
}

ArmState ArmSdkController::current_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ArmState{joint_map_.ordered_joint_names(), current_positions_,
                  current_velocities_, has_state_};
}

bool ArmSdkController::ValidateCommandShape(const ArmCommand& command,
                                            bool allow_delta,
                                            std::string& message) const {
  if (command.joint_names.empty()) {
    message = "joint command is empty";
    return false;
  }
  if (command.is_delta && !allow_delta) {
    message = "delta commands are not accepted by set_pose";
    return false;
  }

  const size_t required_size =
      command.is_delta ? command.deltas.size() : command.positions.size();
  if (command.joint_names.size() != required_size) {
    message = command.is_delta ? "joint_names and deltas size mismatch"
                               : "joint_names and positions size mismatch";
    return false;
  }
  if (command.velocities.size() > command.joint_names.size() ||
      command.kp.size() > command.joint_names.size() ||
      command.kd.size() > command.joint_names.size()) {
    message = "optional arrays must not be longer than joint_names";
    return false;
  }

  std::unordered_set<std::string> seen;
  for (const auto& name : command.joint_names) {
    if (!seen.insert(name).second) {
      message = "duplicate joint in command: " + name;
      return false;
    }
    size_t index = 0;
    if (!FindJointIndex(name, index)) {
      message = "unknown joint: " + name;
      return false;
    }
  }

  message = "ok";
  return true;
}

bool ArmSdkController::FindJointIndex(const std::string& name,
                                      size_t& index) const {
  const auto& names = joint_map_.ordered_joint_names();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end()) {
    return false;
  }
  index = static_cast<size_t>(std::distance(names.begin(), it));
  return true;
}

double ArmSdkController::ClampToJointLimit(size_t index,
                                           double position) const {
  if (index >= limits_.size()) {
    return position;
  }
  return SafetyChecker::Clamp(position, limits_[index].min_position,
                              limits_[index].max_position);
}

double ArmSdkController::MaxStepForJoint(size_t index) const {
  double max_step = max_position_step_rad_;
  if (index < limits_.size() && limits_[index].max_velocity > 0.0) {
    max_step =
        std::min(max_step, limits_[index].max_velocity * control_period_sec_);
  }
  return max_step;
}

bool ArmSdkController::LowStateFreshLocked(const rclcpp::Time& now) const {
  if (!has_state_) {
    return false;
  }
  return (now - last_lowstate_time_).seconds() <= lowstate_fresh_timeout_sec_;
}

std::vector<double> ArmSdkController::ServoReferenceLocked(
    const rclcpp::Time& now) const {
  return LowStateFreshLocked(now) ? current_positions_ : commanded_positions_;
}

std::vector<double> ArmSdkController::BuildGainVector(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& sparse_values,
    const std::vector<double>& fallback) const {
  auto result = fallback;
  for (size_t i = 0; i < sparse_values.size(); ++i) {
    size_t index = 0;
    if (FindJointIndex(joint_names[i], index)) {
      result[index] = sparse_values[i];
    }
  }
  return result;
}

void ArmSdkController::StartControlLocked() {
  if (mode_ == Mode::kReady || mode_ == Mode::kWaitLowState) {
    commanded_positions_ = current_positions_;
  }
  release_weight_ = 1.0;
  mode_ = HasMotionErrorLocked(1e-6) ? Mode::kActiveMoving : Mode::kActiveHold;
}

void ArmSdkController::ActivateNextQueuedTargetLocked() {
  if (queued_targets_.empty()) {
    active_source_ = CommandSource::kNone;
    mode_ = Mode::kActiveHold;
    return;
  }

  const auto next = queued_targets_.front();
  queued_targets_.pop();
  target_positions_ = next.positions;
  kp_ = next.kp;
  kd_ = next.kd;
  active_source_ = CommandSource::kService;
  mode_ = Mode::kActiveMoving;
}

bool ArmSdkController::CurrentTargetReachedLocked() const {
  return !HasMotionErrorLocked(1e-6);
}

bool ArmSdkController::HasMotionErrorLocked(double tolerance) const {
  for (size_t i = 0; i < target_positions_.size() &&
                     i < commanded_positions_.size(); ++i) {
    if (std::abs(target_positions_[i] - commanded_positions_[i]) > tolerance) {
      return true;
    }
  }
  return false;
}

void ArmSdkController::ResetTargetsToMeasuredLocked() {
  target_positions_ = current_positions_;
  commanded_positions_ = current_positions_;
}

void ArmSdkController::OnLowState(
    const unitree_hg::msg::LowState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < joint_map_.ordered_joint_names().size(); ++i) {
    const int motor_index =
        joint_map_.to_motor_index(joint_map_.ordered_joint_names()[i]);
    current_positions_[i] = msg->motor_state[motor_index].q;
    current_velocities_[i] = msg->motor_state[motor_index].dq;
  }
  last_lowstate_time_ = node_->now();

  if (!has_state_) {
    has_state_ = true;
    ResetTargetsToMeasuredLocked();
    mode_ = Mode::kReady;
    RCLCPP_INFO(node_->get_logger(),
                "LowState received. Arm interface is ready.");
  }
}

void ArmSdkController::ControlLoop() {
  const auto period = std::chrono::duration<double>(control_period_sec_);
  auto next_tick = std::chrono::steady_clock::now() + period;
  while (rclcpp::ok() && !stop_control_thread_.load()) {
    PublishControl();
    std::this_thread::sleep_until(next_tick);
    next_tick += period;

    const auto now = std::chrono::steady_clock::now();
    if (next_tick < now - period) {
      next_tick = now + period;
    }
  }
}

void ArmSdkController::PublishControl() {
  unitree_hg::msg::LowCmd cmd;
  g1_control_msgs::msg::ArmCommandDebug debug_msg;
  bool should_publish = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = node_->now();
    if (!has_state_ || mode_ == Mode::kWaitLowState || mode_ == Mode::kReady) {
      return;
    }

    if (mode_ == Mode::kReleasing) {
      const double elapsed = (now - release_start_time_).seconds();
      if (release_duration_sec_ > 0.0) {
        release_weight_ = Clamp(1.0 - elapsed / release_duration_sec_, 0.0, 1.0);
      } else {
        release_weight_ = 0.0;
      }

      for (size_t i = 0; i < joint_map_.ordered_joint_names().size(); ++i) {
        const int motor_index =
            joint_map_.to_motor_index(joint_map_.ordered_joint_names()[i]);
        cmd.motor_cmd[motor_index].q =
            static_cast<float>(commanded_positions_[i]);
        cmd.motor_cmd[motor_index].dq = 0.0F;
        cmd.motor_cmd[motor_index].kp = static_cast<float>(kp_[i]);
        cmd.motor_cmd[motor_index].kd = static_cast<float>(kd_[i]);
        cmd.motor_cmd[motor_index].tau = 0.0F;
      }
      cmd.motor_cmd[joint_map_.not_used_joint_index()].q =
          static_cast<float>(release_weight_);
      debug_msg = BuildDebugLocked(commanded_positions_, now);
      should_publish = true;

      if (release_weight_ <= 0.0) {
        mode_ = Mode::kReady;
        active_source_ = CommandSource::kNone;
      }
    } else {
      for (size_t i = 0; i < commanded_positions_.size(); ++i) {
        const double diff = target_positions_[i] - commanded_positions_[i];
        commanded_positions_[i] +=
            Clamp(diff, -MaxStepForJoint(i), MaxStepForJoint(i));
      }

      if (active_source_ == CommandSource::kService &&
          CurrentTargetReachedLocked()) {
        ActivateNextQueuedTargetLocked();
      } else {
        mode_ = HasMotionErrorLocked(1e-6) ? Mode::kActiveMoving
                                           : Mode::kActiveHold;
      }

      for (size_t i = 0; i < joint_map_.ordered_joint_names().size(); ++i) {
        const int motor_index =
            joint_map_.to_motor_index(joint_map_.ordered_joint_names()[i]);
        cmd.motor_cmd[motor_index].q =
            static_cast<float>(commanded_positions_[i]);
        cmd.motor_cmd[motor_index].dq = 0.0F;
        cmd.motor_cmd[motor_index].kp = static_cast<float>(kp_[i]);
        cmd.motor_cmd[motor_index].kd = static_cast<float>(kd_[i]);
        cmd.motor_cmd[motor_index].tau = 0.0F;
      }
      cmd.motor_cmd[joint_map_.not_used_joint_index()].q = 1.0F;
      debug_msg = BuildDebugLocked(commanded_positions_, now);
      should_publish = true;
    }
  }

  if (should_publish) {
    if (debug_publisher_) {
      debug_publisher_->publish(debug_msg);
    }
    publisher_->publish(cmd);
  }
}

g1_control_msgs::msg::ArmCommandDebug ArmSdkController::BuildDebugLocked(
    const std::vector<double>& commanded_positions,
    const rclcpp::Time& stamp) const {
  g1_control_msgs::msg::ArmCommandDebug msg;
  msg.header.stamp = stamp;
  msg.joint_names = joint_map_.ordered_joint_names();
  msg.measured_positions = current_positions_;
  msg.target_positions = target_positions_;
  msg.commanded_positions = commanded_positions;
  msg.control_active = mode_ == Mode::kActiveHold ||
                       mode_ == Mode::kActiveMoving ||
                       mode_ == Mode::kReleasing;
  msg.trajectory_mode = active_source_ == CommandSource::kService;
  msg.release_after_motion = mode_ == Mode::kReleasing;
  return msg;
}

}  // namespace g1_control_interface
