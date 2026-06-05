#include "g1_control_interface/loco/loco_controller.hpp"

#include <algorithm>
#include <utility>

namespace g1_control_interface {

LocoController::LocoController(rclcpp::Node* node)
    : node_(node), client_(node) {
  LoadParameters();
  command_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(keepalive_period_sec_),
      [this]() { OnCommandTimer(); });
}

void LocoController::LoadParameters() {
  command_timeout_sec_ =
      node_->declare_parameter<double>("loco_command_timeout_sec", 0.3);
  keepalive_period_sec_ =
      node_->declare_parameter<double>("loco_keepalive_period_sec", 0.05);
  max_vx_ = node_->declare_parameter<double>("loco_max_vx", 0.5);
  max_vy_ = node_->declare_parameter<double>("loco_max_vy", 0.3);
  max_omega_ = node_->declare_parameter<double>("loco_max_omega", 0.8);
}

bool LocoController::enable(std::string& message) {
  const auto ret = client_.set_fsm_id(500);
  active_ = ret == 0;
  message = active_ ? "locomotion enabled (fsm_id=500)"
                    : "failed to set locomotion mode";
  UpdateFsmCache(active_ ? 500 : -1, -1, message);
  return active_;
}

bool LocoController::disable(std::string& message) {
  command_pending_ = false;
  const auto ret = client_.stop_move();
  active_ = false;
  message = ret == 0 ? "locomotion stopped" : "failed to stop locomotion";
  UpdateFsmMessage(message);
  return ret == 0;
}

bool LocoController::command_velocity(const LocoCommand& command,
                                      std::string& message) {
  if (command.enable && !active_) {
    if (!enable(message)) {
      return false;
    }
  }
  latest_command_ = command;
  latest_command_.vx = std::clamp(command.vx, -max_vx_, max_vx_);
  latest_command_.vy = std::clamp(command.vy, -max_vy_, max_vy_);
  latest_command_.omega = std::clamp(command.omega, -max_omega_, max_omega_);
  last_command_time_ = node_->now();
  command_deadline_ =
      command.continuous
          ? rclcpp::Time(0, 0, RCL_ROS_TIME)
          : last_command_time_ + rclcpp::Duration::from_seconds(
                                    std::max(0.05, command.duration_sec));
  command_pending_ = true;
  message = "locomotion command buffered";
  return true;
}

bool LocoController::stop(std::string& message) {
  command_pending_ = false;
  active_ = false;
  const auto ret = client_.stop_move();
  message = ret == 0 ? "locomotion stop command sent"
                     : "failed to send locomotion stop command";
  UpdateFsmMessage(message);
  return ret == 0;
}

bool LocoController::is_active() const { return active_; }

bool LocoController::get_fsm(int& fsm_id, int& fsm_mode, std::string& message) {
  const auto ret_id = client_.get_fsm_id(fsm_id);
  const auto ret_mode = client_.get_fsm_mode(fsm_mode);
  const bool ok = (ret_id == 0 && ret_mode == 0);
  message = ok ? "fsm query ok" : "fsm query failed";
  if (ok) {
    UpdateFsmCache(fsm_id, fsm_mode, message);
  } else {
    UpdateFsmCache(-1, -1, message);
  }
  return ok;
}

void LocoController::get_cached_fsm(int& fsm_id, int& fsm_mode,
                                    std::string& message) const {
  std::lock_guard<std::mutex> lock(mutex_);
  fsm_id = cached_fsm_id_;
  fsm_mode = cached_fsm_mode_;
  message = cached_fsm_message_;
}

void LocoController::UpdateFsmCache(int fsm_id, int fsm_mode,
                                    std::string message) {
  std::lock_guard<std::mutex> lock(mutex_);
  cached_fsm_id_ = fsm_id;
  cached_fsm_mode_ = fsm_mode;
  cached_fsm_message_ = std::move(message);
}

void LocoController::UpdateFsmMessage(std::string message) {
  std::lock_guard<std::mutex> lock(mutex_);
  cached_fsm_message_ = std::move(message);
}

void LocoController::OnCommandTimer() {
  if (!active_ || !command_pending_) {
    return;
  }

  const auto now = node_->now();
  const bool stream_timed_out =
      last_command_time_.nanoseconds() > 0 &&
      (now - last_command_time_).seconds() > command_timeout_sec_;
  const bool one_shot_expired =
      command_deadline_.nanoseconds() > 0 && now > command_deadline_;

  if (stream_timed_out || one_shot_expired) {
    std::string ignored;
    stop(ignored);
    return;
  }

  // Keep the command alive with a short safety horizon. If the process dies or
  // the command stream stalls, the watchdog above will stop the robot quickly.
  (void)client_.set_velocity(latest_command_.vx, latest_command_.vy,
                             latest_command_.omega,
                             std::max(0.15, keepalive_period_sec_ * 3.0));
}

}  // namespace g1_control_interface
