#include "g1_control_interface/loco/loco_controller.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <utility>

namespace g1_control_interface {

LocoController::LocoController(rclcpp::Node* node)
    : node_(node), client_(node) {
  LoadParameters();
  startup_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(startup_delay_sec_), [this]() {
        startup_timer_->cancel();
        BeginFsmInitialization();
      });
  command_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(keepalive_period_sec_),
      [this]() { OnCommandTimer(); });
}

void LocoController::LoadParameters() {
  command_timeout_sec_ =
      node_->declare_parameter<double>("loco_command_timeout_sec", 0.3);
  keepalive_period_sec_ =
      node_->declare_parameter<double>("loco_keepalive_period_sec", 0.05);
  startup_delay_sec_ =
      node_->declare_parameter<double>("loco_fsm_startup_delay_sec", 0.5);
  api_response_timeout_sec_ =
      node_->declare_parameter<double>("loco_api_response_timeout_sec", 0.25);
  velocity_duration_sec_ =
      node_->declare_parameter<double>("loco_velocity_duration_sec", 1.0);
  max_vx_ = node_->declare_parameter<double>("loco_max_vx", 0.5);
  max_vy_ = node_->declare_parameter<double>("loco_max_vy", 0.3);
  max_omega_ = node_->declare_parameter<double>("loco_max_omega", 0.8);
  api_failure_limit_ =
      node_->declare_parameter<int64_t>("loco_api_failure_limit", 3);

  command_timeout_sec_ = std::max(0.05, command_timeout_sec_);
  keepalive_period_sec_ = std::max(0.01, keepalive_period_sec_);
  startup_delay_sec_ = std::max(0.05, startup_delay_sec_);
  api_response_timeout_sec_ = std::max(0.05, api_response_timeout_sec_);
  velocity_duration_sec_ = std::max(0.1, velocity_duration_sec_);
  api_failure_limit_ = std::max<int64_t>(1, api_failure_limit_);
}

bool LocoController::enable(std::string& message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interface_enabled_ = true;
    if (fsm_ready_) {
      message = "locomotion ready (fsm_id=500)";
      return true;
    }
  }

  BeginFsmInitialization();
  message = "locomotion FSM initialization requested";
  return true;
}

bool LocoController::disable(std::string& message) {
  const auto stopped = stop(message);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interface_enabled_ = false;
  }
  message = stopped ? "locomotion command interface disabled; FSM unchanged"
                    : "locomotion disabled but stop response timed out";
  UpdateFsmMessage(message);
  return stopped;
}

bool LocoController::command_velocity(const LocoCommand& command,
                                      std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command.enable) {
    message = "locomotion command rejected: enable is false";
    return false;
  }
  if (!interface_enabled_) {
    message = "locomotion command rejected: interface disabled";
    return false;
  }
  if (!fsm_ready_) {
    message = "locomotion command rejected: FSM 500 not ready";
    return false;
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
  command_active_ = std::abs(latest_command_.vx) > 1e-6 ||
                    std::abs(latest_command_.vy) > 1e-6 ||
                    std::abs(latest_command_.omega) > 1e-6;
  message = "locomotion command buffered";
  return true;
}

bool LocoController::stop(std::string& message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_pending_ = false;
    command_active_ = false;
  }
  const auto ret = client_.stop_move();
  message = ret == 0 ? "locomotion stop command sent"
                     : "failed to send locomotion stop command";
  UpdateFsmMessage(message);
  return ret == 0;
}

bool LocoController::is_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_active_;
}

bool LocoController::get_fsm(int& fsm_id, int& fsm_mode, std::string& message) {
  const auto ret_id = client_.get_fsm_id(fsm_id);
  const auto ret_mode = client_.get_fsm_mode(fsm_mode);
  const bool ok = (ret_id == 0 && ret_mode == 0);
  message = ok ? "fsm query ok" : "fsm query failed";
  if (ok) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fsm_ready_ = fsm_id == 500;
      fsm_state_ = fsm_ready_ ? FsmInitializationState::kReady
                              : FsmInitializationState::kError;
    }
    UpdateFsmCache(fsm_id, fsm_mode, message);
  } else {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fsm_ready_ = false;
      fsm_state_ = FsmInitializationState::kError;
    }
    UpdateFsmCache(-1, -1, message);
  }
  return ok;
}

void LocoController::BeginFsmInitialization() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fsm_state_ == FsmInitializationState::kChecking ||
        fsm_state_ == FsmInitializationState::kSetting ||
        fsm_state_ == FsmInitializationState::kVerifying) {
      return;
    }
    fsm_ready_ = false;
    command_pending_ = false;
    command_active_ = false;
    fsm_state_ = FsmInitializationState::kChecking;
    cached_fsm_message_ = "checking locomotion FSM";
  }

  RCLCPP_INFO(node_->get_logger(),
              "Checking locomotion FSM before accepting commands");
  client_.get_fsm_id_async(
      [this](int32_t ret, int fsm_id) { OnInitialFsmResult(ret, fsm_id); },
      ApiResponseTimeout());
}

void LocoController::OnInitialFsmResult(int32_t ret, int fsm_id) {
  if (ret != 0) {
    MarkFsmError("initial FSM query failed (ret=" + std::to_string(ret) + ")");
    return;
  }
  if (fsm_id == 500) {
    MarkFsmReady("locomotion ready; FSM already 500");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_state_ = FsmInitializationState::kSetting;
    cached_fsm_id_ = fsm_id;
    cached_fsm_message_ = "setting locomotion FSM to 500";
  }
  RCLCPP_WARN(node_->get_logger(),
              "Locomotion FSM is %d; requesting official Start mode (500)",
              fsm_id);
  client_.set_fsm_id_async(
      500, [this](int32_t set_ret) { OnSetFsmResult(set_ret); },
      ApiResponseTimeout());
}

void LocoController::OnSetFsmResult(int32_t ret) {
  if (ret != 0) {
    MarkFsmError("failed to set FSM 500 (ret=" + std::to_string(ret) + ")");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_state_ = FsmInitializationState::kVerifying;
    cached_fsm_message_ = "verifying locomotion FSM 500";
  }
  client_.get_fsm_id_async(
      [this](int32_t query_ret, int fsm_id) {
        OnVerifiedFsmResult(query_ret, fsm_id);
      },
      ApiResponseTimeout());
}

void LocoController::OnVerifiedFsmResult(int32_t ret, int fsm_id) {
  if (ret != 0) {
    MarkFsmError("FSM verification failed (ret=" + std::to_string(ret) + ")");
    return;
  }
  if (fsm_id != 500) {
    MarkFsmError("FSM verification returned " + std::to_string(fsm_id));
    return;
  }
  MarkFsmReady("locomotion ready; FSM set and verified as 500");
}

void LocoController::MarkFsmReady(std::string message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_ready_ = true;
    fsm_state_ = FsmInitializationState::kReady;
    consecutive_api_failures_ = 0;
    cached_fsm_id_ = 500;
    cached_fsm_mode_ = -1;
    cached_fsm_message_ = message;
  }
  RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
}

void LocoController::MarkFsmError(std::string message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_ready_ = false;
    command_pending_ = false;
    command_active_ = false;
    fsm_state_ = FsmInitializationState::kError;
    cached_fsm_id_ = -1;
    cached_fsm_mode_ = -1;
    cached_fsm_message_ = message;
  }
  RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
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
  LocoCommand command;
  bool expired = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fsm_ready_ || !interface_enabled_ || !command_pending_) {
      return;
    }

    const auto now = node_->now();
    const bool stream_timed_out =
        last_command_time_.nanoseconds() > 0 &&
        (now - last_command_time_).seconds() > command_timeout_sec_;
    const bool one_shot_expired =
        command_deadline_.nanoseconds() > 0 && now > command_deadline_;
    if (stream_timed_out || one_shot_expired) {
      command_pending_ = false;
      command_active_ = false;
      expired = true;
    } else {
      command = latest_command_;
    }
  }

  if (expired) {
    SendStopAsync("command watchdog");
    return;
  }

  client_.set_velocity_async(
      command.vx, command.vy, command.omega, velocity_duration_sec_,
      [this](int32_t ret) { OnVelocityResponse(ret); }, ApiResponseTimeout());
}

void LocoController::OnVelocityResponse(int32_t ret) {
  bool enter_error = false;
  int64_t failure_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ret == 0) {
      consecutive_api_failures_ = 0;
      return;
    }

    failure_count = ++consecutive_api_failures_;
    if (failure_count >= api_failure_limit_ && fsm_ready_) {
      fsm_ready_ = false;
      command_pending_ = false;
      command_active_ = false;
      fsm_state_ = FsmInitializationState::kError;
      cached_fsm_id_ = -1;
      cached_fsm_mode_ = -1;
      cached_fsm_message_ = "velocity API responses failed repeatedly";
      enter_error = true;
    }
  }

  RCLCPP_WARN(node_->get_logger(),
              "SetVelocity response failed (ret=%d, consecutive=%" PRId64 ")",
              ret, failure_count);
  if (enter_error) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Locomotion gate closed after repeated API failures");
    SendStopAsync("API failure gate");
  }
}

void LocoController::SendStopAsync(const char* reason) {
  client_.set_velocity_async(
      0.0, 0.0, 0.0, 1.0,
      [this, reason](int32_t ret) {
        if (ret != 0) {
          RCLCPP_WARN(node_->get_logger(),
                      "StopMove response failed after %s (ret=%d)", reason,
                      ret);
        }
      },
      ApiResponseTimeout());
}

std::chrono::milliseconds LocoController::ApiResponseTimeout() const {
  return std::chrono::milliseconds(
      static_cast<int64_t>(std::max(0.01, api_response_timeout_sec_) * 1000.0));
}

}  // namespace g1_control_interface
