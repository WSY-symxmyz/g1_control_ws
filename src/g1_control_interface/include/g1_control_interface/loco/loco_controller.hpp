#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "g1_control_interface/common/types.hpp"
#include "g1_control_interface/loco/loco_client_wrapper.hpp"
#include "rclcpp/timer.hpp"

namespace g1_control_interface {

// Safe locomotion wrapper that keeps a short-horizon velocity command alive and
// automatically stops the robot if the command stream times out.
class LocoController {
 public:
  explicit LocoController(rclcpp::Node* node);

  bool enable(std::string& message);
  bool disable(std::string& message);
  bool command_velocity(const LocoCommand& command, std::string& message);
  bool stop(std::string& message);
  bool is_active() const;
  bool get_fsm(int& fsm_id, int& fsm_mode, std::string& message);
  void get_cached_fsm(int& fsm_id, int& fsm_mode, std::string& message) const;

 private:
  enum class FsmInitializationState {
    kUninitialized,
    kChecking,
    kSetting,
    kVerifying,
    kReady,
    kError,
  };

  void LoadParameters();
  void BeginFsmInitialization();
  void OnInitialFsmResult(int32_t ret, int fsm_id);
  void OnSetFsmResult(int32_t ret);
  void OnVerifiedFsmResult(int32_t ret, int fsm_id);
  void MarkFsmReady(std::string message);
  void MarkFsmError(std::string message);
  void OnCommandTimer();
  void OnVelocityResponse(int32_t ret);
  void SendStopAsync(const char* reason);
  std::chrono::milliseconds ApiResponseTimeout() const;
  void UpdateFsmCache(int fsm_id, int fsm_mode, std::string message);
  void UpdateFsmMessage(std::string message);

  rclcpp::Node* node_;
  LocoClientWrapper client_;
  mutable std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  LocoCommand latest_command_;
  FsmInitializationState fsm_state_{FsmInitializationState::kUninitialized};
  bool fsm_ready_{false};
  bool interface_enabled_{true};
  bool command_active_{false};
  bool command_pending_{false};
  double command_timeout_sec_{0.3};
  double keepalive_period_sec_{0.05};
  double startup_delay_sec_{0.5};
  double api_response_timeout_sec_{0.25};
  double velocity_duration_sec_{1.0};
  double max_vx_{0.5};
  double max_vy_{0.3};
  double max_omega_{0.8};
  int64_t api_failure_limit_{3};
  int64_t consecutive_api_failures_{0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time command_deadline_{0, 0, RCL_ROS_TIME};
  int cached_fsm_id_{-1};
  int cached_fsm_mode_{-1};
  std::string cached_fsm_message_{"fsm not queried"};
};

}  // namespace g1_control_interface
