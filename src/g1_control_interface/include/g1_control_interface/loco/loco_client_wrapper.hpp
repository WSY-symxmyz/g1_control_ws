#pragma once

#include <chrono>
#include <cstdint>
#include <functional>

#include "g1_control_interface/common/base_api_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_control_interface {

// Minimal wrapper around the existing Unitree locomotion request/response API.
class LocoClientWrapper {
 public:
  using StatusCallback = std::function<void(int32_t)>;
  using FsmIdCallback = std::function<void(int32_t, int)>;

  explicit LocoClientWrapper(rclcpp::Node* node);

  int32_t get_fsm_id(int& fsm_id);
  int32_t get_fsm_mode(int& fsm_mode);
  int32_t set_fsm_id(int fsm_id);
  int32_t set_velocity(double vx, double vy, double omega, double duration_sec);
  int32_t stop_move();
  void get_fsm_id_async(FsmIdCallback callback,
                        std::chrono::milliseconds timeout);
  void set_fsm_id_async(int fsm_id, StatusCallback callback,
                        std::chrono::milliseconds timeout);
  void set_velocity_async(double vx, double vy, double omega,
                          double duration_sec, StatusCallback callback,
                          std::chrono::milliseconds timeout);

 private:
  rclcpp::Node* node_;
  BaseApiClient base_client_;
};

}  // namespace g1_control_interface
