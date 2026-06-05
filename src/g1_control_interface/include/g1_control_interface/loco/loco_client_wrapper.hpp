#pragma once

#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "g1_control_interface/common/base_api_client.hpp"

namespace g1_control_interface {

// Minimal wrapper around the existing Unitree locomotion request/response API.
class LocoClientWrapper {
 public:
  explicit LocoClientWrapper(rclcpp::Node* node);

  int32_t get_fsm_id(int& fsm_id);
  int32_t get_fsm_mode(int& fsm_mode);
  int32_t set_fsm_id(int fsm_id);
  int32_t set_velocity(double vx, double vy, double omega, double duration_sec);
  int32_t stop_move();

 private:
  rclcpp::Node* node_;
  BaseApiClient base_client_;
};

}  // namespace g1_control_interface
