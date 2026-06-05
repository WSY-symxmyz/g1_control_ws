#pragma once

#include <string>
#include <vector>

namespace g1_control_interface {

enum class ArmModel {
  kG1Arm5,
  kG1Arm7,
};

struct ArmCommand {
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> deltas;
  std::vector<double> velocities;
  std::vector<double> kp;
  std::vector<double> kd;
  double duration_sec{0.0};
  bool is_delta{false};
  bool hold{true};
};

struct ArmState {
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  bool has_state{false};
};

struct LocoCommand {
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
  double duration_sec{1.0};
  bool continuous{true};
  bool enable{true};
};

struct ControllerSnapshot {
  bool lowstate_online{false};
  bool arm_control_active{false};
  bool loco_control_active{false};
  int fsm_id{-1};
  int fsm_mode{-1};
  std::string arm_model;
  std::string message;
};

}  // namespace g1_control_interface
