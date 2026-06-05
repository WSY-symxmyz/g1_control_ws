#include "g1_control_interface/manager/robot_mode_manager.hpp"

namespace g1_control_interface {

RobotModeManager::RobotModeManager(ArmSdkController& arm_controller,
                                   LocoController& loco_controller)
    : arm_controller_(arm_controller), loco_controller_(loco_controller) {}

ControllerSnapshot RobotModeManager::snapshot() {
  ControllerSnapshot snapshot;
  snapshot.lowstate_online = arm_controller_.has_state();
  snapshot.arm_control_active = arm_controller_.control_active();
  snapshot.loco_control_active = loco_controller_.is_active();
  snapshot.arm_model = arm_controller_.arm_model_name();
  loco_controller_.get_cached_fsm(snapshot.fsm_id, snapshot.fsm_mode,
                                  snapshot.message);
  return snapshot;
}

bool RobotModeManager::stop_all(std::string& message) {
  std::string arm_message;
  std::string loco_message;
  const bool arm_ok = arm_controller_.release_control(arm_message);
  const bool loco_ok = loco_controller_.stop(loco_message);
  message = "arm: " + arm_message + "; loco: " + loco_message;
  return arm_ok && loco_ok;
}

}  // namespace g1_control_interface
