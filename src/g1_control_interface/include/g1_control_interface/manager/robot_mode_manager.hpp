#pragma once

#include <string>

#include "g1_control_interface/arm/arm_sdk_controller.hpp"
#include "g1_control_interface/common/types.hpp"
#include "g1_control_interface/loco/loco_controller.hpp"

namespace g1_control_interface {

class RobotModeManager {
 public:
  RobotModeManager(ArmSdkController& arm_controller,
                   LocoController& loco_controller);

  ControllerSnapshot snapshot();
  bool stop_all(std::string& message);

 private:
  ArmSdkController& arm_controller_;
  LocoController& loco_controller_;
};

}  // namespace g1_control_interface
