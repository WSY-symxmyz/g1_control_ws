#pragma once

#include <string>
#include <vector>

#include "g1_control_interface/common/limits.hpp"

namespace g1_control_interface {

class SafetyChecker {
 public:
  static bool ValidateJointCommand(
      const std::vector<std::string>& joint_names,
      const std::vector<double>& positions,
      const std::vector<JointLimit>& limits,
      std::string& message);

  static double Clamp(double value, double min_value, double max_value);
};

}  // namespace g1_control_interface
