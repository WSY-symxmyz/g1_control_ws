#include "g1_control_interface/common/safety_checker.hpp"

#include <algorithm>

namespace g1_control_interface {

bool SafetyChecker::ValidateJointCommand(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions,
    const std::vector<JointLimit>& limits,
    std::string& message) {
  // Commands are accepted as sparse joint updates, but names and target
  // positions still need to be aligned.
  if (joint_names.size() != positions.size()) {
    message = "joint_names and positions size mismatch";
    return false;
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto limit_it =
        std::find_if(limits.begin(), limits.end(), [&](const JointLimit& limit) {
          return limit.name == joint_names[i];
        });
    if (limit_it == limits.end()) {
      continue;
    }
    if (positions[i] < limit_it->min_position ||
        positions[i] > limit_it->max_position) {
      message = "joint target out of range: " + joint_names[i];
      return false;
    }
  }

  message = "ok";
  return true;
}

double SafetyChecker::Clamp(double value, double min_value, double max_value) {
  return std::clamp(value, min_value, max_value);
}

}  // namespace g1_control_interface
