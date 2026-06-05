#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "g1_control_interface/common/types.hpp"

namespace g1_control_interface {

// Centralized name-to-index mapping for the arm/waist joints that this package
// exposes to higher-level applications.
class JointMap {
 public:
  explicit JointMap(ArmModel model);
  explicit JointMap(const std::vector<std::string>& ordered_joint_names);

  const std::vector<std::string>& ordered_joint_names() const;
  int to_motor_index(const std::string& joint_name) const;
  int not_used_joint_index() const;
  std::string arm_model_name() const;

 private:
  ArmModel model_;
  std::vector<std::string> ordered_joint_names_;
  std::unordered_map<std::string, int> name_to_index_;
  int not_used_joint_index_{-1};
};

}  // namespace g1_control_interface
