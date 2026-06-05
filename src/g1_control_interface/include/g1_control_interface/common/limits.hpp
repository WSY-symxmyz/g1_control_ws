#pragma once

#include <string>

namespace g1_control_interface {

struct JointLimit {
  std::string name;
  double min_position{0.0};
  double max_position{0.0};
  double max_velocity{0.0};
};

}  // namespace g1_control_interface
