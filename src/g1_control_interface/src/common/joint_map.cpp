#include "g1_control_interface/common/joint_map.hpp"

#include <stdexcept>
#include <unordered_map>

#include "g1/g1.hpp"

namespace g1_control_interface {

namespace {

std::vector<std::pair<std::string, int>> BuildArm5Pairs() {
  using Joint = G1Arm5JointIndex;
  return {
      {"left_shoulder_pitch", static_cast<int>(Joint::LEFT_SHOULDER_PITCH)},
      {"left_shoulder_roll", static_cast<int>(Joint::LEFT_SHOULDER_ROLL)},
      {"left_shoulder_yaw", static_cast<int>(Joint::LEFT_SHOULDER_YAW)},
      {"left_elbow_pitch", static_cast<int>(Joint::LEFT_ELBOW_PITCH)},
      {"left_elbow_roll", static_cast<int>(Joint::LEFT_ELBOW_ROLL)},
      {"right_shoulder_pitch", static_cast<int>(Joint::RIGHT_SHOULDER_PITCH)},
      {"right_shoulder_roll", static_cast<int>(Joint::RIGHT_SHOULDER_ROLL)},
      {"right_shoulder_yaw", static_cast<int>(Joint::RIGHT_SHOULDER_YAW)},
      {"right_elbow_pitch", static_cast<int>(Joint::RIGHT_ELBOW_PITCH)},
      {"right_elbow_roll", static_cast<int>(Joint::RIGHT_ELBOW_ROLL)},
      {"waist_yaw", static_cast<int>(Joint::WAIST_YAW)},
      {"waist_roll", static_cast<int>(Joint::WAIST_ROLL)},
      {"waist_pitch", static_cast<int>(Joint::WAIST_PITCH)},
  };
}

std::vector<std::pair<std::string, int>> BuildArm7Pairs() {
  using Joint = G1Arm7JointIndex;
  return {
      {"left_shoulder_pitch", static_cast<int>(Joint::LEFT_SHOULDER_PITCH)},
      {"left_shoulder_roll", static_cast<int>(Joint::LEFT_SHOULDER_ROLL)},
      {"left_shoulder_yaw", static_cast<int>(Joint::LEFT_SHOULDER_YAW)},
      {"left_elbow", static_cast<int>(Joint::LEFT_ELBOW)},
      {"left_wrist_roll", static_cast<int>(Joint::LEFT_WRIST_ROLL)},
      {"left_wrist_pitch", static_cast<int>(Joint::LEFT_WRIST_PITCH)},
      {"left_wrist_yaw", static_cast<int>(Joint::LEFT_WRIST_YAW)},
      {"right_shoulder_pitch", static_cast<int>(Joint::RIGHT_SHOULDER_PITCH)},
      {"right_shoulder_roll", static_cast<int>(Joint::RIGHT_SHOULDER_ROLL)},
      {"right_shoulder_yaw", static_cast<int>(Joint::RIGHT_SHOULDER_YAW)},
      {"right_elbow", static_cast<int>(Joint::RIGHT_ELBOW)},
      {"right_wrist_roll", static_cast<int>(Joint::RIGHT_WRIST_ROLL)},
      {"right_wrist_pitch", static_cast<int>(Joint::RIGHT_WRIST_PITCH)},
      {"right_wrist_yaw", static_cast<int>(Joint::RIGHT_WRIST_YAW)},
      {"waist_yaw", static_cast<int>(Joint::WAIST_YAW)},
      {"waist_roll", static_cast<int>(Joint::WAIST_ROLL)},
      {"waist_pitch", static_cast<int>(Joint::WAIST_PITCH)},
  };
}

}  // namespace

JointMap::JointMap(ArmModel model) : model_(model) {
  const auto pairs =
      model_ == ArmModel::kG1Arm5 ? BuildArm5Pairs() : BuildArm7Pairs();

  for (const auto& [name, index] : pairs) {
    ordered_joint_names_.push_back(name);
    name_to_index_.emplace(name, index);
  }

  not_used_joint_index_ =
      model_ == ArmModel::kG1Arm5
          ? static_cast<int>(G1Arm5JointIndex::NOT_USED_JOINT)
          : static_cast<int>(G1Arm7JointIndex::NOT_USED_JOINT);
}

JointMap::JointMap(const std::vector<std::string>& ordered_joint_names)
    : model_(ordered_joint_names.size() > 13 ? ArmModel::kG1Arm7
                                             : ArmModel::kG1Arm5) {
  const auto pairs =
      model_ == ArmModel::kG1Arm5 ? BuildArm5Pairs() : BuildArm7Pairs();
  std::unordered_map<std::string, int> known_pairs;
  for (const auto& [name, index] : pairs) {
    known_pairs.emplace(name, index);
  }

  for (const auto& name : ordered_joint_names) {
    const auto it = known_pairs.find(name);
    if (it == known_pairs.end()) {
      throw std::out_of_range("Unknown configured joint name: " + name);
    }
    ordered_joint_names_.push_back(name);
    name_to_index_.emplace(name, it->second);
  }

  not_used_joint_index_ =
      model_ == ArmModel::kG1Arm5
          ? static_cast<int>(G1Arm5JointIndex::NOT_USED_JOINT)
          : static_cast<int>(G1Arm7JointIndex::NOT_USED_JOINT);
}

const std::vector<std::string>& JointMap::ordered_joint_names() const {
  return ordered_joint_names_;
}

int JointMap::to_motor_index(const std::string& joint_name) const {
  const auto it = name_to_index_.find(joint_name);
  if (it == name_to_index_.end()) {
    throw std::out_of_range("Unknown joint name: " + joint_name);
  }
  return it->second;
}

int JointMap::not_used_joint_index() const { return not_used_joint_index_; }

std::string JointMap::arm_model_name() const {
  return model_ == ArmModel::kG1Arm5 ? "G1ARM5" : "G1ARM7";
}

}  // namespace g1_control_interface
