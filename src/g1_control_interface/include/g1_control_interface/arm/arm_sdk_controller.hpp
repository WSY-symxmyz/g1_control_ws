#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include "g1_control_msgs/msg/arm_command_debug.hpp"
#include "g1_control_interface/common/joint_map.hpp"
#include "g1_control_interface/common/limits.hpp"
#include "g1_control_interface/common/types.hpp"

namespace g1_control_interface {

// Joint-level upper-body position servo built on the dedicated /arm_sdk channel.
// It follows the official arm_sdk example structure: wait for /lowstate, start
// from the measured pose, publish complete absolute arm commands at a fixed
// period, and release control through the NOT_USED_JOINT weight.
class ArmSdkController {
 public:
  ArmSdkController(rclcpp::Node* node, ArmModel model, double control_period_sec);
  ~ArmSdkController();

  bool has_state() const;
  bool set_joint_targets(const ArmCommand& command, std::string& message);
  bool stream_joint_targets(const ArmCommand& command, std::string& message);
  bool hold_current_pose(std::string& message);
  bool release_control(std::string& message);
  bool release_control(double duration_sec, std::string& message);
  ArmState current_state() const;
  bool control_active() const;
  std::string arm_model_name() const;

 private:
  void LoadParameters(double control_period_sec);
  std::vector<JointLimit> BuildJointLimits() const;
  bool ValidateCommandShape(const ArmCommand& command, bool allow_delta,
                            std::string& message) const;
  bool FindJointIndex(const std::string& name, size_t& index) const;
  double ClampToJointLimit(size_t index, double position) const;
  double MaxStepForJoint(size_t index) const;
  bool LowStateFreshLocked(const rclcpp::Time& now) const;
  std::vector<double> ServoReferenceLocked(const rclcpp::Time& now) const;
  std::vector<double> BuildGainVector(
      const std::vector<std::string>& joint_names,
      const std::vector<double>& sparse_values,
      const std::vector<double>& fallback) const;
  void StartControlLocked();
  void ActivateNextQueuedTargetLocked();
  bool CurrentTargetReachedLocked() const;
  bool HasMotionErrorLocked(double tolerance) const;
  void ResetTargetsToMeasuredLocked();
  void OnLowState(const unitree_hg::msg::LowState::SharedPtr msg);
  void ControlLoop();
  void PublishControl();
  g1_control_msgs::msg::ArmCommandDebug BuildDebugLocked(
      const std::vector<double>& commanded_positions,
      const rclcpp::Time& stamp) const;

  enum class Mode {
    kWaitLowState,
    kReady,
    kActiveHold,
    kActiveMoving,
    kReleasing,
  };

  enum class CommandSource {
    kNone,
    kServo,
    kService,
  };

  struct QueuedTarget {
    std::vector<double> positions;
    std::vector<double> kp;
    std::vector<double> kd;
  };

  rclcpp::Node* node_;
  ArmModel arm_model_;
  JointMap joint_map_;
  mutable std::mutex mutex_;

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr publisher_;
  rclcpp::Publisher<g1_control_msgs::msg::ArmCommandDebug>::SharedPtr
      debug_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr subscription_;
  std::thread control_thread_;
  std::atomic_bool stop_control_thread_{false};

  std::vector<double> current_positions_;
  std::vector<double> current_velocities_;
  std::vector<double> target_positions_;
  std::vector<double> commanded_positions_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<JointLimit> limits_;
  std::queue<QueuedTarget> queued_targets_;

  bool has_state_{false};
  Mode mode_{Mode::kWaitLowState};
  CommandSource active_source_{CommandSource::kNone};
  double control_period_sec_{0.02};
  double max_delta_per_command_rad_{0.05};
  double max_abs_step_per_command_rad_{0.05};
  double max_position_step_rad_{0.01};
  double lowstate_fresh_timeout_sec_{0.10};
  double release_duration_sec_{2.0};
  double default_kp_{60.0};
  double default_kd_{1.5};
  double waist_kp_scale_{4.0};
  double waist_kd_scale_{4.0};
  double release_weight_{0.0};
  rclcpp::Time last_lowstate_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time release_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace g1_control_interface
