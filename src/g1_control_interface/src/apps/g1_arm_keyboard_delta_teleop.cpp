#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <memory>
#include <optional>
#include <string>

#include "g1_control_msgs/msg/arm_joint_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

volatile sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

class TerminalRawMode {
 public:
  TerminalRawMode() {
    if (!isatty(STDIN_FILENO)) {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active_ = true;
    }
  }

  TerminalRawMode(const TerminalRawMode&) = delete;
  TerminalRawMode& operator=(const TerminalRawMode&) = delete;

  ~TerminalRawMode() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }
  }

  bool active() const { return active_; }

 private:
  termios original_termios_{};
  bool active_{false};
};

struct JointDelta {
  std::string joint_name;
  double delta;
};

class KeyboardDeltaTeleopNode : public rclcpp::Node {
 public:
  KeyboardDeltaTeleopNode() : Node("g1_arm_keyboard_delta_teleop") {
    topic_name_ =
        declare_parameter<std::string>("topic_name", "/g1/command/arm_joints");
    delta_step_rad_ = declare_parameter<double>("delta_step_rad", 0.01);
    kp_ = declare_parameter<double>("kp", 40.0);
    kd_ = declare_parameter<double>("kd", 1.0);
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", 100.0);

    publisher_ =
        create_publisher<g1_control_msgs::msg::ArmJointCommand>(topic_name_, 10);
  }

  double loop_rate_hz() const { return loop_rate_hz_; }

  void publish_delta(const JointDelta& command) {
    g1_control_msgs::msg::ArmJointCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = "";
    msg.joint_names = {command.joint_name};
    msg.is_delta = true;
    msg.positions = {};
    msg.deltas = {command.delta};
    msg.velocities = {};
    msg.kp = {kp_};
    msg.kd = {kd_};
    msg.duration.sec = 0;
    msg.duration.nanosec = 0;
    msg.hold = true;

    publisher_->publish(msg);
  }

  std::optional<JointDelta> command_from_key(char key) const {
    switch (key) {
      case 'w':
      case 'W':
        return JointDelta{"left_shoulder_pitch", delta_step_rad_};
      case 's':
      case 'S':
        return JointDelta{"left_shoulder_pitch", -delta_step_rad_};
      case 'a':
      case 'A':
        return JointDelta{"left_shoulder_roll", delta_step_rad_};
      case 'd':
      case 'D':
        return JointDelta{"left_shoulder_roll", -delta_step_rad_};
      default:
        return std::nullopt;
    }
  }

  std::optional<JointDelta> command_from_arrow(char arrow_code) const {
    switch (arrow_code) {
      case 'A':
        return JointDelta{"right_shoulder_pitch", delta_step_rad_};
      case 'B':
        return JointDelta{"right_shoulder_pitch", -delta_step_rad_};
      case 'D':
        return JointDelta{"right_shoulder_roll", delta_step_rad_};
      case 'C':
        return JointDelta{"right_shoulder_roll", -delta_step_rad_};
      default:
        return std::nullopt;
    }
  }

 private:
  std::string topic_name_;
  double delta_step_rad_{0.01};
  double kp_{40.0};
  double kd_{1.0};
  double loop_rate_hz_{100.0};
  rclcpp::Publisher<g1_control_msgs::msg::ArmJointCommand>::SharedPtr
      publisher_;
};

bool ReadByte(char* ch, int timeout_us) {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(STDIN_FILENO, &read_fds);

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = timeout_us;

  const int ready =
      select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
    return false;
  }

  return read(STDIN_FILENO, ch, 1) == 1;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardDeltaTeleopNode>();
  TerminalRawMode terminal;

  if (!terminal.active()) {
    RCLCPP_ERROR(node->get_logger(),
                 "stdin is not an interactive terminal; keyboard teleop cannot "
                 "read key presses.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "G1 arm keyboard delta teleop started. Keys: w/s left pitch, "
              "a/d left roll, arrows right pitch/roll, q quit.");

  rclcpp::WallRate rate(node->loop_rate_hz());
  while (rclcpp::ok() && g_stop_requested == 0) {
    char ch = 0;
    if (!ReadByte(&ch, 1000)) {
      rate.sleep();
      continue;
    }

    if (ch == 'q' || ch == 'Q') {
      break;
    }

    std::optional<JointDelta> command;
    if (ch == '\033') {
      char bracket = 0;
      char arrow_code = 0;
      if (ReadByte(&bracket, 5000) && bracket == '[' &&
          ReadByte(&arrow_code, 5000)) {
        command = node->command_from_arrow(arrow_code);
      }
    } else {
      command = node->command_from_key(ch);
    }

    if (command.has_value()) {
      node->publish_delta(command.value());
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "G1 arm keyboard delta teleop stopped.");
  rclcpp::shutdown();
  return 0;
}
