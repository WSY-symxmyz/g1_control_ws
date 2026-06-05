#include "rclcpp/rclcpp.hpp"

#include "g1_control_interface/manager/g1_controller_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_control_interface::G1ControllerNode>();
  // Multiple threads are important here because locomotion API calls wait for
  // asynchronous responses while arm control keeps publishing in parallel.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
