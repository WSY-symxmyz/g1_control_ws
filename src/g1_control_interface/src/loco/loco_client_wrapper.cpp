#include "g1_control_interface/loco/loco_client_wrapper.hpp"

#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"

namespace g1_control_interface {

namespace {

constexpr int32_t kRobotApiIdLocoGetFsmId = 7001;
constexpr int32_t kRobotApiIdLocoGetFsmMode = 7002;
constexpr int32_t kRobotApiIdLocoSetFsmId = 7101;
constexpr int32_t kRobotApiIdLocoSetVelocity = 7105;

}  // namespace

LocoClientWrapper::LocoClientWrapper(rclcpp::Node* node)
    : node_(node),
      base_client_(node_, "/api/sport/request", "/api/sport/response") {}

int32_t LocoClientWrapper::get_fsm_id(int& fsm_id) {
  unitree_api::msg::Request request;
  request.header.identity.api_id = kRobotApiIdLocoGetFsmId;
  nlohmann::json json;
  const auto ret = base_client_.Call(request, json);
  if (ret == 0 && json.contains("data")) {
    json["data"].get_to(fsm_id);
  }
  return ret;
}

int32_t LocoClientWrapper::get_fsm_mode(int& fsm_mode) {
  unitree_api::msg::Request request;
  request.header.identity.api_id = kRobotApiIdLocoGetFsmMode;
  nlohmann::json json;
  const auto ret = base_client_.Call(request, json);
  if (ret == 0 && json.contains("data")) {
    json["data"].get_to(fsm_mode);
  }
  return ret;
}

int32_t LocoClientWrapper::set_fsm_id(int fsm_id) {
  unitree_api::msg::Request request;
  request.header.identity.api_id = kRobotApiIdLocoSetFsmId;
  nlohmann::json json;
  json["data"] = fsm_id;
  request.parameter = json.dump();
  return base_client_.Call(request);
}

int32_t LocoClientWrapper::set_velocity(double vx, double vy, double omega,
                                        double duration_sec) {
  unitree_api::msg::Request request;
  request.header.identity.api_id = kRobotApiIdLocoSetVelocity;
  nlohmann::json json;
  // Match the payload format used by the existing G1 locomotion examples.
  json["velocity"] = {vx, vy, omega};
  json["duration"] = duration_sec;
  request.parameter = json.dump();
  return base_client_.Call(request);
}

int32_t LocoClientWrapper::stop_move() { return set_velocity(0.0, 0.0, 0.0, 1.0); }

}  // namespace g1_control_interface
