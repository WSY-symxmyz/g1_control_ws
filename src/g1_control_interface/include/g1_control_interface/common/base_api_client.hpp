#pragma once

#include <atomic>
#include <cinttypes>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_control_interface {

// Thin request/response helper for Unitree API topics.
//
// The response subscription is created in a dedicated reentrant callback group
// so the waiting thread can coexist with the response callback when the node is
// spun by a multi-threaded executor.
class BaseApiClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

 public:
  using ResponseCallback = std::function<void(int32_t, const nlohmann::json&)>;

  BaseApiClient(rclcpp::Node* node, std::string request_topic,
                std::string response_topic)
      : node_(node),
        request_topic_(std::move(request_topic)),
        response_topic_(std::move(response_topic)),
        callback_group_(
            node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
        request_publisher_(
            node_->create_publisher<Request>(request_topic_, rclcpp::QoS(1))),
        next_identity_id_(node_->now().nanoseconds()) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    response_subscription_ = node_->create_subscription<Response>(
        response_topic_, rclcpp::QoS(1),
        [this](const std::shared_ptr<const Response> response) {
          HandleResponse(response);
        },
        options);
    timeout_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20), [this]() { ExpireRequests(); },
        callback_group_);
  }

  int64_t CallAsync(Request request, ResponseCallback callback,
                    std::chrono::milliseconds timeout) {
    const auto identity_id = next_identity_id_.fetch_add(1);
    request.header.identity.id = identity_id;

    PendingRequest pending;
    pending.api_id = request.header.identity.api_id;
    pending.deadline = std::chrono::steady_clock::now() + timeout;
    pending.callback = std::move(callback);
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_requests_.emplace(identity_id, std::move(pending));
    }

    request_publisher_->publish(request);
    return identity_id;
  }

  int32_t Call(Request request, nlohmann::json& json) {
    constexpr auto kBlockingTimeout = std::chrono::milliseconds(500);
    using Result = std::pair<int32_t, nlohmann::json>;
    auto response_promise = std::make_shared<std::promise<Result>>();
    auto response_future = response_promise->get_future();

    CallAsync(
        std::move(request),
        [response_promise](int32_t ret, const nlohmann::json& response_json) {
          response_promise->set_value({ret, response_json});
        },
        kBlockingTimeout);

    const auto status = response_future.wait_for(
        kBlockingTimeout + std::chrono::milliseconds(100));
    if (status != std::future_status::ready) {
      return -1;
    }

    auto result = response_future.get();
    json = std::move(result.second);
    return result.first;
  }

  int32_t Call(Request request) {
    nlohmann::json ignored;
    return Call(std::move(request), ignored);
  }

 private:
  struct PendingRequest {
    int64_t api_id{0};
    std::chrono::steady_clock::time_point deadline;
    ResponseCallback callback;
  };

  struct ExpiredRequest {
    int64_t identity_id{0};
    int64_t api_id{0};
    ResponseCallback callback;
  };

  void HandleResponse(const std::shared_ptr<const Response>& response) {
    ResponseCallback callback;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      const auto it = pending_requests_.find(response->header.identity.id);
      if (it == pending_requests_.end()) {
        return;
      }
      callback = std::move(it->second.callback);
      pending_requests_.erase(it);
    }

    nlohmann::json json = nlohmann::json::object();
    if (!response->data.empty()) {
      try {
        json = nlohmann::json::parse(response->data.data());
      } catch (const nlohmann::detail::exception&) {
        json = nlohmann::json::object();
      }
    }
    if (callback) {
      callback(response->header.status.code, json);
    }
  }

  void ExpireRequests() {
    std::vector<ExpiredRequest> expired_requests;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      for (auto it = pending_requests_.begin();
           it != pending_requests_.end();) {
        if (now >= it->second.deadline) {
          expired_requests.push_back(
              {it->first, it->second.api_id, std::move(it->second.callback)});
          it = pending_requests_.erase(it);
        } else {
          ++it;
        }
      }
    }

    const auto empty_json = nlohmann::json::object();
    for (auto& request : expired_requests) {
      RCLCPP_WARN(node_->get_logger(),
                  "Unitree API response timeout (api_id=%" PRId64
                  ", identity=%" PRId64 ")",
                  request.api_id, request.identity_id);
      if (request.callback) {
        request.callback(-1, empty_json);
      }
    }
  }

  rclcpp::Node* node_;
  std::string request_topic_;
  std::string response_topic_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<Request>::SharedPtr request_publisher_;
  rclcpp::Subscription<Response>::SharedPtr response_subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::atomic<int64_t> next_identity_id_;
  std::mutex pending_mutex_;
  std::unordered_map<int64_t, PendingRequest> pending_requests_;
};

}  // namespace g1_control_interface
