#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <string>
#include <utility>

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
  BaseApiClient(rclcpp::Node* node, std::string request_topic,
                std::string response_topic)
      : node_(node),
        request_topic_(std::move(request_topic)),
        response_topic_(std::move(response_topic)),
        callback_group_(node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant)),
        request_publisher_(
            node_->create_publisher<Request>(request_topic_, rclcpp::QoS(1))) {}

  int32_t Call(Request request, nlohmann::json& json) {
    std::promise<const std::shared_ptr<const Response>> response_promise;
    auto response_future = response_promise.get_future();
    auto fulfilled = std::make_shared<std::atomic_bool>(false);

    request.header.identity.id =
        static_cast<int64_t>(node_->now().nanoseconds());
    const auto identity_id = request.header.identity.id;

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    auto response_subscription = node_->create_subscription<Response>(
        response_topic_, rclcpp::QoS(1),
        [fulfilled, &response_promise, identity_id](
            const std::shared_ptr<const Response> response) {
          if (response->header.identity.id == identity_id) {
            bool expected = false;
            if (fulfilled->compare_exchange_strong(expected, true)) {
              response_promise.set_value(response);
            }
          }
        },
        options);

    request_publisher_->publish(request);
    const auto status = response_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      const auto response = response_future.get();
      if (response->header.status.code != 0) {
        return response->header.status.code;
      }
      try {
        json = nlohmann::json::parse(response->data.data());
      } catch (const nlohmann::detail::exception&) {
        json = nlohmann::json::object();
      }
      (void)response_subscription;
      return 0;
    }

    (void)response_subscription;
    return -1;
  }

  int32_t Call(Request request) {
    nlohmann::json ignored;
    return Call(std::move(request), ignored);
  }

 private:
  rclcpp::Node* node_;
  std::string request_topic_;
  std::string response_topic_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<Request>::SharedPtr request_publisher_;
};

}  // namespace g1_control_interface
