// Copyright 2024 Sriramprasad Kothapalli
// Licensed under the MIT License.
// See LICENSE file in the root of this repository for details.

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;

/**
 * @class TalkerNode
 * @brief A ROS2 node that publishes messages to the "chatter" topic and provides
 * a service to change the base message string.
 */
class TalkerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the TalkerNode class.
   * Initializes the publisher, timer, and service.
   */
  TalkerNode()
  : Node("talker"), base_output_string_("Hello ROS2") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TalkerNode::publish_message, this));
    count_ = 0;

    // Create a service to change the base output string
    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "change_output_string",
        std::bind(&TalkerNode::handle_change_output_string, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "TalkerNode has been started.");
    RCLCPP_DEBUG(this->get_logger(),
    "Debugging information: Node initialized.");
    RCLCPP_WARN(this->get_logger(),
    "Warning: This is a simple warning at startup.");
  }

 private:
  /**
   * @brief Publishes a message to the "chatter" topic.
   * The message contains a base string and a count value.
   */
  void publish_message() {
    auto message = std_msgs::msg::String();
    message.data = base_output_string_ +
     ", message count: "
                   + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Publishing: '" << message.data << "'");
    publisher_->publish(message);
    // Additional log levels
    if (count_ % 5 == 0) {
      RCLCPP_WARN(this->get_logger(),
      "Count has reached a multiple of 5: %zu", count_);
    }

    if (count_ > 20) {
      RCLCPP_ERROR(this->get_logger(),
                   "Count has exceeded 20, something might be wrong!");
    }

    if (count_ > 50) {
      RCLCPP_FATAL(this->get_logger(),
      "Count has exceeded 50, shutting down node.");
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Handles requests to change the base output string.
   * @param request The request containing a boolean indicating whether
   * to change the base string.
   * @param response The response indicating success or failure of the operation.
   */
  void handle_change_output_string(
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    if (request->data) {
      base_output_string_ = "Changed ROS2 String";
      response->success = true;
      response->message = "Base output string changed successfully.";
      RCLCPP_WARN(this->get_logger(), "Base output string has been changed.");
    } else {
      base_output_string_ = "Hello ROS2";
      response->success = true;
      response->message = "Base output string reset to default.";
      RCLCPP_DEBUG(this->get_logger(),
                   "Base output string has been reset to default.");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /// < Publisher for the "chatter" topic.
  rclcpp::TimerBase::SharedPtr timer_;
  /// < Timer to periodically publish messages.
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
  /// < Service to change the base output string.
  size_t count_;
  /// < Counter for the number of messages published.
  std::string base_output_string_;
  /// < The base string used in published messages.
};

/**
 * @brief Main function to run the TalkerNode.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing TalkerNode...");
  auto node = std::make_shared<TalkerNode>();
  rclcpp::spin(node);
  RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
               "Shutting down TalkerNode due to rclcpp::shutdown()");
  rclcpp::shutdown();
  return 0;
}
