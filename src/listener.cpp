#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class ListenerNode
 * @brief A ROS2 node that subscribes to the "chatter" topic and logs received messages.
 */
class ListenerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ListenerNode class.
   * Initializes the subscriber.
   */
  ListenerNode()
  : Node("listener") {
    // Create a subscriber to the "chatter" topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "ListenerNode has been started.");
  }

 private:
  /**
   * @brief Callback function that is called when a message is received on the "chatter" topic.
   * @param msg The message received from the topic.
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  /// < Subscriber for the "chatter" topic.
};

/**
 * @brief Main function to run the ListenerNode.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Exit status.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing ListenerNode...");
  auto node = std::make_shared<ListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
