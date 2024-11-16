#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// Initialize ROS 2 once globally to avoid multiple initializations
struct ROS2Fixture {
    ROS2Fixture() {
        if (!initialized) {
            rclcpp::init(0, nullptr);
            initialized = true;
        }
    }

    ~ROS2Fixture() {
        // Shutdown only once, during program exit
        if (initialized && rclcpp::ok()) {
            rclcpp::shutdown();
            initialized = false;
        }
    }

    static bool initialized;  // Flag to ensure initialization happens only once
};

// Define the static variable
bool ROS2Fixture::initialized = false;

TEST_CASE_METHOD(ROS2Fixture, "Talker Node Level 2 Integration Test") {
    // Create a node to use in the tests
    auto node = std::make_shared<rclcpp::Node>("talker_integration_test");

    SECTION("Test Chatter Topic Publisher") {
        // Create a subscription to listen to the chatter topic
        auto chatter_callback_called = false;
        auto subscription = node->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            [node, &chatter_callback_called](std_msgs::msg::String::UniquePtr msg) {
                RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());
                REQUIRE(msg->data.find("Hello ROS2") != std::string::npos);
                chatter_callback_called = true;
            });

        // Spin to give time to receive messages, retry multiple times to ensure success
        for (int i = 0; i < 10; ++i) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(1s);
            if (chatter_callback_called) {
                break;  // Exit early if message has been received
            }
        }

        // Verify that the subscription callback was called
        REQUIRE(chatter_callback_called);
    }

    SECTION("Test TF Broadcasting from Talker Node") {
        // Initialize the TF2 Buffer and Listener
        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Try to get the transform from world to talk
        bool transform_received = false;

        // Retry multiple times if the transform is not found immediately
        for (int i = 0; i < 5; ++i) {
            try {
                auto transform = tf_buffer.lookupTransform("world", "talk", tf2::TimePointZero, 2s);
                REQUIRE(transform.child_frame_id == "talk");
                REQUIRE(transform.header.frame_id == "world");
                transform_received = true;
                break;  // Exit loop if transform is received
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(node->get_logger(), "Attempt %d: Could not get transform: %s", i + 1, ex.what());
                std::this_thread::sleep_for(1s);
            }
        }

        // Verify that the transform was received
        REQUIRE(transform_received);
    }
}
