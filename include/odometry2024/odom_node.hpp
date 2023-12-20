#pragma once

#include <concepts>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace odometry2024::won::odom_node::impl {
    using std_msgs::msg::String;

    struct OdomNode : rclcpp::Node {
        rclcpp::Publisher<String>::SharedPtr pub;
        rclcpp::Subscription<String>::SharedPtr sub;

        OdomNode()
        : rclcpp::Node("odom_node"),
        pub{this->create_publisher<String>("greet", 10)},
        //1でトピック名、2で品質レベル、第3引数でコールバック関数
        sub{this->create_subscription<String> (
            "your_name",
            10, 
            [this](String::ConstSharedPtr msg) -> void {
                String new_msg{};
                new_msg.data = "Hello, " + msg->data + "!";
                RCLCPP_INFO_STREAM(this->get_logger(), new_msg.data);
                this->pub->publish(new_msg);
            }
        )}
        {}
    };

}

namespace odometry2024::won::odom_node {
    using impl::OdomNode;
}