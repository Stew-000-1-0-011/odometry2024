#pragma once

#include <cstdint>

#include <cmath>
#include <bit>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::stew::test_node::impl {
    using i64 = std::int64_t;
    using namespace std::chrono_literals;

    struct OdomRawData
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        int16_t encoder_count = 0;
    };

    struct TestNode : rclcpp::Node
    {
        i64 x{0};
        i64 y{0};
        i64 current_x{0};
        i64 current_y{0};

        rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub;
        rclcpp::TimerBase::SharedPtr tim;

        TestNode()
        : Node("odom_test")
        , sub(this->create_subscription<can_plugins2::msg::Frame>("can_rx", 10, [this](const can_plugins2::msg::Frame::SharedPtr msg) {
            if(msg->id == 0x555) {
                this->current_x = std::bit_cast<OdomRawData>(msg->data).encoder_count;
            }
            else if(msg->id == 0x556) {
                this->current_y = std::bit_cast<OdomRawData>(msg->data).encoder_count;
            }
        }))
        , tim(this->create_wall_timer(1ms, [this]{
            this->x += this->current_x;
            this->y += this->current_y;
            RCLCPP_INFO_STREAM(this->get_logger(), this->x / 8192.0 << " " << this->y / 8912.0);
        }))
        {}
    };
}

namespace odometry2024::stew::test_node {
    using impl::TestNode;
}