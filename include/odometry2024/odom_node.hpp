#pragma once

#include <concepts>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::won::odom_node::impl {
    using namespace std::chrono_literals;

    struct OdomNode : rclcpp::Node {
        tf2_ros::TransformBroadcaster tf_broadcaster;
        // オドメトリの値を蓄積するデータメンバをここに追加

        rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub;
        rclcpp::TimerBase::SharedPtr timer;

        OdomNode()
            : rclcpp::Node("odom_node")
            , tf_broadcaster(*this)
            , sub(this->create_subscription<can_plugins2::msg::Frame>(
                "can_rx",
                10,
                [this](const can_plugins2::msg::Frame::SharedPtr msg) {
                    this->can_rx_callback(msg);
                }
            ))
            , timer(this->create_wall_timer(
                1ms,
                [this]() {
                    this->timer_callback();
                }
            ))
        {}

        void can_rx_callback(const can_plugins2::msg::Frame::SharedPtr msg) {
            // オドメトリの値を蓄積する処理をここに追加
        }

        void timer_callback() {
            // 蓄積したオドメトリの値をtf_broadcasterから送信し、オドメトリの値をリセット
        }
    };
}

namespace odometry2024::won::odom_node {
    using impl::OdomNode;
}