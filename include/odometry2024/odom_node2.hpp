#pragma once

#include <cmath>
#include <bit>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::won::odom_node::impl
{
    using namespace std::chrono_literals;
    using std::int16_t;

    struct Xy
    {
        float x = 0;
        float y = 0;
    };

    struct OdomNode : rclcpp::Node
    {
        tf2_ros::TransformBroadcaster tf_broadcaster;
        // オドメトリの値を蓄積するデータメンバをここに追加
        float rpy{};
        Xy coordinate{};

        rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub;
        rclcpp::TimerBase::SharedPtr timer;

        OdomNode()
            : rclcpp::Node("odom_node")
            , tf_broadcaster(*this)
            , sub(this->create_subscription<can_plugins2::msg::Frame> (
                "can_rx",
                10,
                [this](const can_plugins2::msg::Frame::SharedPtr msg)
                {
                    switch (msg->id)
                    {
                    case 0x555:
                    {
                        this->coordinate = std::bit_cast<Xy>(msg->data);
                    }
                    break;

                    case 0x556:
                    {
                        this->rpy = std::bit_cast<float>(msg->data);
                    }
                    break;
                    
                    default:;
                    }
                }
            ))
            , timer(this->create_wall_timer (
                1ms,
                [this]()
                {
                    this->timer_callback();
                }
            ))
        {}

        void timer_callback()
        {
            // 蓄積したオドメトリの値をtf_broadcasterから送信
            geometry_msgs::msg::TransformStamped t{};

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";

            t.transform.translation.x = this->coordinate.x;
            t.transform.translation.y = this->coordinate.y;
            t.transform.translation.z = 0.0;

            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = std::sin(this->yaw);
            t.transform.rotation.w = std::cos(this->yaw);

            this->tf_broadcaster.sendTransform(t);
        }
    };
}

namespace odometry2024::won::odom_node
{
    using impl::OdomNode;
}