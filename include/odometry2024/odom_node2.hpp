#pragma once

#include <cmath>
#include <bit>
#include <random>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::won::odom_node::impl
{
    using namespace std::chrono_literals;
    using std::int16_t;

    constexpr double xy_stddev = 0.05;
    constexpr double th_stddev = 0.02;
    constexpr double xy_error_max = 0.08;
    constexpr double th_error_max = 0.03;

    struct Xy
    {
        float x = 0;
        float y = 0;
    };

    struct ClampedRandomGenerator final {
        std::default_random_engine eng;
        std::normal_distribution<double> dist;
        double abs_max;

        static auto make(const double stddev, const double abs_max) -> ClampedRandomGenerator {
            return ClampedRandomGenerator {
                std::default_random_engine {
                    std::random_device{}()
                }
                , std::normal_distribution<double>{0.0, stddev}
                , abs_max
            };
        }

        auto operator()() -> double {
            return std::clamp(this->dist(this->eng), -this->abs_max, this->abs_max);
        }
    };

    struct OdomNode : rclcpp::Node
    {
        tf2_ros::TransformBroadcaster tf_broadcaster;
        // オドメトリの値を蓄積するデータメンバをここに追加
        float yaw{};
        Xy coordinate{};
        std::default_random_engine eng{std::random_device{}()};
        ClampedRandomGenerator crg_xy = ClampedRandomGenerator::make(xy_stddev, xy_error_max);
        ClampedRandomGenerator crg_th = ClampedRandomGenerator::make(th_stddev, th_error_max);

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
                        this->yaw = [data = msg->data]{
                            float dst;
                            std::memcpy(&dst, data.data(), sizeof(float));
                            return dst;
                        }();
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

            const auto stamp = this->get_clock()->now();
            const auto [x, y, th] = std::make_tuple(this->coordinate.x, this->coordinate.y, this->yaw);
            const auto [ex, ey, eth] = std::make_tuple(this->crg_xy(), this->crg_xy(), this->crg_th());

            this->tf_broadcaster.sendTransform(make_transform (
                "odom", "base_link"
                , stamp
                , x + ex
                , y + ey
                , th + eth
            ));

            this->tf_broadcaster.sendTransform(make_transform (
                "base_link", "true_base_link"
                , stamp
                , -ex
                , -ey
                , -eth
            ));
        }

        static auto make_transform(const std::string_view frame_id, const std::string_view child_frame_id, builtin_interfaces::msg::Time stamp, double x, const double y, const double th) -> geometry_msgs::msg::TransformStamped {
            geometry_msgs::msg::TransformStamped t{};
            
            t.header.stamp = stamp;
            t.header.frame_id = frame_id;
            t.child_frame_id = child_frame_id;
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = std::sin(th);
            t.transform.rotation.w = std::cos(th);

            return t;
        }
    };
}

namespace odometry2024::won::odom_node
{
    using impl::OdomNode;
}