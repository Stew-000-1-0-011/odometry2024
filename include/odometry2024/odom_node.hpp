#pragma once

#include <cmath>
#include <bit>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::won::odom_node::impl
{
    using namespace std::chrono_literals;
    using std::int16_t;

    struct Rpy
    {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
    };

    struct Xyz
    {
        double x = 0;
        double y = 0;
        double z = 0;
    };

    struct Quaternion
    {
        double x = 0;
        double y = 0;
        double z = 0;
        double w = 1;
    };

    struct OdomRawData
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
        int16_t encoder_count = 0;
    };

    struct LatestData
    {
        Xyz gyro{};
        Xyz acc{};
        int16_t encoder_x{};
        int16_t encoder_y{};
    };

    struct OdomNode : rclcpp::Node
    {
        static constexpr double gyro_coefficient = 2000 / double(0x8000);
        static constexpr double processing_time = 0.001;

        tf2_ros::TransformBroadcaster tf_broadcaster;
        // オドメトリの値を蓄積するデータメンバをここに追加
        Rpy rpy{};
        Xyz coordinate{};
        LatestData latest_data{};

        std::default_random_engine eng{std::random_device{}()};
        std::normal_distribution<double> dist_xy{0.0, 0.05};
        std::normal_distribution<double> dist_th{0.0, 0.02};

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
                        auto gyro_x = std::bit_cast<OdomRawData>(msg->data);
                        this->latest_data.gyro = Xyz {
                            gyro_coefficient * gyro_x.x * std::numbers::pi / 180.0
                            , gyro_coefficient * gyro_x.y * std::numbers::pi / 180.0
                            , gyro_coefficient * gyro_x.z * std::numbers::pi / 180.0
                        };
                        this->latest_data.encoder_x = gyro_x.encoder_count;
                    }
                    break;

                    case 0x556:
                    {
                        auto acc_y = std::bit_cast<OdomRawData>(msg->data);
                        this->latest_data.acc = Xyz {
                            (double)acc_y.x
                            , (double)acc_y.y
                            , (double)acc_y.z
                        };  // スケールがRPYの計算に影響しないため
                        this->latest_data.encoder_y = acc_y.encoder_count;
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

        static Rpy get_euler_angles(Xyz gyro, Xyz acc, Rpy rpy)
        {
            constexpr double k = 0.04;
            
            // 微小時間あたりのオイラー角を計算
            Rpy micro_rpy{};
            micro_rpy.roll = gyro.x * processing_time +
                             gyro.y * processing_time * (std::sin(rpy.roll) * (std::sin(rpy.pitch) / std::cos(rpy.pitch))) +
                             gyro.z * processing_time * (std::cos(rpy.roll) * (std::sin(rpy.pitch) / std::cos(rpy.pitch)));
            micro_rpy.pitch = gyro.y * processing_time * std::cos(rpy.roll) +
                              gyro.z * processing_time * -std::sin(rpy.roll);
            micro_rpy.yaw = gyro.y * processing_time * (std::sin(rpy.roll) / std::cos(rpy.pitch)) +
                            gyro.z * processing_time * (std::cos(rpy.roll) / std::cos(rpy.pitch));

            // ジャイロセンサから求めるオイラー角に微小オイラー角を足しこむ(radian)
            auto gyro_rpy = Rpy {
                rpy.roll + micro_rpy.roll,
                rpy.pitch + micro_rpy.pitch,
                rpy.yaw + micro_rpy.yaw
            };

            // 加速度センサからオイラー角を求める(radian)
            auto acc_rpy = Rpy {
                std::atan2(acc.y, acc.z),
                std::atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)),
                0.0
            };

            // 簡易相補フィルターによる加速度センサとジャイロセンサから求めたオイラー角の合成(radian)
            rpy.roll = acc_rpy.roll * k + gyro_rpy.roll * (1.0 - k);
            rpy.pitch = acc_rpy.pitch * k + gyro_rpy.pitch * (1.0 - k);
            rpy.yaw = gyro_rpy.yaw;

            return rpy;
        }

        static Xyz get_local_speed(int16_t raw_count_encoder1, int16_t raw_count_encoder2)
        {
            Xyz local_speed{};
            double circumference = 3.0;
            local_speed.x = raw_count_encoder1 / 2048 * circumference;
            local_speed.y = raw_count_encoder2 / 2048 * circumference;

            return local_speed;
        }

        static Xyz update_coordinate(Xyz coordinate, Rpy rpy, int16_t encoder_x, int16_t encoder_y)
        {
            double rotation_matrix_zy[3][3] = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}};
            double rotation_matrix_zyx[3][3] = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}};
            double rotation_matrix_z[3][3];
            double rotation_matrix_y[3][3];
            double rotation_matrix_x[3][3];

            // x,y,z軸各回転行列の計算
            rotation_matrix_z[0][0] = std::cos(rpy.yaw);
            rotation_matrix_z[0][1] = std::sin(rpy.yaw);
            rotation_matrix_z[0][2] = 0;
            rotation_matrix_z[1][0] = -std::sin(rpy.yaw);
            rotation_matrix_z[1][1] = std::cos(rpy.yaw);
            rotation_matrix_z[1][2] = 0;
            rotation_matrix_z[2][0] = 0;
            rotation_matrix_z[2][1] = 0;
            rotation_matrix_z[2][2] = 1;

            rotation_matrix_y[0][0] = std::cos(rpy.pitch);
            rotation_matrix_y[0][1] = 0;
            rotation_matrix_y[0][2] = -std::sin(rpy.pitch);
            rotation_matrix_y[1][0] = 0;
            rotation_matrix_y[1][1] = 1;
            rotation_matrix_y[1][2] = 0;
            rotation_matrix_y[2][0] = std::sin(rpy.pitch);
            rotation_matrix_y[2][1] = 0;
            rotation_matrix_y[2][2] = std::cos(rpy.pitch);

            rotation_matrix_x[0][0] = 1;
            rotation_matrix_x[0][1] = 0;
            rotation_matrix_x[0][2] = 0;
            rotation_matrix_x[1][0] = 0;
            rotation_matrix_x[1][1] = std::cos(rpy.roll);
            rotation_matrix_x[1][2] = std::sin(rpy.roll);
            rotation_matrix_x[2][0] = 0;
            rotation_matrix_x[2][1] = -std::sin(rpy.roll);
            rotation_matrix_x[2][2] = std::cos(rpy.roll);
            // RzRyをかける
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        rotation_matrix_zy[i][j] += rotation_matrix_z[i][k] * rotation_matrix_y[k][j];
                    }
                }
            }
            // Rxをかける
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        rotation_matrix_zyx[i][j] += rotation_matrix_zy[i][k] * rotation_matrix_x[k][j];
                    }
                }
            }

            // 機体座標系での速度を求める
            Xyz local_speed = get_local_speed(encoder_x, encoder_y);

            // ワールド座標系での機体速度を求める
            auto world_speed = Xyz {
                rotation_matrix_zyx[0][0] * local_speed.x + rotation_matrix_zyx[0][1] * local_speed.y,
                rotation_matrix_zyx[1][0] * local_speed.x + rotation_matrix_zyx[1][1] * local_speed.y
            };

            // ワールド座標系での機体座標に微小時間での移動量を足しこむ
            coordinate.x += world_speed.x * processing_time;
            coordinate.y += world_speed.y * processing_time;

            return coordinate;
        }

        static Quaternion convert_euler_to_quaternion(Rpy rpy)
        {
            Quaternion quaternion{};
            quaternion.x = std::sin(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::cos(rpy.yaw / 2) - std::cos(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::sin(rpy.yaw / 2);
            quaternion.y = std::sin(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::sin(rpy.yaw / 2) + std::cos(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::cos(rpy.yaw / 2);
            quaternion.z = -std::sin(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::cos(rpy.yaw / 2) + std::cos(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::sin(rpy.yaw / 2);
            quaternion.w = std::sin(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::sin(rpy.yaw / 2) + std::cos(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::cos(rpy.yaw / 2);

            return quaternion;
        }

        void timer_callback()
        {
            // 値の計算
            const auto rpy = get_euler_angles(this->latest_data.gyro, this->latest_data.acc, this->rpy);
            const auto coordinate = update_coordinate(this->coordinate, rpy, this->latest_data.encoder_x, this->latest_data.encoder_y);

            // グローバル座標・姿勢の更新
            this->rpy = rpy;
            this->coordinate = coordinate;

            // 蓄積したオドメトリの値をtf_broadcasterから送信
            geometry_msgs::msg::TransformStamped t{};

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";

            t.transform.translation.x = coordinate.x + dist_xy(eng);
            t.transform.translation.y = coordinate.y + dist_xy(eng);
            t.transform.translation.z = 0;

            const auto quaternion = convert_euler_to_quaternion (
                Rpy {
                    0
                    , 0
                    , rpy.yaw + dist_th(eng)
                }
            );
            t.transform.rotation.x = quaternion.x;
            t.transform.rotation.y = quaternion.y;
            t.transform.rotation.z = quaternion.z;
            t.transform.rotation.w = quaternion.w;

            this->tf_broadcaster.sendTransform(t);
        }
    };
}

namespace odometry2024::won::odom_node
{
    using impl::OdomNode;
}