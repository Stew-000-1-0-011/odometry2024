#pragma once

#include <concepts>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

namespace odometry2024::won::odom_node::impl {
    using namespace std::chrono_literals;

    struct Rpy{
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
  };

    struct Xyz{
    double x = 0;
    double y = 0;
    double z = 0;
  };

    struct Quaternion{
      double x = 0;
      double y = 0;
      double z = 0;
      double w = 0;
    };

    struct OdomNode : rclcpp::Node {
        tf2_ros::TransformBroadcaster tf_broadcaster;
        // オドメトリの値を蓄積するデータメンバをここに追加
        Rpy rpy;
        Rpy gyro_rpy;
        Quaternion quaternion;
        Xyz world_coordinate;

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

        Rpy get_euler_angles(Xyz gyro_xyz, Xyz acc_xyz, Rpy rpy, Rpy gyro_rpy){
            double micro_roll;
            double micro_pitch;
            double micro_yaw;
            double processing_time = 0.001;
            double k = 0.04;

            //微小時間あたりのオイラー角を計算
            micro_roll = gyro_xyz.x * processing_time +
            gyro_xyz.y * processing_time * (std::sin(gyro_rpy.roll/180.0) * (std::sin(gyro_rpy.pitch/180.0) / std::cos(gyro_rpy.pitch/180.0))) +
            gyro_xyz.z * processing_time * (std::cos(gyro_rpy.roll/180.0) * (std::sin(gyro_rpy.pitch/180.0) / std::cos(gyro_rpy.pitch/180.0)));
            micro_pitch = gyro_xyz.y * processing_time * std::cos(gyro_rpy.roll/180.0) +
            gyro_xyz.z * processing_time * -std::sin(gyro_rpy.roll/180.0);
            micro_yaw = gyro_xyz.y * processing_time * (std::sin(gyro_rpy.roll/180.0) / std::cos(gyro_rpy.pitch/180.0)) +
            gyro_xyz.z * processing_time * (std::cos(gyro_rpy.roll/180.0) / std::cos(gyro_rpy.pitch/180.0));

            //ジャイロセンサから求めるオイラー角に微小オイラー角を足しこむ(degree)
            gyro_rpy.roll += micro_roll;
            gyro_rpy.pitch += micro_pitch;
            gyro_rpy.yaw += micro_yaw;

            //加速度センサからオイラー角を求める
            Rpy acc_rpy;
            acc_rpy.roll  = std::atan2(acc_xyz.y, acc_xyz.z) * 180 / std::numbers::pi;
            acc_rpy.pitch = std::atan2(-acc_xyz.x, sqrt(acc_xyz.y * acc_xyz.y + acc_xyz.z * acc_xyz.z)) * 180 / std::numbers::pi;

            //簡易相補フィルターによる加速度センサとジャイロセンサから求めたオイラー角の合成
            rpy.roll = acc_rpy.roll * k + gyro_rpy.roll * (1-k);
            rpy.pitch = acc_rpy.pitch * k + gyro_rpy.pitch * (1-k);
            rpy.yaw = gyro_rpy.yaw;

            return rpy;
        }

        Xyz get_local_speed(int16_t raw_count_encoder1, int16_t raw_count_encoder2){
            Xyz local_speed;
            double circumference = 3.0;
            double local_alpha = 0;
            double local_beta = 0.5*std::numbers::pi;
            local_speed.x = std::cos(local_alpha + std::numbers::pi / 2) * raw_count_encoder1 / 8192 * circumference + std::cos(local_beta + std::numbers::pi / 2) * raw_count_encoder2 / 8192 * circumference;
            local_speed.y = std::sin(local_alpha + std::numbers::pi /2) * raw_count_encoder1 / 8192 * circumference + std::sin(local_beta + std::numbers::pi /2) * raw_count_encoder2 / 8192 * circumference;

            return local_speed;
        }

        Xyz get_world_coordinate(Xyz world_coordinate, Rpy rpy, int16_t raw_count_encoder1, int16_t raw_count_encoder2){
            uint8_t i;
            uint8_t j;
            uint8_t k;
            Xyz world_speed;
            Xyz local_speed = get_local_speed(raw_count_encoder1, raw_count_encoder2);
            double processing_time = 0.001;
            double rotation_matrix_zy[3][3] = {
                {0,0,0},
                {0,0,0},
                {0,0,0}
            };
            double rotation_matrix_zyx[3][3] = {
                {0,0,0},
                {0,0,0},
                {0,0,0}
            };
            double rotation_matrix_z[3][3];
            double rotation_matrix_y[3][3];
            double rotation_matrix_x[3][3];

            //x,y,z軸各回転行列の計算
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
            //RzRyをかける
            for(i=0;i<3;i++){
                for(j=0;j<3;j++){
                    for(k=0;k<3;k++){
                        rotation_matrix_zy[i][j] += rotation_matrix_z[i][k] * rotation_matrix_y[k][j];
                    }
                }
            }
            //Rxをかける
            for(i=0;i<3;i++){
                for(j=0;j<3;j++){
                    for(k=0;k<3;k++){
                        rotation_matrix_zyx[i][j] += rotation_matrix_zy[i][k] * rotation_matrix_x[k][j];
                    }
                }
            }

            //ワールド座標系での機体速度を求める
            world_speed.x += rotation_matrix_zyx[0][0] * local_speed.x;
            world_speed.x += rotation_matrix_zyx[0][1] * local_speed.y;

            world_speed.y += rotation_matrix_zyx[1][0] * local_speed.x;
            world_speed.y += rotation_matrix_zyx[1][1] * local_speed.y;

            //ワールド座標系での機体座標に微小時間での移動量を足しこむ
            world_coordinate.x += world_speed.x * processing_time;
            world_coordinate.y += world_speed.y * processing_time;

            return world_coordinate;
        }

        Quaternion convert_euler_to_quaternion(Rpy rpy){
          Quaternion quaternion;
          quaternion.x = std::sin(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::cos(rpy.yaw / 2) + std::cos(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::sin(rpy.yaw / 2);
          quaternion.y = std::cos(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::cos(rpy.yaw / 2) - std::sin(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::sin(rpy.yaw / 2);
          quaternion.z = std::cos(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::sin(rpy.yaw / 2) + std::sin(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::cos(rpy.yaw / 2);
          quaternion.w = std::cos(rpy.roll / 2) * std::cos(rpy.pitch / 2) * std::cos(rpy.yaw / 2) - std::sin(rpy.roll / 2) * std::sin(rpy.pitch / 2) * std::sin(rpy.yaw / 2);

          return quaternion;
        }

        void can_rx_callback(const can_plugins2::msg::Frame::SharedPtr msg) {
            // オドメトリの値を蓄積する処理をここに追加
            Xyz gyro_xyz;
            Xyz acc_xyz;
            int16_t raw_count_encoder1;
            int16_t raw_count_encoder2;

            //センサーが情報を取ってくる時間間隔は1ms
            if (msg->id == 0x555){//ジャイロセンサの値+エンコーダ1
                std::memcpy(&gyro_xyz.x, &msg->data[0], sizeof(int16_t));
                std::memcpy(&gyro_xyz.y, &msg->data[2], sizeof(int16_t));
                std::memcpy(&gyro_xyz.z, &msg->data[4], sizeof(int16_t));
                std::memcpy(&raw_count_encoder1, &msg->data[6], sizeof(int16_t));
            }else if (msg->id == 0x556){//加速度センサの値+エンコーダ2
                std::memcpy(&acc_xyz.x, &msg->data[0], sizeof(int16_t));
                std::memcpy(&acc_xyz.y, &msg->data[2], sizeof(int16_t));
                std::memcpy(&acc_xyz.z, &msg->data[4], sizeof(int16_t));
                std::memcpy(&raw_count_encoder2, &msg->data[6], sizeof(int16_t));
            }
            this->world_coordinate = get_world_coordinate(this->world_coordinate, this->rpy, raw_count_encoder1, raw_count_encoder2);
            this->quaternion = convert_euler_to_quaternion(get_euler_angles(gyro_xyz, acc_xyz, this->rpy, this->gyro_rpy));
            RCLCPP_INFO_STREAM(this->get_logger(),  gyro_xyz.x << gyro_xyz.y << gyro_xyz.z << raw_count_encoder1 << "\n" << acc_xyz.x << acc_xyz.x << acc_xyz.z << raw_count_encoder2 << "\n\n");
        }

        void timer_callback() {
          // 蓄積したオドメトリの値をtf_broadcasterから送信し、オドメトリの値をリセット
          geometry_msgs::msg::TransformStamped t;

          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "world";
        //   t.child_frame_id =

          t.transform.translation.x = this->world_coordinate.x;
          t.transform.translation.y = this->world_coordinate.y;
          t.transform.translation.z = this->world_coordinate.z;

          t.transform.rotation.x = this->quaternion.x;
          t.transform.rotation.y = this->quaternion.y;
          t.transform.rotation.z = this->quaternion.z;
          t.transform.rotation.w = this->quaternion.w;

          this->tf_broadcaster.sendTransform(t);
        }
    };
}
namespace odometry2024::won::odom_node {
    using impl::OdomNode;
}