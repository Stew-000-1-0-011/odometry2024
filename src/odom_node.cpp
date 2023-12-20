#include <memory>
#include <odometry2024/odom_node.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry2024::won::odom_node::OdomNode>());
    rclcpp::shutdown();
}