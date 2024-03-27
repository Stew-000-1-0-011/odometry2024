#include <memory>
#include <odometry2024/test_node.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry2024::stew::test_node::TestNode>());
    rclcpp::shutdown();
}