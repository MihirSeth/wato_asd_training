#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control_node"), control_core_(std::make_shared<robot::ControlCore>(this->get_logger())) {
    // Subscriber for the path
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Path message received.");
            control_core_->updatePath(msg);
            RCLCPP_INFO(this->get_logger(), "Path updated with %lu points.", msg->poses.size());
        });

    // Subscriber for odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Odometry message received.");
            control_core_->updateOdometry(msg);
            RCLCPP_INFO(this->get_logger(), "Odometry updated.");
        });

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_core_->setPublisher(cmd_vel_pub_);

    // Timer for control loop
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&robot::ControlCore::controlLoop, control_core_));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}