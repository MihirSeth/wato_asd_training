#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Core logic
    std::shared_ptr<robot::ControlCore> control_core_;
};

#endif // CONTROL_NODE_HPP
