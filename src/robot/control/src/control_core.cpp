#include "control_core.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger &logger) : logger_(logger), lookahead_distance_(1.0), linear_speed_(0.5) {}

void ControlCore::updatePath(const nav_msgs::msg::Path::SharedPtr path) {
    current_path_ = path;
    RCLCPP_INFO(logger_, "Path updated with %lu points.", path->poses.size());
}

void ControlCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
    robot_odom_ = odom;
}

void ControlCore::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        RCLCPP_WARN(logger_, "Missing path or odometry data!");
        return;
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_INFO(logger_, "No valid lookahead point found. Stopping robot.");
        stopRobot();
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

void ControlCore::setPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) {
    cmd_vel_pub_ = pub;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
    for (const auto &pose : current_path_->poses) {
        double distance = computeDistance(robot_odom_->pose.pose.position, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;
    double target_yaw = std::atan2(target.pose.position.y - robot_odom_->pose.pose.position.y,
                                   target.pose.position.x - robot_odom_->pose.pose.position.x);
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double angle_diff = std::atan2(std::sin(target_yaw - robot_yaw), std::cos(target_yaw - robot_yaw));

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_diff;  // Gain factor for angular velocity
    return cmd_vel;
}

void ControlCore::stopRobot() {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

} // namespace robot
