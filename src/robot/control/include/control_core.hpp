#ifndef CONTROL_CORE_HPP
#define CONTROL_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>

namespace robot {

class ControlCore {
public:
    ControlCore(const rclcpp::Logger &logger);

    void updatePath(const nav_msgs::msg::Path::SharedPtr path);
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);
    void controlLoop();
    void setPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);

private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    void stopRobot();
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    rclcpp::Logger logger_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    double lookahead_distance_;
    double linear_speed_;
};

} // namespace robot

#endif // CONTROL_CORE_HPP
