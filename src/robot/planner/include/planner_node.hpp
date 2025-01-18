#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    // ROS2 Parameters and Topics
    void loadParameters();
    void setupCommunication();

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Planning and Utilities
    void publishPath();
    void resetGoal();
    bool isGoalReached() const;

    // Planner Core
    robot::PlannerCore planner_;

    // ROS2 Communication
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Shared Resources
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::PointStamped current_goal_;
    std::mutex map_mutex_;

    // State Tracking
    bool active_goal_{false};
    bool have_odom_{false};
    double odom_x_{0.0};
    double odom_y_{0.0};
    rclcpp::Time goal_start_time_;

    // Parameters
    std::string map_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    std::string path_topic_;
    double smoothing_factor_{0.2};
    int max_iterations_{20};
    double goal_tolerance_{0.3};
    double plan_timeout_{10.0};
};

#endif // PLANNER_NODE_HPP_
