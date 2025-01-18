#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner_node"), planner_(robot::PlannerCore(this->get_logger())) {
    loadParameters();
    setupCommunication();
    planner_.initPlanner(smoothing_factor_, max_iterations_);
}

void PlannerNode::loadParameters() {
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("goal_topic", "/goal_pose");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("path_topic", "/path");
    this->declare_parameter("smoothing_factor", 0.2);
    this->declare_parameter("max_iterations", 20);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("plan_timeout", 10.0);

    map_topic_ = this->get_parameter("map_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    plan_timeout_ = this->get_parameter("plan_timeout").as_double();
}

void PlannerNode::setupCommunication() {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = msg;
    }

    if (active_goal_ && (now() - goal_start_time_).seconds() <= plan_timeout_) {
        publishPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (active_goal_) {
        RCLCPP_WARN(this->get_logger(), "A goal is already active. Ignoring new goal.");
        return;
    }

    if (!map_) {
        RCLCPP_WARN(this->get_logger(), "Map not available. Cannot set goal.");
        return;
    }

    current_goal_ = *msg;
    active_goal_ = true;
    goal_start_time_ = now();
    publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    have_odom_ = true;
}

void PlannerNode::timerCallback() {
    if (!active_goal_) return;

    double elapsed = (now() - goal_start_time_).seconds();

    if (elapsed > plan_timeout_) {
        RCLCPP_WARN(this->get_logger(), "Goal timeout. Resetting goal.");
        resetGoal();
        return;
    }

    if (isGoalReached()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached.");
        resetGoal();
        return;
    }
}

void PlannerNode::publishPath() {
    if (!have_odom_) {
        RCLCPP_WARN(this->get_logger(), "Odometry unavailable. Cannot publish path.");
        resetGoal();
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!planner_.planPath(odom_x_, odom_y_, current_goal_.point.x, current_goal_.point.y, map_)) {
        RCLCPP_ERROR(this->get_logger(), "Path planning failed.");
        resetGoal();
        return;
    }

    auto path = planner_.getPath();
    path->header.stamp = now();
    path->header.frame_id = map_->header.frame_id;
    path_pub_->publish(*path);
}

void PlannerNode::resetGoal() {
    active_goal_ = false;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = map_ ? map_->header.frame_id : "map";
    path_pub_->publish(empty_path);

    RCLCPP_INFO(this->get_logger(), "Active goal reset.");
}

bool PlannerNode::isGoalReached() const {
    double distance = std::hypot(odom_x_ - current_goal_.point.x, odom_y_ - current_goal_.point.y);
    return distance < goal_tolerance_;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
