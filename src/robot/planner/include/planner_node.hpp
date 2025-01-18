#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "planner_core.hpp"
#include "std_msgs/msg/string.hpp"
#include <set>
#include <utility>

struct Node_astar {
    int x, y;
    float g, h;
    int parent_x, parent_y;

    Node_astar(int x, int y, float g = 0, float h = 0, int parent_x = -500, int parent_y = -500)
        : x(x), y(y), g(g), h(h), parent_x(parent_x), parent_y(parent_y) {}
    Node_astar() : x(0), y(0), g(0), h(0), parent_x(-1), parent_y(-1) {}
    float f() const { return g + h; }
    bool operator>(const Node_astar &other) const { return f() > other.f(); }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    //rclcpp::TimerBase::SharedPtr timer_one;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    int arr[300][300] = {0};//array map i think im gonna die
    int g_x = 0;
    int g_y = 0;

    bool goal_received_ = false;
    void aStar();
    void publishPathMessage();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void reconstructPath(const Node_astar&, const std::map<std::pair<int, int>, Node_astar>&);
    void timerCallback();
    bool goalReached();
    //void planPath();
};

#endif 
