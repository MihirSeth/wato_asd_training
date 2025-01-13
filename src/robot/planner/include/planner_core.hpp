#ifndef PLANNER_CORE_HPP
#define PLANNER_CORE_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <unordered_map>

// Supporting Structures
struct CellIndex {
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex& idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode& a, const AStarNode& b) {
        return a.f_score > b.f_score;
    }
};

namespace robot {

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void computePath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::PointStamped& goal,
        nav_msgs::msg::Path& path);

private:
    rclcpp::Logger logger_;

    CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y) const;
    bool isValid(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    std::vector<CellIndex> getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const;
    void reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
                         const CellIndex& current, nav_msgs::msg::Path& path) const;
};

} // namespace robot

#endif // PLANNER_CORE_HPP
