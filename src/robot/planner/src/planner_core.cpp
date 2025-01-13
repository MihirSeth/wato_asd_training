#include "planner_core.hpp"
#include <queue>
#include <cmath>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

void PlannerCore::computePath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::PointStamped& goal,
    nav_msgs::msg::Path& path) 
{
    // Convert start and goal positions to grid indices
    CellIndex start_idx = worldToGrid(map, start.position.x, start.position.y);
    CellIndex goal_idx = worldToGrid(map, goal.point.x, goal.point.y);

    // Check validity of start and goal
    if (!isValid(map, start_idx) || !isValid(map, goal_idx)) {
        RCLCPP_WARN(logger_, "Invalid start or goal position!");
        return;
    }

    // Open list (priority queue) and closed set
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    open_list.emplace(start_idx, 0.0);
    g_score[start_idx] = 0.0;

    // Perform A* search
    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        // Check if the goal is reached
        if (current.index == goal_idx) {
            reconstructPath(came_from, goal_idx, path);
            return;
        }

        // Explore neighbors
        for (const CellIndex& neighbor : getNeighbors(map, current.index)) {
            double tentative_g_score = g_score[current.index] + 1.0;

            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g_score;

                double f_score = tentative_g_score + heuristic(neighbor, goal_idx);
                open_list.emplace(neighbor, f_score);
            }
        }
    }

    RCLCPP_WARN(logger_, "Pathfinding failed: No valid path to goal!");
}

CellIndex PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y) const {
    int grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
    return CellIndex(grid_x, grid_y);
}

bool PlannerCore::isValid(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const {
    int width = map.info.width;
    int height = map.info.height;
    int index = idx.y * width + idx.x;

    return idx.x >= 0 && idx.x < width &&
           idx.y >= 0 && idx.y < height &&
           map.data[index] == 0; // 0 indicates free space
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}

std::vector<CellIndex> PlannerCore::getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& idx) const {
    std::vector<CellIndex> neighbors;
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1} // Right, Left, Up, Down
    };

    for (const auto& [dx, dy] : directions) {
        CellIndex neighbor(idx.x + dx, idx.y + dy);
        if (isValid(map, neighbor)) {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

void PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    const CellIndex& current, nav_msgs::msg::Path& path) const 
{
    CellIndex idx = current;

    while (came_from.find(idx) != came_from.end()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = idx.x;
        pose.pose.position.y = idx.y;
        pose.pose.position.z = 0.0;

        path.poses.push_back(pose);
        idx = came_from.at(idx);
    }

    // Reverse the path to get from start to goal
    std::reverse(path.poses.begin(), path.poses.end());
}

} // namespace robot
