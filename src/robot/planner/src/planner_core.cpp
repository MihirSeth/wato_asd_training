#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger),
      path_(std::make_shared<nav_msgs::msg::Path>()),
      map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void PlannerCore::initPlanner(double smoothing_factor, int max_iterations) {
    smoothing_factor_ = smoothing_factor;
    iterations_ = max_iterations;
}

bool PlannerCore::planPath(
    double start_world_x,
    double start_world_y,
    double goal_world_x,
    double goal_world_y,
    nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    map_ = map;

    CellIndex start_idx, goal_idx;
    if (!convertToMapCoordinates(start_world_x, start_world_y, start_idx) ||
        !convertToMapCoordinates(goal_world_x, goal_world_y, goal_idx)) {
        RCLCPP_WARN(logger_, "Start or goal position is out of map bounds. Aborting.");
        return false;
    }

    RCLCPP_INFO(logger_,
                "Starting A* planning from world (%0.2f, %0.2f) [cell (%d, %d)] to goal (%0.2f, %0.2f) [cell (%d, %d)].",
                start_world_x, start_world_y, start_idx.x, start_idx.y,
                goal_world_x, goal_world_y, goal_idx.x, goal_idx.y);

    std::vector<CellIndex> cell_path;
    if (!runAStar(start_idx, goal_idx, cell_path)) {
        RCLCPP_WARN(logger_, "A* failed to compute a path.");
        return false;
    }

    convertPathToWorld(cell_path);
    return true;
}

bool PlannerCore::runAStar(
    const CellIndex& start_idx,
    const CellIndex& goal_idx,
    std::vector<CellIndex>& result_path) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_scores;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> predecessors;

    initializeAStar(start_idx, goal_idx, open_set, g_scores);

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        if (current == goal_idx) {
            tracePath(predecessors, current, result_path);
            return true;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            if (!isValidCell(neighbor)) continue;

            double new_g_score = g_scores[current] + calculateCost(current, neighbor);
            if (new_g_score < g_scores[neighbor]) {
                g_scores[neighbor] = new_g_score;
                predecessors[neighbor] = current;
                double f_score = new_g_score + euclideanHeuristic(neighbor, goal_idx);
                open_set.emplace(neighbor, f_score);
            }
        }
    }
    return false;
}

void PlannerCore::initializeAStar(
    const CellIndex& start_idx,
    const CellIndex& goal_idx,
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>& open_set,
    std::unordered_map<CellIndex, double, CellIndexHash>& g_scores) {
    g_scores[start_idx] = 0.0;
    open_set.emplace(start_idx, euclideanHeuristic(start_idx, goal_idx));
}

double PlannerCore::calculateCost(const CellIndex& from, const CellIndex& to) const {
    int cell_cost = getCellCost(to);
    double step_cost = stepDistance(from, to);
    return step_cost + static_cast<double>(cell_cost) / 25.0;
}

void PlannerCore::convertPathToWorld(const std::vector<CellIndex>& cell_path) {
    path_->poses.clear();

    for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map_->header;

        double wx, wy;
        convertToWorldCoordinates(cell, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;  // Neutral orientation
        path_->poses.push_back(pose);
    }
}

void PlannerCore::tracePath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& predecessors,
    CellIndex current,
    std::vector<CellIndex>& path_out) {
    path_out.clear();
    while (predecessors.find(current) != predecessors.end()) {
        path_out.push_back(current);
        current = predecessors.at(current);
    }
    path_out.push_back(current);
    std::reverse(path_out.begin(), path_out.end());
}

int PlannerCore::getCellCost(const CellIndex& idx) const {
    if (idx.x < 0 || idx.x >= map_->info.width || idx.y < 0 || idx.y >= map_->info.height) {
        return 127;  // Out of bounds treated as high cost
    }

    int map_index = idx.y * map_->info.width + idx.x;
    int8_t value = map_->data[map_index];
    return (value < 0) ? 100 : value;  // Treat unknown values as high cost
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& idx) const {
    return {
        {idx.x - 1, idx.y}, {idx.x + 1, idx.y},
        {idx.x, idx.y - 1}, {idx.x, idx.y + 1},
        {idx.x - 1, idx.y - 1}, {idx.x + 1, idx.y + 1},
        {idx.x - 1, idx.y + 1}, {idx.x + 1, idx.y - 1}};
}

bool PlannerCore::isValidCell(const CellIndex& idx) const {
    return idx.x >= 0 && idx.x < map_->info.width &&
           idx.y >= 0 && idx.y < map_->info.height &&
           getCellCost(idx) <= 90;
}

}  // namespace robot
