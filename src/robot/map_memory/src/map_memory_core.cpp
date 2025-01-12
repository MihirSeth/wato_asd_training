#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid &latest_costmap,
    nav_msgs::msg::OccupancyGrid &global_map) {
    RCLCPP_INFO(logger_, "Integrating latest costmap into the global map.");

    // Initialize global map if it is empty
    if (global_map.data.empty()) {
        global_map = latest_costmap;
        return;
    }

    // Transform the latest costmap to the global frame (if necessary)
    // For simplicity, we assume that the costmap is already aligned.

    // Merge costmap data into the global map
    for (size_t i = 0; i < latest_costmap.data.size(); ++i) {
        if (latest_costmap.data[i] != -1) { // Only update known cells
            global_map.data[i] = latest_costmap.data[i];
        }
    }
}

} // namespace robot
