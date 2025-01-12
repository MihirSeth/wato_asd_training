#ifndef MAP_MEMORY_CORE_HPP
#define MAP_MEMORY_CORE_HPP

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robot {

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void integrateCostmap(
        const nav_msgs::msg::OccupancyGrid &latest_costmap,
        nav_msgs::msg::OccupancyGrid &global_map);

private:
    rclcpp::Logger logger_;
};

} // namespace robot

#endif // MAP_MEMORY_CORE_HPP
