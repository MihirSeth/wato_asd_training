#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot {

class CostmapCore {
public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    nav_msgs::msg::OccupancyGrid generateOccupancyGrid() const;

private:
    void worldToGrid(double wx, double wy, int& x, int& y) const;
    void inflateObstacles();

    rclcpp::Logger logger_;
    std::vector<int> costmap_;
    double resolution_;
    int size_x_, size_y_;
    double origin_x_, origin_y_;
};

} // namespace robot

#endif // COSTMAP_CORE_HPP
