#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger) {
    resolution_ = 0.1; // 0.1 meters per cell
    size_x_ = 100; // Example size, 10x10 meters
    size_y_ = 100;
    origin_x_ = -5.0;
    origin_y_ = -5.0;

    costmap_.resize(size_x_ * size_y_, 0); // Initialize to free space
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
    std::fill(costmap_.begin(), costmap_.end(), 0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range >= scan->range_min && range <= scan->range_max) {
            int x_grid, y_grid;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            worldToGrid(x, y, x_grid, y_grid);
            if (x_grid >= 0 && x_grid < size_x_ && y_grid >= 0 && y_grid < size_y_) {
                costmap_[y_grid * size_x_ + x_grid] = 100;
            }
        }
    }

    inflateObstacles();
}

nav_msgs::msg::OccupancyGrid CostmapCore::generateOccupancyGrid() const {
    nav_msgs::msg::OccupancyGrid grid_msg;

    grid_msg.header.stamp = rclcpp::Clock().now();
    grid_msg.header.frame_id = "map";

    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = size_x_;
    grid_msg.info.height = size_y_;
    grid_msg.info.origin.position.x = origin_x_;
    grid_msg.info.origin.position.y = origin_y_;
    grid_msg.info.origin.position.z = 0.0;

    // Convert std::vector<int> to std::vector<int8_t>
    grid_msg.data = std::vector<int8_t>(costmap_.begin(), costmap_.end());

    return grid_msg;
}

void CostmapCore::worldToGrid(double wx, double wy, int& x, int& y) const {
    x = static_cast<int>((wx - origin_x_) / resolution_);
    y = static_cast<int>((wy - origin_y_) / resolution_);
}

void CostmapCore::inflateObstacles() {
    int inflation_radius = static_cast<int>(1.0 / resolution_);
    std::vector<int> inflated_costmap = costmap_;

    for (int y = 0; y < size_y_; ++y) {
        for (int x = 0; x < size_x_; ++x) {
            if (costmap_[y * size_x_ + x] == 100) {
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= 1.0) {
                                int cost = static_cast<int>(100 * (1.0 - distance));
                                inflated_costmap[ny * size_x_ + nx] = std::max(inflated_costmap[ny * size_x_ + nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated_costmap;
}

} // namespace robot
