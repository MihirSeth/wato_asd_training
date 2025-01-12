#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    // Initialize publisher for OccupancyGrid
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    // Initialize subscriber for LaserScan
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Process LaserScan to update the costmap
    costmap_.processLaserScan(scan);

    // Publish the updated costmap
    auto costmap_msg = costmap_.generateOccupancyGrid();
    costmap_pub_->publish(costmap_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
