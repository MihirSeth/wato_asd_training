#include <chrono>
#include <memory>
#include <string>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 50);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidar_sub, this, std::placeholders::_1));
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage(float x) {
  auto message = std_msgs::msg::String();
  message.data = std::to_string(x);
  RCLCPP_INFO(this->get_logger(), "Coord: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  auto message = std_msgs::msg::String();
  for(auto i: scan->ranges){
   publishMessage(i); 
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
