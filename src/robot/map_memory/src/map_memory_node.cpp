#include "map_memory_node.hpp"
const int GRIDSIZE = 300;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(-100.0), last_y(-100.0) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.data.resize(300*300);
  for(int i = 0; i < 300*300; i++){
    global_map_.data[i] = 0;
  } //init everything to 0. albeit poorly  
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/map_display", 50);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    //this->global_map_.header = msg->header;
    this->global_map_.info = msg->info;
    this->latest_costmap_ = *msg;
    this->costmap_updated_ = true;
}

void MapMemoryNode::publishMessage(int8_t* grid){
    /*std::stringstream ss;
    for (int y = 0; y < GRIDSIZE; y+=5) {
      for (int x = 0; x < GRIDSIZE; x+=5) {
        if(grid[300*x + y] > 50){
          ss << "X" << " ";
        }
        else{
          ss << ". ";
        }
          
      }
      ss << "\n";
    }
    auto message = std_msgs::msg::String();
    message.data = ss.str();
    message.data = "";
    RCLCPP_INFO(this->get_logger(), "MemoryMap:\n%s", message.data.c_str());
    string_pub_->publish(message); */
    return;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= this->distance_threshold) {
    this->last_x = x;
    this->last_y = y;
    this->should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap(){
  if (this->should_update_map_ && this->costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(this->global_map_);
    publishMessage(this->global_map_.data.data());
    this->should_update_map_ = false;
    this->costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap(){
  for(int x = 0; x < GRIDSIZE; x++){
    for(int y = 0; y < GRIDSIZE; y++){
      if(global_map_.data[300*x + y] == 0 && latest_costmap_.data[300*x + y] > 0){
        global_map_.data[300*x + y] = latest_costmap_.data[300*x + y];
      }
      else if(global_map_.data[300*x + y] <  latest_costmap_.data[300*x + y] && latest_costmap_.data[300*x + y] > 50){
        global_map_.data[300*x + y] = latest_costmap_.data[300*x + y];
      }
      global_map_.data[300*x + y] *= 0.92;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
