#include "planner_node.hpp"
#include <set>
#include <utility>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(400), std::bind(&PlannerNode::timerCallback, this));
  //string i wanna die
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/path_display", 10);
  //timer_one = this->create_wall_timer(
  //  std::chrono::milliseconds(1500), std::bind(&PlannerNode::publishPathMessage, this));
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return (std::sqrt(dx * dx + dy * dy) < 1); // Threshold for reaching the goal
}
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  for(int x = 0; x< 300; x++){
    for(int y = 0; y < 300; y++){
      if (300 * x + y < this->current_map_.data.size()) {
        this->arr[x][y] = this->current_map_.data[300 * y + x];
      } else {
        RCLCPP_ERROR(this->get_logger(), "Out of bounds access in map data at (%d, %d)", x, y);
        return;
      }
    }
  }
  RCLCPP_ERROR(this->get_logger(), "updated correctly jit");
}

void PlannerNode::publishPathMessage() {
  
  std::stringstream ss;
  for (int x = 0; x < 299; x += 5) {
    for (int y = 0; y < 299; y += 5) {
      if (arr[x][y] == 300) {
        ss << "O ";  // Path
      } 
      else if(x == this->g_x && y == this->g_y){
        ss << "G ";
      } 
      else if (this->arr[x][y] >= 80) {
        ss << "X ";  // Obstacles
      } else {
        ss << ". ";  // Free space
      }
      
    }
    ss << "\n";
  }

  auto message = std_msgs::msg::String();
  message.data = ss.str();
  RCLCPP_INFO(this->get_logger(), "PathMap:\n%s", message.data.c_str());
  string_pub_->publish(message);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  this->robot_pose_ = msg->pose.pose;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  this->goal_ = *msg;
  this->g_x = static_cast<int>(round(msg->point.x*10 + 150));
  this->g_y = static_cast<int>(round(msg->point.y*10 + 150));
  this->goal_received_ = true;
  this->state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  publishPathMessage();
  aStar();
}
/*
struct Node_astar{
  int x, y;
  float g, h;
  int parent_x, parent_y;
  Node_astar(int x, int y, float g = 0, float h = 0, int parent_x = -500, int parent_y = -500)
      : x(x), y(y), g(g), h(h), parent_x(parent_x), parent_y(parent_y){}

  // Compare based on f = g + h
  float f() const { return g + h; }
  bool operator>(const Node_astar& other) const { return f() > other.f(); }
}; */


float grid_dist(Node_astar a, Node_astar b){
  float y_dist = abs(a.y - b.y);
  float x_dist = abs(a.x - b.x);
  float diagonal_dist = 1.3 * std::min(x_dist, y_dist);
  float straight_dist = std::max(x_dist, y_dist) - std::min(x_dist, y_dist);
  return diagonal_dist + straight_dist; // this is the distance that the A* video showed
}

float grid_dist(float a_x, float a_y, float b_x, float b_y){
  float y_dist = abs(a_y - b_y);
  float x_dist = abs(a_x - b_x);
  float diagonal_dist = 1.3 * std::min(x_dist, y_dist);
  float straight_dist = std::max(x_dist, y_dist) - std::min(x_dist, y_dist);
  return diagonal_dist + straight_dist; // this is the distance that the A* video showed
}

bool same_node(Node_astar a, Node_astar b){
  return (a.x == b.x) && (a.y == b.y);
}

void PlannerNode::aStar(){
  int start_x = static_cast<int>(round(this->robot_pose_.position.x*10 + 150));
  int start_y = static_cast<int>(round(this->robot_pose_.position.y*10 + 150));
  int stop_x = static_cast<int>(round(this->goal_.point.x*10 + 150));
  int stop_y = static_cast<int>(round(this->goal_.point.y*10 + 150));
  RCLCPP_INFO(this->get_logger(), "A STAR IS STARTING");
  Node_astar start_Node = Node_astar(start_x, start_y, 0, grid_dist(start_x, start_y, stop_x, stop_y), -400, -400);
  Node_astar goal_Node = Node_astar(stop_x, stop_y);

  std::map<std::pair<int,int>, Node_astar> openNodes;
  std::map<std::pair<int, int>, Node_astar> closedNodes;

  openNodes[std::make_pair(start_x, start_y)] = start_Node;

  while(!openNodes.empty()){
    auto lowest_f = openNodes.begin();
    for (auto it = openNodes.begin(); it != openNodes.end(); ++it){
      if (it->second.f() < lowest_f->second.f()) {
        lowest_f = it;
      }
    } //find Node_astar with the lowest f value
    Node_astar current = lowest_f->second;
    openNodes.erase(lowest_f);
    closedNodes[std::make_pair(current.x, current.y)] = current;

    if(same_node(current, goal_Node)){
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = "sim_world";
      //path_msg.poses.header.frame_id = "sim_world";
      RCLCPP_INFO(this->get_logger(), "Found the node");
      RCLCPP_INFO(this->get_logger(), "Start Pos: (%d, %d), Goal: (%d, %d)", start_x, start_y, this->g_x, this->g_y);
      int i = 0;
      //int print_this[600] = {0};
      while((current.parent_x != -400 && current.parent_x != -400) || i < 600){
        current = closedNodes.at(std::make_pair(current.parent_x, current.parent_y));
        //RCLCPP_INFO(this->get_logger(), "Current node: (%d, %d), Parent node: (%d, %d)", 
        //            current.x, current.y, current.parent_x, current.parent_y);
        geometry_msgs::msg::PoseStamped pose;
    //points_set.insert(std::make_pair(current.x, current.y));
        pose.header.frame_id = "sim_world";
        pose.pose.position.x = (current.x - 150)/10;
        pose.pose.position.y = (current.y - 150)/10;
        path_msg.poses.push_back(pose);
        this->arr[current.x][current.y] = 300;
        if (++i > 600 || current.parent_x == -400) {
          //RCLCPP_WARN(this->get_logger(), "Backtracking loop limit reached. Potential issue.");
          break;
        }
      }
      publishPathMessage();
      std::reverse(path_msg.poses.begin(), path_msg.poses.end());
      path_pub_->publish(path_msg);
      return;
    }
    for(int x = -1; x <= 1; x++){
      for(int y = -1; y <= 1; y++){
        if(x == 0 && y == 0){
          continue;
        }
        int neighbour_x = current.x + x;
        int neighbour_y = current.y + y;
        if(neighbour_x < 0 || neighbour_x >= 300 || neighbour_y < 0 || neighbour_y >= 300 ){
          //this is L behaviour as it's out of the map!
          continue;
        }
        else if(this->arr[neighbour_x][neighbour_y] >= 70){ //yo too close to the wall its basically a barrier dawg?
          continue;
        }
        else if(closedNodes.count(std::make_pair(neighbour_x, neighbour_y))){
          continue; //if its already in closed its already explored so maybe thats causing an inf loop? perchance.
        }
        else{
          //sorry nested if statements 
          if(openNodes.count(std::make_pair(neighbour_x, neighbour_y))){
            Node_astar neighbour = openNodes[std::make_pair(neighbour_x, neighbour_y)];
            int extra_distance = 1;
            if(abs(x) == 1 && abs(y) == 1){
              extra_distance += 0.3;
            }
            if(neighbour.f() >= current.g + extra_distance + 4*this->arr[neighbour_x][neighbour_y] + grid_dist(neighbour, goal_Node)){
              //update the shit dawg
              neighbour.g = current.g + extra_distance + 4*this->arr[neighbour_x][neighbour_y];
              neighbour.h = grid_dist(neighbour, goal_Node);
              neighbour.parent_x = current.x;
              neighbour.parent_y = current.y;
            }
          }
          else{
            int extra_distance = 1;
            if(abs(x) == 1 && abs(y) == 1){
              extra_distance += 0.3;
            }
            Node_astar neighbour(neighbour_x, neighbour_y, current.g + extra_distance + 4*this->arr[neighbour_x][neighbour_y], grid_dist(neighbour_x, neighbour_y, goal_Node.x, goal_Node.y), current.x, current.y);
            openNodes[std::make_pair(neighbour_x, neighbour_y)] = neighbour;
          }
        }
        //logic for the neighbours? use the data from costmap
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "YOUR ALGORITHM IS BROKEN STUPID");
  publishPathMessage();
}

void PlannerNode::reconstructPath(const Node_astar &goal_node, const std::map<std::pair<int, int>, Node_astar> &closedNodes) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "sim_world";
  //points_set.clear();
  Node_astar current = goal_node;
  while (closedNodes.count(std::make_pair(current.parent_x, current.parent_y))) {
    geometry_msgs::msg::PoseStamped pose;
    //points_set.insert(std::make_pair(current.x, current.y));
    pose.pose.position.x = (current.x - 150)/10;
    pose.pose.position.y = (current.y - 150)/10;
    path_msg.poses.push_back(pose);
    //this->arr[current.x][current.y] = 900;
    current = closedNodes.at(std::make_pair(current.parent_x, current.parent_y));
  }

  std::reverse(path_msg.poses.begin(), path_msg.poses.end());
  path_pub_->publish(path_msg);
  publishPathMessage();
}

void PlannerNode::timerCallback() {
  if (this->state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      this->state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      aStar();
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
