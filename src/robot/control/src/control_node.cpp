#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.3;  // Lookahead distance
  goal_tolerance_ = 0.3;     // Distance to consider the goal reached
  linear_speed_ = 0.45;       // Constant forward speed

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}


void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!this->current_path_ || !this->robot_odom_) {
    return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // Return nullopt if path is empty or no valid pose exists
  if (!this->current_path_ || this->current_path_->poses.empty()) {
    return std::nullopt;
  }

  auto robot_position = robot_odom_->pose.pose.position;
  double robot_x = robot_position.x;
  double robot_y = robot_position.y;
  double robot_theta = extractYaw(robot_odom_->pose.pose.orientation);

  bool forward = false;

  for (size_t i = 0; i < this->current_path_->poses.size(); ++i) {
    const auto &pose = this->current_path_->poses[i];
    double dx = pose.pose.position.x - robot_x;
    double dy = pose.pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < this->lookahead_distance_) {
      continue;
    }

    double angle_to_point = std::atan2(dy, dx);
    double angle_diff = angle_to_point - robot_theta;

    if (angle_diff > M_PI){
      angle_diff -= 2 * M_PI;
    }
    if (angle_diff < -M_PI){
      angle_diff += 2 * M_PI;
    }

    if (std::abs(angle_diff) < M_PI / 2) {
      forward = true;
      return this->current_path_->poses[i];
    }
  }

  if(!forward){
    for(size_t i = 0; i < this->current_path_->poses.size(); ++i) {
      const auto &pose = this->current_path_->poses[i];
      double dx = pose.pose.position.x - robot_x;
      double dy = pose.pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);
      if(distance < this->lookahead_distance_) {
        continue;
      }
      else{
        return this->current_path_->poses[i];
      }
    }
  }

  return std::nullopt;
}



geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;

  const auto &robot_position = robot_odom_->pose.pose.position;
  const auto &robot_orientation = robot_odom_->pose.pose.orientation;

  double target_yaw = std::atan2(target.pose.position.y - robot_position.y,
                                 target.pose.position.x - robot_position.x);

  double current_yaw = extractYaw(robot_orientation);
  double distance = computeDistance(robot_position, target.pose.position);
  // Calculate the angular error
  double steer_angle = target_yaw - current_yaw;
  
  if (steer_angle > M_PI) {
    steer_angle -= 2 * M_PI;
  } else if (steer_angle < -M_PI) {
    steer_angle += 2 * M_PI;
  }

  // Stop if close to the goal
  double distance_to_goal = computeDistance(robot_position, current_path_->poses.back().pose.position);
  if (distance_to_goal < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    return cmd_vel; 
  }

  // Set linear and angular velocities
  cmd_vel.linear.x = this->linear_speed_;
  if(abs(steer_angle) > M_PI / 2){
    cmd_vel.linear.x = 0;
  }
  cmd_vel.angular.z = steer_angle*1.4;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  // TODO: Implement distance calculation between two points
  return sqrt((a.x -b.x)*(a.x -b.x) + (a.y - b.y)*(a.y - b.y));//not gonna include z cuz im lazy
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // TODO: Implement quaternion to yaw conversion
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;

  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
