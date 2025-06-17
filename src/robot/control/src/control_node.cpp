#include "control_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    // No path or odometry, stop the robot
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }
  // Check if goal is reached
  const auto& goal = current_path_->poses.back().pose.position;
  const auto& robot = robot_odom_->pose.pose.position;
  if (computeDistance(goal, robot) < goal_tolerance_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }
  // Find lookahead point
  auto lookahead = findLookaheadPoint();
  if (!lookahead) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }
  // Compute and publish velocity command
  auto cmd = computeVelocity(*lookahead);
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || !robot_odom_) return std::nullopt;
  const auto& robot = robot_odom_->pose.pose.position;
  for (const auto& pose_stamped : current_path_->poses) {
    if (computeDistance(robot, pose_stamped.pose.position) > lookahead_distance_) {
      return pose_stamped;
    }
  }
  // If no point is far enough, return the last point (goal)
  if (!current_path_->poses.empty()) {
    return current_path_->poses.back();
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd;
  const auto& robot_pose = robot_odom_->pose.pose;
  double robot_yaw = extractYaw(robot_pose.orientation);
  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;
  // Transform target to robot frame
  double x_r = std::cos(-robot_yaw) * dx - std::sin(-robot_yaw) * dy;
  double y_r = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;
  // Pure Pursuit curvature
  double ld = std::hypot(x_r, y_r);
  if (ld < 1e-6) return cmd;
  double curvature = 2.0 * y_r / (ld * ld);
  cmd.linear.x = linear_speed_;
  cmd.angular.z = curvature * linear_speed_;
  return cmd;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion q(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
