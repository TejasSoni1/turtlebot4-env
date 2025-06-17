#include <chrono>
#include <memory>
#include "costmap_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);
  // Publish costmap at 10 Hz
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CostmapNode::publishCostmap, this));

  // Create a subscription to /lidar topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::handleLaserScan, this, std::placeholders::_1)
  );
}

void CostmapNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg->ranges.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received LIDAR scan. First range: %f", msg->ranges[0]);
    costmap_.updateFromLaserScan(*msg);
    costmap_.inflateObstacles(0.3); // 0.3m inflation radius
  } else {
    RCLCPP_WARN(this->get_logger(), "Received LIDAR scan with no ranges.");
  }
}

void CostmapNode::publishCostmap() {
  auto msg = costmap_.getOccupancyGridMsg();
  msg.header.stamp = this->now();
  msg.header.frame_id = "sim_world"; // Ensure correct frame for visualization
  costmap_pub_->publish(msg);
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}