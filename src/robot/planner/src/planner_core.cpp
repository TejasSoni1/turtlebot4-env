#include "planner_core.hpp"
#include <limits>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

// Check if a cell is free (not an obstacle and within bounds)
bool PlannerCore::isCellFree(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const {
  if (x < 0 || y < 0 || x >= (int)map.info.width || y >= (int)map.info.height) {
    RCLCPP_WARN(logger_, "Cell (%d, %d) is out of bounds. Map size: %dx%d", x, y, map.info.width, map.info.height);
    return false;
  }
  int idx = y * map.info.width + x;
  int8_t value = map.data[idx];
  // Allow both free (0) and unknown (-1) cells for planning
  bool is_free = value == 0 || value == -1;
  if (!is_free) {
    RCLCPP_DEBUG(logger_, "Cell (%d, %d) is occupied with value: %d", x, y, value);
  }
  return is_free;
}

// Convert a pose to a grid cell
CellIndex PlannerCore::poseToCell(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& pose) const {
  double world_x = pose.position.x;
  double world_y = pose.position.y;
  int x = static_cast<int>((world_x - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((world_y - map.info.origin.position.y) / map.info.resolution);
  RCLCPP_INFO(logger_, "Converting pose (%.2f, %.2f) to cell (%d, %d). Map origin: (%.2f, %.2f), resolution: %.2f",
              world_x, world_y, x, y, map.info.origin.position.x, map.info.origin.position.y, map.info.resolution);
  return CellIndex(x, y);
}

// Convert a point to a grid cell
CellIndex PlannerCore::pointToCell(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& pt) const {
  int x = static_cast<int>((pt.x - map.info.origin.position.x) / map.info.resolution);
  int y = static_cast<int>((pt.y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(x, y);
}

// Convert a grid cell to a pose (center of cell)
geometry_msgs::msg::Pose PlannerCore::cellToPose(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  geometry_msgs::msg::Pose pose;
  pose.position.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  pose.position.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

// Get 4-connected neighbors (up, down, left, right)
std::vector<CellIndex> PlannerCore::getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  std::vector<CellIndex> neighbors;
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  for (int i = 0; i < 4; ++i) {
    int nx = cell.x + dx[i];
    int ny = cell.y + dy[i];
    if (isCellFree(map, nx, ny)) {
      neighbors.emplace_back(nx, ny);
    }
  }
  return neighbors;
}

// Heuristic: Euclidean distance
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Reconstruct path from came_from map
std::vector<CellIndex> PlannerCore::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current) {
  std::vector<CellIndex> path;
  path.push_back(current);
  while (came_from.count(current)) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// Main A* algorithm
nav_msgs::msg::Path PlannerCore::planAStar(const nav_msgs::msg::OccupancyGrid& map,
                                           const geometry_msgs::msg::Pose& start,
                                           const geometry_msgs::msg::Point& goal) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map.header.frame_id;
  path_msg.header.stamp = rclcpp::Clock().now();

  CellIndex start_cell = poseToCell(map, start);
  CellIndex goal_cell = pointToCell(map, goal);
  RCLCPP_INFO(logger_, "A* start cell: (%d, %d), goal cell: (%d, %d)", start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  open_set.emplace(start_cell, heuristic(start_cell, goal_cell));
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, double, CellIndexHash> f_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  g_score[start_cell] = 0.0;
  f_score[start_cell] = heuristic(start_cell, goal_cell);
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set;
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    if (current == goal_cell) {
      auto cell_path = reconstructPath(came_from, current);
      for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map.header.frame_id;
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.pose = cellToPose(map, cell);
        path_msg.poses.push_back(pose_stamped);
      }
      RCLCPP_INFO(logger_, "A* found a path of length %zu", cell_path.size());
      return path_msg;
    }
    closed_set[current] = true;
    for (const auto& neighbor : getNeighbors(map, current)) {
      if (closed_set.count(neighbor)) continue;
      double tentative_g = g_score[current] + heuristic(current, neighbor);
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        f_score[neighbor] = tentative_g + heuristic(neighbor, goal_cell);
        open_set.emplace(neighbor, f_score[neighbor]);
      }
    }
  }
  RCLCPP_WARN(logger_, "A* could not find a path!");
  return path_msg;
}

} // namespace robot
