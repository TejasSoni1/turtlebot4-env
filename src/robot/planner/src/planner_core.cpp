#include "planner_core.hpp"
#include <limits>
#include <cstdlib>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

// Check if a cell is free (not an obstacle and within bounds)
bool PlannerCore::isCellFree(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const {
  if (x < 0 || y < 0 || x >= (int)map.info.width || y >= (int)map.info.height) {
    return false;
  }
  int idx = y * map.info.width + x;
  int8_t value = map.data[idx];
  
  // Be more strict: only allow truly free (0) and unknown (-1) cells
  // Reject any inflated obstacle areas (anything > 0)
  return value == -1 || value == 0;
}

// Check if a cell can be used as a start position (more permissive)
bool PlannerCore::canStartFrom(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const {
  if (x < 0 || y < 0 || x >= (int)map.info.width || y >= (int)map.info.height) {
    return false;
  }
  int idx = y * map.info.width + x;
  int8_t value = map.data[idx];
  
  // Allow starting from anywhere except solid obstacles (100)
  return value != 100;
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

// Get 8-connected neighbors (including diagonals for smoother paths)
std::vector<CellIndex> PlannerCore::getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  std::vector<CellIndex> neighbors;
  // 8-connected: cardinal + diagonal directions
  const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
  
  for (int i = 0; i < 8; ++i) {
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

// Validate that a path doesn't go through obstacles
bool PlannerCore::isPathValid(const nav_msgs::msg::OccupancyGrid& map, const std::vector<CellIndex>& path) const {
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& cell = path[i];
    // Be more lenient with the first cell (start position)
    if (i == 0) {
      if (!canStartFrom(map, cell.x, cell.y)) {
        RCLCPP_DEBUG(logger_, "Cannot start from cell (%d, %d)", cell.x, cell.y);
        return false;
      }
    } else {
      if (!isCellFree(map, cell.x, cell.y)) {
        RCLCPP_DEBUG(logger_, "Path goes through obstacle at cell (%d, %d)", cell.x, cell.y);
        return false;
      }
    }
  }
  return true;
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
  
  // Validate start and goal cells
  // For start cell, be more permissive - allow planning from inflated areas
  if (!canStartFrom(map, start_cell.x, start_cell.y)) {
    int start_idx = start_cell.y * map.info.width + start_cell.x;
    RCLCPP_ERROR(logger_, "Cannot start from cell (%d, %d) with value %d", 
                 start_cell.x, start_cell.y, 
                 (start_idx >= 0 && start_idx < (int)map.data.size()) ? map.data[start_idx] : -999);
    return path_msg;
  }
  
  // For goals in unknown areas, we'll still try to plan to them
  int start_idx = start_cell.y * map.info.width + start_cell.x;
  RCLCPP_INFO(logger_, "A* start cell: (%d, %d) (value: %d), goal cell: (%d, %d)", 
              start_cell.x, start_cell.y, map.data[start_idx], goal_cell.x, goal_cell.y);
  
  // Check distance - if goal is very far, use a more conservative approach
  double distance = heuristic(start_cell, goal_cell);
  int max_iterations = (distance > 200) ? 5000 : 10000; // Reduce iterations for far goals
  
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, double, CellIndexHash> f_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, bool, CellIndexHash> in_open_set;
  
  // Initialize with start node
  g_score[start_cell] = 0.0;
  f_score[start_cell] = heuristic(start_cell, goal_cell);
  open_set.emplace(start_cell, f_score[start_cell]);
  in_open_set[start_cell] = true;
  
  int iterations = 0;
  
  while (!open_set.empty() && iterations < max_iterations) {
    iterations++;
    
    // Early termination if we're taking too long on far goals
    if (iterations % 1000 == 0 && distance > 200) {
      RCLCPP_DEBUG(logger_, "A* iteration %d, distance to goal: %.1f", iterations, distance);
    }
    
    CellIndex current = open_set.top().index;
    open_set.pop();
    in_open_set[current] = false;
    
    if (current == goal_cell) {
      auto cell_path = reconstructPath(came_from, current);
      
      // Validate the path before returning it
      if (!isPathValid(map, cell_path)) {
        RCLCPP_WARN(logger_, "Generated path goes through obstacles, continuing search...");
        closed_set[current] = true;
        continue; // Continue searching for a better path
      }
      
      for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map.header.frame_id;
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.pose = cellToPose(map, cell);
        path_msg.poses.push_back(pose_stamped);
      }
      RCLCPP_INFO(logger_, "A* found a valid path of length %zu in %d iterations", cell_path.size(), iterations);
      return path_msg;
    }
    
    closed_set[current] = true;
    
    for (const auto& neighbor : getNeighbors(map, current)) {
      if (closed_set.count(neighbor)) continue;
      
      // Calculate movement cost (1.0 for cardinal, ~1.414 for diagonal)
      double move_cost = (abs(neighbor.x - current.x) + abs(neighbor.y - current.y) == 1) ? 1.0 : 1.414;
      double tentative_g = g_score[current] + move_cost;
      
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        f_score[neighbor] = tentative_g + heuristic(neighbor, goal_cell);
        
        if (!in_open_set.count(neighbor) || !in_open_set[neighbor]) {
          open_set.emplace(neighbor, f_score[neighbor]);
          in_open_set[neighbor] = true;
        }
      }
    }
  }
  
  if (iterations >= max_iterations) {
    RCLCPP_WARN(logger_, "A* reached maximum iterations (%d). Goal may be unreachable or too far.", max_iterations);
    
    // Fallback: Find the closest reachable point to the goal
    CellIndex best_cell = start_cell;
    double best_distance = heuristic(start_cell, goal_cell);
    
    for (const auto& pair : g_score) {
      CellIndex cell = pair.first;
      double dist_to_goal = heuristic(cell, goal_cell);
      if (dist_to_goal < best_distance) {
        best_distance = dist_to_goal;
        best_cell = cell;
      }
    }
    
    if (best_cell != start_cell) {
      RCLCPP_INFO(logger_, "Creating partial path to closest reachable point (%.1f units from goal)", best_distance);
      auto cell_path = reconstructPath(came_from, best_cell);
      for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = map.header.frame_id;
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.pose = cellToPose(map, cell);
        path_msg.poses.push_back(pose_stamped);
      }
      return path_msg;
    }
  } else {
    RCLCPP_WARN(logger_, "A* could not find a path after %d iterations!", iterations);
  }
  return path_msg;
}

} // namespace robot
