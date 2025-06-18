#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <cmath>

namespace robot
{

// 2D grid index
struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return (x != other.x || y != other.y); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    // Main A* interface: returns a vector of waypoints
    nav_msgs::msg::Path planAStar(const nav_msgs::msg::OccupancyGrid& map,
                                  const geometry_msgs::msg::Pose& start,
                                  const geometry_msgs::msg::Point& goal);
  private:
    rclcpp::Logger logger_;
    // Helper methods
    bool isCellFree(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const;
    bool canStartFrom(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const;
    CellIndex poseToCell(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& pose) const;
    CellIndex pointToCell(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& pt) const;
    geometry_msgs::msg::Pose cellToPose(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;
    std::vector<CellIndex> getNeighbors(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    std::vector<CellIndex> reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current);
    // Path validation and smoothing
    bool isPathValid(const nav_msgs::msg::OccupancyGrid& map, const std::vector<CellIndex>& path) const;
};

}  

#endif
