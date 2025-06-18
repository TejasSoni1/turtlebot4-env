#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), grid_(height_, std::vector<int8_t>(width_, -1)) {} // Initialize to -1 (unknown)

void CostmapCore::reset() {
  for (auto& row : grid_) {
    std::fill(row.begin(), row.end(), -1); // Unknown by default
  }
}

// Bresenham's line algorithm for raytracing
void raytrace(int x0, int y0, int x1, int y1, std::function<void(int,int)> set_free) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    set_free(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan) {
  reset();
  double angle = scan.angle_min;
  int robot_x = static_cast<int>((0.0 - origin_x_) / resolution_);
  int robot_y = static_cast<int>((0.0 - origin_y_) / resolution_);
  
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (std::isfinite(r) && r > scan.range_min && r < scan.range_max) {
      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      int grid_x = static_cast<int>((x - origin_x_) / resolution_);
      int grid_y = static_cast<int>((y - origin_y_) / resolution_);
      
      // Raytrace from robot to hit cell, mark as free (0)
      raytrace(robot_x, robot_y, grid_x, grid_y, [this](int x, int y) {
        this->setCell(x, y, 0);
      });
      // Mark hit cell as occupied (100) - use full occupancy for obstacles
      setCell(grid_x, grid_y, 100);
    }
    angle += scan.angle_increment;
  }
}

void CostmapCore::inflateObstacles(double inflation_radius) {
  int inflation_cells = static_cast<int>(inflation_radius / resolution_);
  std::vector<std::vector<int8_t>> inflated_grid = grid_;
  
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y][x] == 100) { // Only inflate actual obstacles
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            double dist = std::hypot(dx * resolution_, dy * resolution_);
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && dist <= inflation_radius) {
              if (inflated_grid[ny][nx] != 100) { // Don't overwrite actual obstacles
                // Use binary inflation - either obstacle (99) or free (0)
                inflated_grid[ny][nx] = 99; // High cost for inflated areas
              }
            }
          }
        }
      }
    }
  }
  grid_ = inflated_grid;
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGridMsg() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  // Flatten 2D grid to 1D
  msg.data.reserve(width_ * height_);
  for (const auto& row : grid_) {
    msg.data.insert(msg.data.end(), row.begin(), row.end());
  }
  return msg;
}

void CostmapCore::setCell(int x, int y, int8_t value) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    grid_[y][x] = value;
  }
}

int8_t CostmapCore::getCell(int x, int y) const {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    return grid_[y][x];
  }
  return 0;
}

} // namespace robot