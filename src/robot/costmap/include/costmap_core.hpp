#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <algorithm>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan);
    void inflateObstacles(double inflation_radius);
    nav_msgs::msg::OccupancyGrid getOccupancyGridMsg() const;
    void reset();

  private:
    rclcpp::Logger logger_;

    // Grid parameters
    double resolution_ = 0.1; // meters per cell
    int width_ = 400; // number of cells (40m wide)
    int height_ = 400; // number of cells (40m tall)
    double origin_x_ = -20.0; // meters
    double origin_y_ = -20.0; // meters
    std::vector<std::vector<int8_t>> grid_; // 2D Occupancy grid

    void setCell(int x, int y, int8_t value);
    int8_t getCell(int x, int y) const;
};

}  

#endif