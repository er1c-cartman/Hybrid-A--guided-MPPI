#ifndef SCOUT_NAVIGATION_GRIDMAP_CONVERTER_HPP
#define SCOUT_NAVIGATION_GRIDMAP_CONVERTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace scout_navigation
{

class GridMapConverter : public rclcpp::Node
{
public:
  GridMapConverter();

  bool isInitialized() const { return initialized_; }

private:
  void traversabilityMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr grid_map_msg);

  bool readParameters();

protected:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr traversability_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_pub_;
  
  grid_map::GridMap grid_map_;
  float max_value_grid_map_;
  float min_value_grid_map_;

  std::string traversability_map_topic_;
  std::string occupancy_map_topic_;
  std::string trav_layer_;

  bool initialized_;
};

} // namespace scout_navigation

#endif // SCOUT_NAVIGATION_GRIDMAP_CONVERTER_HPP

