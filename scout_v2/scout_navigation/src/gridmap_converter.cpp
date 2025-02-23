#include "scout_navigation/gridmap_converter.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace scout_navigation
{

GridMapConverter::GridMapConverter()
    : Node("gridmap_converter"), initialized_(false)
{
  this->declare_parameter<std::string>("layer", "traversability");
  this->declare_parameter<float>("max_value_grid_map", 1.0);
  this->declare_parameter<float>("min_value_grid_map", 0.0);
  this->declare_parameter<std::string>("traversability_topic", "/traversability_map");
  this->declare_parameter<std::string>("occupancy_map_topic", "/traversability_occ_map");

  bool all_params_set = true;
  all_params_set &= this->get_parameter("layer", trav_layer_);
  all_params_set &= this->get_parameter("max_value_grid_map", max_value_grid_map_);
  all_params_set &= this->get_parameter("min_value_grid_map", min_value_grid_map_);
  all_params_set &= this->get_parameter("traversability_topic", traversability_map_topic_);
  all_params_set &= this->get_parameter("occupancy_map_topic", occupancy_map_topic_);

  if (!all_params_set) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve all parameters");
  } else {
    RCLCPP_INFO(this->get_logger(), "All parameters successfully retrieved");
  }

  RCLCPP_INFO(this->get_logger(), "Parameter 'layer': %s", trav_layer_.c_str());
  RCLCPP_INFO(this->get_logger(), "Parameter 'max_value_grid_map': %f", max_value_grid_map_);
  RCLCPP_INFO(this->get_logger(), "Parameter 'min_value_grid_map': %f", min_value_grid_map_);
  RCLCPP_INFO(this->get_logger(), "Parameter 'traversability_topic': %s", traversability_map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Parameter 'occupancy_map_topic': %s", occupancy_map_topic_.c_str());

  if (trav_layer_.empty() || traversability_map_topic_.empty() || occupancy_map_topic_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "One or more parameters are empty. Initialization failed.");
    return;
  }

  traversability_map_sub_ =
      this->create_subscription<grid_map_msgs::msg::GridMap>(
          traversability_map_topic_, 10,
          std::bind(&GridMapConverter::traversabilityMapCallback, this, std::placeholders::_1));

  occupancy_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_map_topic_, 10);

  initialized_ = true;
}

void GridMapConverter::traversabilityMapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr grid_map_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, grid_map_);

  nav_msgs::msg::OccupancyGrid occupancy_map_msg;
  try
  {
    grid_map::GridMapRosConverter::toOccupancyGrid(
        grid_map_, trav_layer_, min_value_grid_map_, max_value_grid_map_,
        occupancy_map_msg);

    int OccSize = occupancy_map_msg.info.width * occupancy_map_msg.info.height;

    for (int i = 0; i < OccSize; ++i)
    {
      if (occupancy_map_msg.data[i] != -1)
      {
        int dataTemp = occupancy_map_msg.data[i];
        occupancy_map_msg.data[i] = 100 - dataTemp;
      }
    }
  }
  catch (std::out_of_range& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Layer %s does not exist!", trav_layer_.c_str());
    return;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "Published first occupancy map");
  occupancy_map_pub_->publish(occupancy_map_msg);
}

} // namespace scout_navigation

