#ifndef SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP
#define SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scout_navigation
{
class OccMapToCostmapNode : public rclcpp::Node
{
public:
    OccMapToCostmapNode();

private:
    // Subscription and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_nav_pub_;  // Publisher for the inflated costmap

    // Costmaps
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_nav_;  // Costmap to which inflation will be applied

    // tf2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Callback functions
    void occMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // Inflation and publishing
    void applyStaticLayerAndInflation();  // Applies inflation to the static layer
    void publishCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, const std::string &topic_name);
};
} // namespace scout_navigation

#endif // SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP

