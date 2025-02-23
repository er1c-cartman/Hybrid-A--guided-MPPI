#include "scout_navigation/occmap_to_costmap.hpp"
#include <cmath>
#include <queue>
#include <nav2_costmap_2d/costmap_2d.hpp>  // Include costmap_2d

namespace scout_navigation
{

// Define costmap constants
constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char FREE_SPACE = 0;
constexpr unsigned char LETHAL_OBSTACLE = 254;
constexpr double INFLATION_RADIUS = 1.0;  // Inflation radius in meters
constexpr unsigned char INSCRIBED_OBSTACLE = 253; // Close to obstacle

OccMapToCostmapNode::OccMapToCostmapNode()
    : Node("occmap_to_costmap_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    // Declare parameters for topic names
    this->declare_parameter("map_topic", "/traversability_occ_map");

    // Create publishers for both costmaps
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
    costmap_nav_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap_nav", 10);

    // Initialize the costmaps
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>();
    costmap_nav_ = std::make_shared<nav2_costmap_2d::Costmap2D>();

    // Subscribe to the occupancy map
    occ_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/traversability_occ_map", rclcpp::SensorDataQoS(),
        std::bind(&OccMapToCostmapNode::occMapCallback, this, std::placeholders::_1));
}

void OccMapToCostmapNode::occMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received OccupancyGrid message with width: %d, height: %d", msg->info.width, msg->info.height);

    unsigned int size_x = msg->info.width;
    unsigned int size_y = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    // Resize both costmaps to match the incoming occupancy grid
    costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    costmap_nav_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);

    // Convert the occupancy grid values to costmap values for the first costmap
    for (unsigned int y = 0; y < size_y; ++y)
    {
        for (unsigned int x = 0; x < size_x; ++x)
        {
            int8_t occ_value = msg->data[y * size_x + x];
            unsigned char cost_value = FREE_SPACE;

            if (occ_value == -1)
            {
                cost_value = NO_INFORMATION;
            }
            else if (occ_value == 0)
            {
                cost_value = FREE_SPACE;
            }
            else if (occ_value == 100)
            {
                cost_value = LETHAL_OBSTACLE;
            }
            else
            {
                cost_value = static_cast<unsigned char>((occ_value / 100.0) * 254);
            }

            costmap_->setCost(x, y, cost_value);
        }
    }

    // Publish the initial costmap (converted from occupancy map)
    publishCostmap(costmap_, "costmap");

    // Apply inflation to the second costmap (`costmap_nav`) using the first costmap as the static layer
    applyStaticLayerAndInflation();

    // Publish the second costmap (inflation applied)
    publishCostmap(costmap_nav_, "costmap_nav");
}

void OccMapToCostmapNode::applyStaticLayerAndInflation()
{
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    int inflation_cells = static_cast<int>(INFLATION_RADIUS / resolution);

    // Copy the first costmap (`costmap_`) to the second costmap (`costmap_nav_`)
    for (unsigned int y = 0; y < size_y; ++y)
    {
        for (unsigned int x = 0; x < size_x; ++x)
        {
            costmap_nav_->setCost(x, y, costmap_->getCost(x, y));
        }
    }

    // Use BFS-like method to apply inflation starting from all occupied cells
    std::queue<std::pair<int, int>> cell_queue;
    for (unsigned int y = 0; y < size_y; ++y)
    {
        for (unsigned int x = 0; x < size_x; ++x)
        {
            unsigned char current_cost = costmap_nav_->getCost(x, y);
            // Only push cells that are occupied (exclude FREE_SPACE and NO_INFORMATION)
            if (current_cost != FREE_SPACE && current_cost != NO_INFORMATION)
            {
                cell_queue.push({x, y});
            }
        }
    }

    // Inflate around occupied cells
    while (!cell_queue.empty())
    {
        auto [cell_x, cell_y] = cell_queue.front();
        cell_queue.pop();

        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
        {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
            {
                int new_x = cell_x + dx;
                int new_y = cell_y + dy;

                if (new_x >= 0 && new_x < static_cast<int>(size_x) &&
                    new_y >= 0 && new_y < static_cast<int>(size_y))
                {
                    double distance = std::hypot(dx * resolution, dy * resolution);
                    if (distance <= INFLATION_RADIUS)
                    {
                        unsigned char neighbor_cost = costmap_nav_->getCost(new_x, new_y);

                        // Inflate only if the neighboring cell is free space
                        if (neighbor_cost == FREE_SPACE)
                        {
                            // Calculate the inflation cost based on distance
                            unsigned char inflated_cost = static_cast<unsigned char>(INSCRIBED_OBSTACLE - 
                                (INSCRIBED_OBSTACLE - FREE_SPACE) * (distance / INFLATION_RADIUS));

                            // Only update if the new inflated cost is higher
                            if (costmap_nav_->getCost(new_x, new_y) < inflated_cost)
                            {
                                costmap_nav_->setCost(new_x, new_y, inflated_cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

void OccMapToCostmapNode::publishCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, const std::string &topic_name)
{
    auto costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    // Fill in the OccupancyGrid message header
    costmap_msg->header.frame_id = "odom";
    costmap_msg->header.stamp = this->get_clock()->now();
    costmap_msg->info.width = costmap->getSizeInCellsX();
    costmap_msg->info.height = costmap->getSizeInCellsY();
    costmap_msg->info.resolution = costmap->getResolution();
    costmap_msg->info.origin.position.x = costmap->getOriginX();
    costmap_msg->info.origin.position.y = costmap->getOriginY();
    costmap_msg->info.origin.orientation.w = 1.0;

    // Resize the OccupancyGrid data to match the costmap size
    costmap_msg->data.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());

    // Copy the costmap values into the OccupancyGrid message data
    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y)
    {
        for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x)
        {
            costmap_msg->data[y * costmap->getSizeInCellsX() + x] = costmap->getCost(x, y);
        }
    }

    // Publish the message to the specified topic
    if (topic_name == "costmap")
    {
        costmap_pub_->publish(*costmap_msg);
    }
    else if (topic_name == "costmap_nav")
    {
        costmap_nav_pub_->publish(*costmap_msg);
    }
}

} // namespace scout_navigation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<scout_navigation::OccMapToCostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

