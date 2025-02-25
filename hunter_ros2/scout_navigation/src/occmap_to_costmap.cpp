#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "scout_navigation/occmap_to_costmap.hpp"

class OccMapToCostmapNode : public rclcpp::Node
{
public:
    OccMapToCostmapNode()
    : Node("occmap_to_costmap")
    {
        occmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/traversability_occ_map", 10, std::bind(&OccMapToCostmapNode::occMapCallback, this, std::placeholders::_1));
        
        // Create publisher to publish the costmap
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

        // Initialize costmaps
        costmap_ = std::make_shared<Costmap>();
        costmap_nav_ = std::make_shared<Costmap>();
    }

private:
    void occMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
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

        // Convert the occupancy grid values to costmap values
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

        // Clear costs in the center region
        double clear_radius = 4.0;  // Define radius in cells
        costmap_->clearCenterCosts(clear_radius);

        // Publish the converted costmap
        nav_msgs::msg::OccupancyGrid costmap_msg;
        costmap_msg.header.stamp = this->get_clock()->now();
        costmap_msg.header.frame_id = "odom";  // Set the frame ID to match the incoming map

        costmap_msg.info.width = size_x;
        costmap_msg.info.height = size_y;
        costmap_msg.info.resolution = resolution;
        costmap_msg.info.origin.position.x = origin_x;
        costmap_msg.info.origin.position.y = origin_y;

        // Fill the costmap message data
        costmap_msg.data.resize(size_x * size_y);
        for (unsigned int y = 0; y < size_y; ++y)
        {
            for (unsigned int x = 0; x < size_x; ++x)
            {
                costmap_msg.data[y * size_x + x] = costmap_->getCost(x, y);
            }
        }

        costmap_pub_->publish(costmap_msg);

        //RCLCPP_INFO(this->get_logger(), "Costmap conversion and publishing complete with center cleared.");
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    std::shared_ptr<Costmap> costmap_;
    std::shared_ptr<Costmap> costmap_nav_;

    // Constants for cost values
    const unsigned char FREE_SPACE = 0;
    const unsigned char LETHAL_OBSTACLE = 254;
    const unsigned char NO_INFORMATION = 255;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccMapToCostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
