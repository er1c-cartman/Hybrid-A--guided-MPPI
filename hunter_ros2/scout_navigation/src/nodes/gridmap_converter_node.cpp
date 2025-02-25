#include "scout_navigation/gridmap_converter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<scout_navigation::GridMapConverter>();

  if (!node->isInitialized())
  {
    RCLCPP_ERROR(node->get_logger(), "[Gridmap Converter] Node not initialized. Aborting...");
    return -1;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "[Gridmap Converter] Node initialized");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
