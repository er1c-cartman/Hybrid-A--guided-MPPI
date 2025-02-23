#include <fstream>
#include <memory>
#include "scout_navigation/test_Astar.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

void costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
void saveMatrixToFile(const std::string& filename,const Eigen::MatrixXd& matrix);
Eigen::MatrixXi costMap_;