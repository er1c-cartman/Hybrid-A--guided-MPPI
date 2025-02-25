#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "test_Astar.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>


// 로컬맵을 저장할 행렬
Eigen::MatrixXi traversability_map(240, 240);

// OccupancyGrid 맵을 파일로 저장하는 함수
void saveMatrixToFile(const std::string& filename, const Eigen::MatrixXi& matrix) {
    std::ofstream file(filename);
    
    if (file.is_open()) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << matrix(i, j);
                if (j != matrix.cols() - 1) {
                    file << " "; // 열 사이에 공백 추가
                }
            }
            file << "\n"; // 행이 끝나면 개행
        }
        file.close();
        std::cout << "Matrix saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

// costMapCB: OccupancyGrid 메시지를 수신하여 맵을 업데이트
void costMapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int width = msg->info.width;
    int height = msg->info.height;
    traversability_map.resize(height, width);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            traversability_map(y, x) = msg->data[idx];
        }
    }
    // is_getting_Map = true;
    // 맵을 파일로 저장 (OccupancyGrid 데이터를 텍스트 파일로 저장)
    saveMatrixToFile("/home/home/paper_ws/src/occ_map.txt", traversability_map);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("astar_planner_node");
    // is_getting_Map = false;
    Astar astar_planner;

    Node qS(200, 200, 0.0, 0.0);
    Node qG(25, 100, 0.0, 0.0);

    auto subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "traversability_occ_map", 10, costMapCB);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        auto full_path = astar_planner.iterative_planner(traversability_map, qS, qG, traversability_map);

        for (const auto& node : full_path) {
            std::cout << "Node at: (" << node.cost << ", " << node.heuristic << ")" << std::endl;
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
