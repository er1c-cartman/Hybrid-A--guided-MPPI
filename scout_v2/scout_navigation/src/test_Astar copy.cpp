#include "scout_navigation/test_Astar.hpp"
#include <cmath>
#include <queue>
#include <limits>
#include <fstream>
#include <vector>
#include <iostream>
#include "geometry_msgs/msg/pose_stamped.hpp"

// 경로 데이터를 txt 파일로 저장하는 함수
void savePathToFile(const std::string& filename, const std::vector<Node>& path) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& node : path) {
            file << node.x << " " << node.y << "\n";  // x, y 좌표를 한 줄씩 기록
        }
        file.close();
        std::cout << "경로가 " << filename << "에 저장되었습니다." << std::endl;
    } else {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
    }
}

// 생성자
Astar::Astar() {}

// 유클리디안 거리를 기반으로 목표까지의 휴리스틱 계산
double heuristic(int x, int y, const Node& qG) {
    return std::sqrt((x - qG.x) * (x - qG.x) + (y - qG.y) * (y - qG.y));
}

// 좌표가 맵의 범위 내에 있는지 확인하는 함수
bool is_valid(int x, int y, const Eigen::MatrixXi& M) {
    return (x >= 0 && y >= 0 && x < M.cols() && y < M.rows());
}

// 현재 노드에서 인접한 후속 노드를 반환하는 함수
std::vector<Node> get_successors(const Node& node, const Eigen::MatrixXi& M, const Node& qG) {
    std::vector<Node> successors;
    std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    
    for (const auto& dir : directions) {
        int new_x = node.x + dir.first;
        int new_y = node.y + dir.second;

        if (is_valid(new_x, new_y, M) && M(new_y, new_x) != 100) {
            double new_cost = node.cost + M(new_y, new_x);
            double h = heuristic(new_x, new_y, qG);
            successors.emplace_back(new_x, new_y, new_cost, h);
        }
    }
    return successors;
}

// A* 경로 계획 함수
std::vector<Node> Astar::a_star(const Eigen::MatrixXi& M, const Node& qS, const Node& qG) {
    std::priority_queue<Node> open_list;
    std::vector<Node> closed_list;
    open_list.emplace(qS);

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (std::hypot(current.x - qG.x, current.y - qG.y) < 0.01) {
            closed_list.push_back(current);
            return closed_list;
        }

        for (const auto& successor : get_successors(current, M, qG)) {
            open_list.push(successor);
        }

        closed_list.push_back(current);
    }

    return closed_list;
}

// 로컬 맵 내에서 목표에 가까운 임시 목표를 찾는 함수
Node Astar::find_local_goal(const Eigen::MatrixXi& M, const Node& current_position, const Node& qG, int local_size) {
    Node best_local_goal = qG;
    double min_cost = std::numeric_limits<double>::infinity();
    int half_size = local_size / 2;

    for (int i = -half_size; i <= half_size; ++i) {
        for (int j = -half_size; j <= half_size; ++j) {
            int local_x = current_position.x + i;
            int local_y = current_position.y + j;

            if (is_valid(local_x, local_y, M)) {
                double heuristic_cost = heuristic(local_x, local_y, qG);
                double map_cost = M(local_y, local_x);
                double total_cost = heuristic_cost + map_cost;

                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_local_goal = Node(local_x, local_y, map_cost, heuristic_cost);
                }
            }
        }
    }

    return best_local_goal;
}

// 트래버서빌리티 맵을 이용해 로컬 맵 생성
void Astar::generate_local_map(Eigen::MatrixXi& M, const Eigen::MatrixXi& traversability_map, const Node& current_position) {
    int half_size = traversability_map.rows() / 2;
    for (int i = 0; i < traversability_map.rows(); ++i) {
        for (int j = 0; j < traversability_map.cols(); ++j) {
            int local_x = current_position.x - half_size + i;
            int local_y = current_position.y - half_size + j;
            if (local_x >= 0 && local_x < M.rows() && local_y >= 0 && local_y < M.cols()) {
                M(local_x, local_y) = traversability_map(i, j);  // 로컬 맵을 로봇 중심으로 복사
            }
        }
    }
}

// 반복적인 경로 계획 함수
std::vector<Node> Astar::iterative_planner(const Eigen::MatrixXi& M, const Node& qS, const Node& qG, const Eigen::MatrixXi& traversability_map) {
    Node current_position = qS;
    std::vector<Node> full_path;
    full_path.push_back(current_position);

    while (std::hypot(current_position.x - qG.x, current_position.y - qG.y) > 0.01) {
        Eigen::MatrixXi local_map = M;  // M을 복사하여 로컬맵 생성
        generate_local_map(local_map, traversability_map, current_position);  // 로봇을 중심으로 로컬 맵 업데이트

        Node local_goal = find_local_goal(local_map, current_position, qG, traversability_map.rows());

        std::vector<Node> path = a_star(local_map, current_position, local_goal);

        if (path.size() > 1) {
            current_position = path[1];
            full_path.push_back(current_position);
        } else {
            break;
        }
    }

    // 경로를 파일에 저장
    savePathToFile("/home/home/paper_ws/src/path.txt", full_path);

    return full_path;
}


