#include "scout_navigation/cost_calculator.hpp"

DistanceCalculator::DistanceCalculator(const Eigen::Vector2d& start, const Eigen::Vector2d& goal)
    : start(start), start_tmp(start), goal(goal), current(start) {}

void DistanceCalculator::state_update(const Eigen::Vector2d& start, const Eigen::Vector2d& current) {
    start_tmp = start;
    this->current = current;
}

double DistanceCalculator::euclidean_dist() const {
    return std::round((current - start_tmp).norm() * 100) / 100.0;
}

double DistanceCalculator::manhattan_dist() const {
    double dist = 0.0;
    for (int i = 0; i < 2; ++i) {
        dist += std::abs(goal[i] - current[i]);
    }
    // return std::round(dist * 100) / 100.0;
    return std::round((current - goal).norm() * 100) / 100.0;
}


Eigen::MatrixXd DistanceCalculator::createCostMap(int rows, int cols) {
    Eigen::MatrixXd cost_map(rows, cols);
    
    // 난수를 생성하기 위한 random 엔진
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1.0, 10.0);  // 1.0 ~ 10.0 사이의 난수
    
    // 장애물 좌표 정의 (3, -3)부터 (3, 6)까지
    int obstacle_x = 7;
    int obstacle_y_min = 4;  // 행렬 좌표에서는 음수 값이 없기 때문에, 적절히 변환합니다.
    int obstacle_y_max = 7;  // 좌표 6을 매트릭스 좌표에 맞게 변환
    
    // 임의의 코스트 맵 생성
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // 장애물 위치에는 큰 값을 부여
            if (i == obstacle_x && j >= obstacle_y_min && j <= obstacle_y_max) {
                cost_map(i, j) = 1000.0;  // 장애물 코스트
            } else {
                cost_map(i, j) = dis(gen);  // 1.0에서 10.0 사이의 임의의 코스트
            }
        }
    }

    return cost_map;
}