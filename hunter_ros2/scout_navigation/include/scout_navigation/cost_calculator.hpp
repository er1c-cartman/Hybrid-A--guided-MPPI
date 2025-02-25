#ifndef DISTANCE_CALCULATOR_HPP
#define DISTANCE_CALCULATOR_HPP

#include <Eigen/Dense>
#include <cmath>
#include <random> 

class DistanceCalculator {
public:
    DistanceCalculator(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
    
    void state_update(const Eigen::Vector2d& start, const Eigen::Vector2d& current);
    
    double euclidean_dist() const;
    
    double manhattan_dist() const;

    Eigen::MatrixXd createCostMap(int rows, int cols);


private:
    Eigen::Vector2d start;
    Eigen::Vector2d start_tmp;
    Eigen::Vector2d goal;
    Eigen::Vector2d current;
};

#endif