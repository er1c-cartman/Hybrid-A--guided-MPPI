#include <vector>
#include <queue>
#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"

// Node 구조체: A* 알고리즘에서 사용되는 경로 정보 (좌표, 비용, 휴리스틱)
struct Node {
    int x, y;             // x, y 좌표
    double cost, heuristic;  // 비용과 휴리스틱 값

    // 기본 생성자 추가
    Node() : x(0), y(0), cost(0.0), heuristic(0.0) {}

    // 매개변수가 있는 생성자
    Node(int x, int y, double cost, double heuristic) : x(x), y(y), cost(cost), heuristic(heuristic) {}

    // 우선순위 큐에서 비용 비교를 위한 연산자
    bool operator<(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);  // 비용이 작을수록 높은 우선순위
    }
};

class Astar {
public:
    Astar();

    std::vector<Node> a_star(const Eigen::MatrixXi& M, const Node& qS, const Node& qG);

    Node find_local_goal(const Eigen::MatrixXi& M, const Node& current_position, const Node& qG, int local_size);

    void generate_local_map(Eigen::MatrixXi& M, const Eigen::MatrixXi& traversability_map, const Node& current_position);  // 세 번째 인자로 current_position 추가

    std::vector<Node> iterative_planner(const Eigen::MatrixXi& M, const Node& qS, const Node& qG, const Eigen::MatrixXi& traversability_map);

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);  // pose callback 함수 추가

    Node current_position_;  // 현재 위치 저장
};
