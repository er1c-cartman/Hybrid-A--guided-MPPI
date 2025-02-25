#include <vector>
#include <queue>
#include <Eigen/Dense>

// Node 구조체: A* 알고리즘에서 사용되는 경로 정보 (좌표, 비용, 휴리스틱)
struct Node {
    int x, y;  // x, y 좌표
    double cost, heuristic;  // 비용과 휴리스틱 값

    Node(int x, int y, double cost, double heuristic) : x(x), y(y), cost(cost), heuristic(heuristic) {}

    bool operator<(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);  // 비용이 작을수록 높은 우선순위
    }
};

// Astar 클래스 선언
class Astar {
public:
    Astar();  // 생성자

    // A* 알고리즘 경로 계획 함수
    std::vector<Node> a_star(const Eigen::MatrixXi& M, const Node& qS, const Node& qG);

    // 로컬 목표 찾기
    Node find_local_goal(const Eigen::MatrixXi& M, const Node& current_position, const Node& qG, int local_size);

    // 로컬 맵 생성
    void generate_local_map(Eigen::MatrixXi& M, const Eigen::MatrixXi& traversability_map);

    // 반복 경로 계획 함수
    std::vector<Node> iterative_planner(const Eigen::MatrixXi& M, const Node& qS, const Node& qG, const Eigen::MatrixXi& traversability_map);
};
