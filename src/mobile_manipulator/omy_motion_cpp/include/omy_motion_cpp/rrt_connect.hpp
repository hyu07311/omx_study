// 관절공간 RRT-Connect (Kuffner & LaValle 2000) + 무작위 shortcut
//   OMPL RRTConnect 와 같은 규칙으로 맞춤 (공정 비교)
//     거리      관절별 |차이| 의 합 (MoveIt JointModelGroup::distance)
//     range     0 이면 0.2 x 상태공간 최대 크기 (OMPL 기본)
//     간선 검사 CollisionChecker::isMotionValid (해상도 = longest_valid_segment_fraction x 최대 크기)
//     샘플      관절 범위 전체 균일 (OMPL 도 범위 전체)
#pragma once

#include "omy_motion_cpp/collision_checker.hpp"

#include <random>
#include <vector>

namespace omy_motion
{

struct RrtOptions
{
  double range = 0.0;          // 한 번에 뻗는 최대 거리 (0 = 자동)
  double timeout_s = 5.0;      // MoveIt allowed_planning_time 과 같음
  int shortcut_iterations = 200;
  unsigned seed = 0;
};

struct RrtResult
{
  bool success = false;
  std::vector<Vector6d> raw_path;   // 두 트리를 이은 경로
  std::vector<Vector6d> path;       // shortcut 후
  double plan_ms = 0.0;             // 트리 확장 (연결까지)
  double simplify_ms = 0.0;         // shortcut
  size_t nodes = 0;                 // 두 트리 노드 수 합
  size_t iterations = 0;
  size_t state_checks = 0;          // 충돌 판정 호출 횟수
};

double pathLength(const std::vector<Vector6d> & path);
// 경로를 관절 하나라도 max_step (rad) 보다 크게 움직이지 않도록 촘촘하게 (검증용)
std::vector<Vector6d> densify(const std::vector<Vector6d> & path, double max_step);
// 점마다 충돌 판정 → 충돌한 점 수
size_t countCollisions(const CollisionChecker & checker, const std::vector<Vector6d> & points);

class RrtConnect
{
public:
  RrtConnect(const CollisionChecker & checker, const RrtOptions & opt = RrtOptions());

  RrtResult plan(const Vector6d & start, const Vector6d & goal);
  // 경로의 두 점을 무작위로 골라 직선이 비어 있으면 사이 점을 지움 (+ 마지막에 불필요한 꼭짓점 제거)
  std::vector<Vector6d> shortcut(std::vector<Vector6d> path, int iterations);

private:
  struct Node
  {
    Vector6d q;
    int parent;
  };
  using Tree = std::vector<Node>;
  enum class Status { kTrapped, kAdvanced, kReached };

  void reduceVertices(std::vector<Vector6d> & path) const;
  int nearest(const Tree & tree, const Vector6d & q) const;
  Status extend(Tree & tree, const Vector6d & target);
  Status connect(Tree & tree, const Vector6d & target);
  Vector6d sample();

  const CollisionChecker & checker_;
  RrtOptions opt_;
  double range_;
  std::mt19937_64 rng_;
};

}  // namespace omy_motion
