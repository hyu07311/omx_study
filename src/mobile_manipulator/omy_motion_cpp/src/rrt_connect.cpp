#include "omy_motion_cpp/rrt_connect.hpp"

#include "omy_motion_cpp/csv.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace omy_motion
{

double pathLength(const std::vector<Vector6d> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += CollisionChecker::distance(path[i - 1], path[i]);
  }
  return len;
}

std::vector<Vector6d> densify(const std::vector<Vector6d> & path, double max_step)
{
  std::vector<Vector6d> out;
  if (path.empty()) {
    return out;
  }
  out.push_back(path[0]);
  for (size_t i = 1; i < path.size(); ++i) {
    const Vector6d d = path[i] - path[i - 1];
    const int n = std::max(1, static_cast<int>(std::ceil(d.cwiseAbs().maxCoeff() / max_step)));
    for (int k = 1; k <= n; ++k) {
      out.push_back(path[i - 1] + d * (static_cast<double>(k) / n));
    }
  }
  return out;
}

size_t countCollisions(const CollisionChecker & checker, const std::vector<Vector6d> & points)
{
  size_t n = 0;
  for (const auto & q : points) {
    n += !checker.isValid(q);
  }
  return n;
}

RrtConnect::RrtConnect(const CollisionChecker & checker, const RrtOptions & opt)
: checker_(checker), opt_(opt), rng_(opt.seed)
{
  range_ = opt.range > 0.0 ? opt.range : 0.2 * checker_.maximumExtent();
}

Vector6d RrtConnect::sample()
{
  Vector6d q;
  const Vector6d lo = checker_.lower(), hi = checker_.upper();
  for (int i = 0; i < kDof; ++i) {
    q[i] = std::uniform_real_distribution<double>(lo[i], hi[i])(rng_);
  }
  return q;
}

int RrtConnect::nearest(const Tree & tree, const Vector6d & q) const
{
  // 노드 수가 수백 개 수준이라 선형 탐색이 kd-tree 보다 단순하고 충분히 빠름
  int best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double d = CollisionChecker::distance(tree[i].q, q);
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

RrtConnect::Status RrtConnect::extend(Tree & tree, const Vector6d & target)
{
  const int near = nearest(tree, target);
  const Vector6d & q_near = tree[near].q;
  const double d = CollisionChecker::distance(q_near, target);
  const bool reach = d <= range_;
  const Vector6d q_new = reach ? target : Vector6d(q_near + (target - q_near) * (range_ / d));
  if (!checker_.isMotionValid(q_near, q_new)) {
    return Status::kTrapped;
  }
  tree.push_back({q_new, near});
  return reach ? Status::kReached : Status::kAdvanced;
}

RrtConnect::Status RrtConnect::connect(Tree & tree, const Vector6d & target)
{
  Status s;
  do {
    s = extend(tree, target);
  } while (s == Status::kAdvanced);
  return s;
}

RrtResult RrtConnect::plan(const Vector6d & start, const Vector6d & goal)
{
  RrtResult res;
  const size_t checks0 = checker_.stateChecks();
  Stopwatch sw;
  if (!checker_.isValid(start) || !checker_.isValid(goal)) {
    res.plan_ms = sw.ms();
    return res;
  }
  Tree ta{{start, -1}}, tb{{goal, -1}};
  bool a_is_start = true;
  while (sw.ms() < opt_.timeout_s * 1000.0) {
    ++res.iterations;
    const Vector6d q_rand = sample();
    if (extend(ta, q_rand) != Status::kTrapped) {
      // 방금 뻗은 점을 향해 반대 트리를 끝까지 이어 봄
      const Vector6d q_new = ta.back().q;
      if (connect(tb, q_new) == Status::kReached) {
        // ta 의 새 점 → 뿌리, tb 의 마지막 점 (= q_new) → 뿌리 를 이어 붙임
        std::vector<Vector6d> half_a, half_b;
        for (int i = static_cast<int>(ta.size()) - 1; i >= 0; i = ta[i].parent) {
          half_a.push_back(ta[i].q);
        }
        for (int i = tb[tb.size() - 1].parent; i >= 0; i = tb[i].parent) {
          half_b.push_back(tb[i].q);
        }
        std::reverse(half_a.begin(), half_a.end());
        // half_a = 뿌리(a) → q_new, half_b = q_new 다음 → 뿌리(b)
        half_a.insert(half_a.end(), half_b.begin(), half_b.end());
        if (!a_is_start) {
          std::reverse(half_a.begin(), half_a.end());
        }
        res.raw_path = half_a;
        res.success = true;
        break;
      }
    }
    std::swap(ta, tb);
    a_is_start = !a_is_start;
  }
  res.plan_ms = sw.ms();
  res.nodes = ta.size() + tb.size();
  if (res.success) {
    Stopwatch sw2;
    res.path = shortcut(res.raw_path, opt_.shortcut_iterations);
    res.simplify_ms = sw2.ms();
  }
  res.state_checks = checker_.stateChecks() - checks0;
  return res;
}

constexpr int kMaxEmptySteps = 50;

void RrtConnect::reduceVertices(std::vector<Vector6d> & path) const
{
  // 앞에서부터 건너뛸 수 있는 꼭짓점 제거 (OMPL reduceVertices 와 비슷)
  for (size_t i = 0; i + 2 < path.size();) {
    if (checker_.isMotionValid(path[i], path[i + 2])) {
      path.erase(path.begin() + i + 1);
    } else {
      ++i;
    }
  }
}

std::vector<Vector6d> RrtConnect::shortcut(std::vector<Vector6d> path, int iterations)
{
  // 먼저 꼭짓점을 줄여 두면 (빈 장면처럼 바로 이어지는 경우) 무작위 단계가 거의 필요 없음
  reduceVertices(path);
  // OMPL PathSimplifier::shortcutPath 와 같은 방식 - 꼭짓점이 아니라 경로 위의 임의의 두 점을 골라
  // 그 사이 직선이 비어 있고 더 짧으면 교체 (꼭짓점끼리만 이으면 크게 돌아가는 구간이 남음)
  int empty = 0;   // 연속으로 줄이지 못한 횟수 (OMPL maxEmptySteps)
  for (int it = 0; it < iterations && path.size() > 2 && empty < kMaxEmptySteps; ++it) {
    ++empty;
    std::vector<double> cum{0.0};
    for (size_t i = 1; i < path.size(); ++i) {
      cum.push_back(cum.back() + CollisionChecker::distance(path[i - 1], path[i]));
    }
    std::uniform_real_distribution<double> pick(0.0, cum.back());
    double s1 = pick(rng_), s2 = pick(rng_);
    if (s1 > s2) {
      std::swap(s1, s2);
    }
    // s 가 들어 있는 구간 i (cum[i] <= s < cum[i+1]) 와 그 위의 점
    auto locate = [&](double s, size_t & seg) {
      seg = std::min<size_t>(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin(), cum.size() - 1) - 1;
      const double len = cum[seg + 1] - cum[seg];
      const double u = len > 0.0 ? (s - cum[seg]) / len : 0.0;
      return Vector6d(path[seg] + (path[seg + 1] - path[seg]) * u);
    };
    size_t i1, i2;
    const Vector6d p1 = locate(s1, i1), p2 = locate(s2, i2);
    if (i1 == i2) {
      continue;
    }
    if (CollisionChecker::distance(p1, p2) >= s2 - s1 - 1e-9 || !checker_.isMotionValid(p1, p2)) {
      continue;
    }
    std::vector<Vector6d> next(path.begin(), path.begin() + i1 + 1);
    next.push_back(p1);
    next.push_back(p2);
    next.insert(next.end(), path.begin() + i2 + 1, path.end());
    path.swap(next);
    empty = 0;
  }
  reduceVertices(path);
  // 거의 겹친 점 제거
  for (size_t i = 1; i + 1 < path.size();) {
    if (CollisionChecker::distance(path[i - 1], path[i]) < 1e-6) {
      path.erase(path.begin() + i);
    } else {
      ++i;
    }
  }
  return path;
}

}  // namespace omy_motion
