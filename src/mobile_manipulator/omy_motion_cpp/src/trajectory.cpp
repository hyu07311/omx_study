#include "omy_motion_cpp/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace omy_motion
{

JointLimits omyJointLimits()
{
  JointLimits l;
  l.max_velocity << 3.372, 3.372, 6.0, 6.0, 6.0, 6.0;
  l.max_acceleration << 5.0, 5.0, 10.0, 10.0, 10.0, 10.0;
  return l;
}

void TimedTrajectory::append(const TimedTrajectory & other)
{
  const double t0 = duration();
  const size_t skip = t.empty() ? 0 : 1;
  for (size_t i = skip; i < other.t.size(); ++i) {
    t.push_back(t0 + other.t[i]);
    q.push_back(other.q[i]);
    qd.push_back(other.qd[i]);
  }
}

namespace
{

// 길이 L 을 최대 속도 v, 가속도 a 의 사다리꼴 (정지 → 정지) 로. 시각 t 에서의 (위치, 속도)
struct Trapezoid
{
  double L, v, a, t_acc, t_flat, T;
  Trapezoid(double L_, double v_, double a_) : L(L_), v(v_), a(a_)
  {
    if (L <= 0.0) {
      t_acc = t_flat = T = 0.0;
      return;
    }
    if (v * v / a > L) {   // 최고 속도에 못 미치는 삼각형
      v = std::sqrt(L * a);
    }
    t_acc = v / a;
    t_flat = (L - v * t_acc) / v;
    T = 2 * t_acc + t_flat;
  }
  std::pair<double, double> at(double t) const
  {
    if (T <= 0.0) {
      return {0.0, 0.0};
    }
    t = std::clamp(t, 0.0, T);
    if (t < t_acc) {
      return {0.5 * a * t * t, a * t};
    }
    if (t < t_acc + t_flat) {
      return {0.5 * a * t_acc * t_acc + v * (t - t_acc), v};
    }
    const double r = T - t;
    return {L - 0.5 * a * r * r, a * r};
  }
};

// 구간 i 를 지나는 데 필요한 "최고 속도 기준 시간" (경로 매개변수 s 의 단위)
double segmentLength(const Vector6d & dq, const Vector6d & vmax)
{
  return dq.cwiseAbs().cwiseQuotient(vmax).maxCoeff();
}

// 경유점 [first, last] 를 한 번의 사다리꼴로
TimedTrajectory profile(const std::vector<Vector6d> & path, size_t first, size_t last, const JointLimits & lim,
                        double scaling, double dt)
{
  const Vector6d vmax = lim.max_velocity * scaling;
  const Vector6d amax = lim.max_acceleration * scaling;
  std::vector<double> s{0.0};
  for (size_t i = first + 1; i <= last; ++i) {
    s.push_back(s.back() + segmentLength(path[i] - path[i - 1], vmax));
  }
  // s 의 속도 1 = 어떤 구간에서든 가장 빠른 관절이 vmax. 가속도는 관절별 amax/vmax 의 최솟값으로 제한
  const double a_s = amax.cwiseQuotient(vmax).minCoeff();
  const Trapezoid tp(s.back(), 1.0, a_s);
  TimedTrajectory out;
  const int n = std::max(1, static_cast<int>(std::ceil(tp.T / dt)));
  size_t seg = 0;
  for (int k = 0; k <= n; ++k) {
    const double t = std::min(k * dt, tp.T);
    const auto [si, vi] = tp.at(t);
    while (seg + 2 < s.size() && si > s[seg + 1]) {
      ++seg;
    }
    const double len = s[seg + 1] - s[seg];
    const Vector6d dq = path[first + seg + 1] - path[first + seg];
    const double u = len > 0.0 ? std::clamp((si - s[seg]) / len, 0.0, 1.0) : 1.0;
    out.t.push_back(t);
    out.q.push_back(path[first + seg] + dq * u);
    out.qd.push_back(len > 0.0 ? Vector6d(dq * (vi / len)) : Vector6d::Zero());
  }
  return out;
}

}  // namespace

TimedTrajectory timeParameterize(const std::vector<Vector6d> & path, const JointLimits & limits, double scaling,
                                 bool stop_at_waypoints, double dt)
{
  TimedTrajectory out;
  if (path.size() < 2) {
    if (!path.empty()) {
      out.t = {0.0};
      out.q = {path[0]};
      out.qd = {Vector6d::Zero()};
    }
    return out;
  }
  if (!stop_at_waypoints) {
    return profile(path, 0, path.size() - 1, limits, scaling, dt);
  }
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    out.append(profile(path, i, i + 1, limits, scaling, dt));
  }
  return out;
}

}  // namespace omy_motion
