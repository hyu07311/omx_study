#include "omy_motion_cpp/cartesian_path.hpp"

#include "omy_motion_cpp/csv.hpp"

#include <algorithm>
#include <cmath>

namespace omy_motion
{

double lineDeviation(const Eigen::Vector3d & p, const Eigen::Vector3d & p0, const Eigen::Vector3d & p1)
{
  const Eigen::Vector3d d = (p1 - p0).normalized();
  const Eigen::Vector3d rel = p - p0;
  return (rel - rel.dot(d) * d).norm();
}

CartesianResult cartesianPath(const Kinematics & kin, const CollisionChecker & checker, const Vector6d & q_start,
                              const Eigen::Isometry3d & target, const CartesianOptions & opt)
{
  Stopwatch sw;
  CartesianResult res;
  res.path.push_back(q_start);
  const Eigen::Isometry3d start = kin.fk(q_start);
  const Eigen::Quaterniond r0(start.linear()), r1(target.linear());
  const Eigen::Vector3d p0 = start.translation(), p1 = target.translation();
  const int n = std::max({1, static_cast<int>(std::ceil((p1 - p0).norm() / opt.step)),
                          static_cast<int>(std::ceil(r0.angularDistance(r1) / opt.max_rot_step))});
  if (!checker.isValid(q_start)) {
    res.failure = "start in collision";
    res.ms = sw.ms();
    return res;
  }
  Vector6d q_prev = q_start;
  int solved = 0;
  for (int k = 1; k <= n; ++k) {
    const double s = static_cast<double>(k) / n;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = r0.slerp(s, r1).toRotationMatrix();
    pose.translation() = p0 + (p1 - p0) * s;
    const IkResult ik = kin.ik(pose, q_prev);
    if (!ik.success) {
      res.failure = "ik";
      break;
    }
    if ((ik.q - q_prev).cwiseAbs().maxCoeff() > opt.jump_threshold) {
      res.failure = "jump";
      break;
    }
    if (!checker.isMotionValid(q_prev, ik.q)) {
      res.failure = "collision";
      res.q_fail = ik.q;
      break;
    }
    res.path.push_back(ik.q);
    if ((p1 - p0).norm() > 1e-9) {
      res.max_dev_mm = std::max(res.max_dev_mm, 1000.0 * lineDeviation(kin.fk(ik.q).translation(), p0, p1));
    }
    q_prev = ik.q;
    solved = k;
  }
  res.fraction = static_cast<double>(solved) / n;
  res.ms = sw.ms();
  return res;
}

}  // namespace omy_motion
