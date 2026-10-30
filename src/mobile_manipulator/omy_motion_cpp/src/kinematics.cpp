#include "omy_motion_cpp/kinematics.hpp"

#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace omy_motion
{

Eigen::Matrix3d rpyToMatrix(double r, double p, double y)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d & r_target, const Eigen::Matrix3d & r_current)
{
  // AngleAxis 변환은 180 도 근처도 안정적 (Python 은 이 구간만 고유벡터로 따로 처리)
  const Eigen::AngleAxisd aa(r_target * r_current.transpose());
  if (aa.angle() < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  return aa.axis() * aa.angle();
}

Vector6d wrap(const Vector6d & q)
{
  Vector6d out;
  for (int i = 0; i < kDof; ++i) {
    const double x = q[i] + M_PI;
    out[i] = x - 2.0 * M_PI * std::floor(x / (2.0 * M_PI)) - M_PI;
  }
  return out;
}

Kinematics::Kinematics(const std::string & urdf_xml, const std::string & base, const std::string & tip)
{
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("failed to parse URDF");
  }
  // tip 에서 base 까지 부모를 따라 올라간 뒤 뒤집음 (= urdf_parser_py 의 get_chain)
  std::vector<urdf::JointConstSharedPtr> joints;
  for (auto link = model.getLink(tip); link && link->name != base; link = link->getParent()) {
    if (!link->parent_joint) {
      throw std::runtime_error("link " + tip + " is not below " + base);
    }
    joints.push_back(link->parent_joint);
  }
  std::reverse(joints.begin(), joints.end());

  std::vector<double> lower, upper;
  for (const auto & j : joints) {
    const auto & o = j->parent_to_joint_origin_transform;
    double r, p, y;
    o.rotation.getRPY(r, p, y);
    Element e;
    e.origin = Eigen::Isometry3d::Identity();
    e.origin.linear() = rpyToMatrix(r, p, y);
    e.origin.translation() = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);
    e.axis = Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z).normalized();
    e.revolute = j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS;
    if (e.revolute) {
      joint_names_.push_back(j->name);
      const bool continuous = j->type == urdf::Joint::CONTINUOUS;
      lower.push_back(continuous ? -M_PI : j->limits->lower);
      upper.push_back(continuous ? M_PI : j->limits->upper);
    } else if (j->type != urdf::Joint::FIXED) {
      throw std::runtime_error("unsupported joint type in chain: " + j->name);
    }
    chain_.push_back(e);
  }
  if (joint_names_.size() != kDof) {
    throw std::runtime_error("expected 6 revolute joints, got " + std::to_string(joint_names_.size()));
  }
  for (int i = 0; i < kDof; ++i) {
    lower_[i] = lower[i];
    upper_[i] = upper[i];
  }
}

Eigen::Isometry3d Kinematics::fk(const Vector6d & q) const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  int i = 0;
  for (const auto & e : chain_) {
    t = t * e.origin;
    if (e.revolute) {
      t.linear() = t.linear() * Eigen::AngleAxisd(q[i++], e.axis).toRotationMatrix();
    }
  }
  return t;
}

Eigen::Isometry3d Kinematics::fkJacobian(const Vector6d & q, Matrix6d & jac) const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 3, kDof> z, p;
  int i = 0;
  for (const auto & e : chain_) {
    t = t * e.origin;
    if (e.revolute) {
      z.col(i) = t.linear() * e.axis;
      p.col(i) = t.translation();
      t.linear() = t.linear() * Eigen::AngleAxisd(q[i], e.axis).toRotationMatrix();
      ++i;
    }
  }
  const Eigen::Vector3d p_end = t.translation();
  for (int k = 0; k < kDof; ++k) {
    jac.block<3, 1>(0, k) = z.col(k).cross(p_end - p.col(k));
    jac.block<3, 1>(3, k) = z.col(k);
  }
  return t;
}

Matrix6d Kinematics::jacobian(const Vector6d & q) const
{
  Matrix6d jac;
  fkJacobian(q, jac);
  return jac;
}

std::pair<double, double> Kinematics::poseError(const Vector6d & q, const Eigen::Isometry3d & target) const
{
  const Eigen::Isometry3d t = fk(q);
  return {(target.translation() - t.translation()).norm(),
          rotationError(target.linear(), t.linear()).norm()};
}

IkResult Kinematics::ik(const Eigen::Isometry3d & target, const Vector6d & q0, const IkOptions & opt) const
{
  IkResult res;
  Vector6d q = clip(q0);
  const Eigen::Matrix<double, 6, 6> damp = opt.damping * opt.damping * Eigen::Matrix<double, 6, 6>::Identity();
  Matrix6d jac;
  Vector6d err;
  for (int it = 0; it < opt.max_iter; ++it) {
    // 같은 반복에서 Python 은 fk 를 두 번 (오차, 야코비안) 부르지만 여기서는 한 번
    const Eigen::Isometry3d t = fkJacobian(q, jac);
    err.head<3>() = target.translation() - t.translation();
    err.tail<3>() = rotationError(target.linear(), t.linear());
    res.pos_err = err.head<3>().norm();
    res.rot_err = err.tail<3>().norm();
    res.iterations = it;
    if (res.pos_err < opt.tol_pos && res.rot_err < opt.tol_rot) {
      res.success = true;
      res.q = q;
      return res;
    }
    Vector6d dq = jac.transpose() * (jac * jac.transpose() + damp).ldlt().solve(err);
    const double norm = dq.norm();
    if (norm > opt.max_step) {
      dq *= opt.max_step / norm;
    }
    q = clip(q + dq);
  }
  res.iterations = opt.max_iter;
  res.q = q;
  return res;
}

std::optional<Vector6d> Kinematics::ikWithRestarts(
  const Eigen::Isometry3d & target, const Vector6d & q_now,
  const std::vector<Vector6d> & random_seeds, int tries, unsigned seed, const IkOptions & opt) const
{
  std::vector<Vector6d> seeds{q_now};
  for (double dq6 : {M_PI / 2, -M_PI / 2, M_PI, -M_PI}) {
    Vector6d q = q_now;
    q[5] += dq6;
    seeds.push_back(clip(q));
  }
  const size_t n_fixed = seeds.size();
  if (random_seeds.empty()) {
    std::mt19937_64 rng(seed);
    const Vector6d lo = sampleLower(), hi = sampleUpper();
    for (int k = 0; k < tries; ++k) {
      Vector6d q;
      for (int i = 0; i < kDof; ++i) {
        q[i] = std::uniform_real_distribution<double>(lo[i], hi[i])(rng);
      }
      seeds.push_back(q);
    }
  } else {
    seeds.insert(seeds.end(), random_seeds.begin(), random_seeds.end());
  }

  std::vector<Vector6d> solutions;
  for (size_t i = 0; i < seeds.size(); ++i) {
    if (i >= n_fixed && solutions.size() >= 3) {
      break;
    }
    if (i == n_fixed && !solutions.empty()) {
      break;
    }
    const IkResult r = ik(target, seeds[i], opt);
    if (!r.success) {
      continue;
    }
    // joint3 (+-150 도) 를 제외한 관절은 지금 각도에 가장 가까운 2pi 배수로 맞춤
    const Vector6d wrapped = q_now + wrap(r.q - q_now);
    Vector6d q;
    for (int k = 0; k < kDof; ++k) {
      q[k] = std::abs(upper_[k]) > M_PI ? wrapped[k] : r.q[k];
    }
    solutions.push_back(clip(q));
  }
  if (solutions.empty()) {
    return std::nullopt;
  }
  return *std::min_element(solutions.begin(), solutions.end(), [&](const Vector6d & a, const Vector6d & b) {
    return (a - q_now).norm() < (b - q_now).norm();
  });
}

Eigen::Isometry3d gripperDownPose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = rpyToMatrix(0.0, 0.0, yaw) * rpyToMatrix(M_PI / 2, 0.0, 0.0);
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

}  // namespace omy_motion
