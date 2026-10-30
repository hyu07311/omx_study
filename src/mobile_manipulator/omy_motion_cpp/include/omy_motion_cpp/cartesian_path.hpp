// 손끝 직선 경로 (MoveIt computeCartesianPath 와 같은 의미의 직접 구현)
//   직선을 step 간격으로 나누고 자세는 slerp → 점마다 직전 해를 시드로 DLS IK
//   점 사이 관절 점프, 점·구간 충돌을 검사해 막히면 거기서 멈추고 fraction 반환
#pragma once

#include "omy_motion_cpp/collision_checker.hpp"
#include "omy_motion_cpp/kinematics.hpp"

#include <vector>

namespace omy_motion
{

struct CartesianOptions
{
  double step = 0.005;           // m (7주차 CARTESIAN_STEP)
  double max_rot_step = 0.05;    // rad, 자세가 많이 바뀌면 이 간격으로 더 잘게
  double jump_threshold = 0.3;   // rad, 이웃한 두 점 사이 관절 하나라도 이보다 크면 실패
};

struct CartesianResult
{
  double fraction = 0.0;          // 풀린 비율 (1 = 끝까지)
  std::vector<Vector6d> path;     // 시작 관절각 포함
  double ms = 0.0;
  double max_dev_mm = 0.0;        // 풀린 점들의 손끝이 직선에서 벗어난 최대 거리
  std::string failure;            // 실패 이유 (ik / jump / collision)
  Vector6d q_fail = Vector6d::Zero();  // 실패한 점의 IK 해 (collision 이면 원인 확인용)
};

CartesianResult cartesianPath(const Kinematics & kin, const CollisionChecker & checker, const Vector6d & q_start,
                              const Eigen::Isometry3d & target, const CartesianOptions & opt = CartesianOptions());

// 점에서 직선 p0-p1 까지 거리 (7주차 line_deviation 과 같은 식)
double lineDeviation(const Eigen::Vector3d & p, const Eigen::Vector3d & p0, const Eigen::Vector3d & p1);

}  // namespace omy_motion
