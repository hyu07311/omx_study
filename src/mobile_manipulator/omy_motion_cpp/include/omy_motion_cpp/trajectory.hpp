// 관절 경유점 → 시간 궤적 (사다리꼴 속도, 관절별 속도·가속도 한계)
//   stop_at_waypoints = true  (RRT-Connect 경로)
//     꼭짓점마다 멈췄다 출발 → 실제 궤적이 충돌 검사한 관절 직선과 정확히 같음
//     (TOTG 는 꼭짓점을 둥글게 깎아 검사하지 않은 곳을 지나갈 수 있음 - 7주차 padding 원인)
//   stop_at_waypoints = false (Cartesian 경로, 5 mm 간격 촘촘한 점)
//     전체를 한 번의 사다리꼴로 지나감 (점 사이 방향 변화가 작아서 모서리 가속도는 무시)
#pragma once

#include "omy_motion_cpp/kinematics.hpp"

#include <vector>

namespace omy_motion
{

struct JointLimits
{
  Vector6d max_velocity;
  Vector6d max_acceleration;
};

// omy_moveit_config/config/joint_limits.yaml 값
JointLimits omyJointLimits();

struct TimedTrajectory
{
  std::vector<double> t;
  std::vector<Vector6d> q;
  std::vector<Vector6d> qd;
  double duration() const { return t.empty() ? 0.0 : t.back(); }
  // 다른 궤적을 시간 이어서 붙임 (첫 점은 이 궤적의 끝점과 같아야 함)
  void append(const TimedTrajectory & other);
};

TimedTrajectory timeParameterize(const std::vector<Vector6d> & path, const JointLimits & limits,
                                 double scaling, bool stop_at_waypoints, double dt = 0.01);

}  // namespace omy_motion
