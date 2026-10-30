// MoveIt PlanningScene (FCL + SRDF 의 허용 충돌 행렬) 을 감싼 충돌 판정
//   우리 RRT-Connect / Cartesian 경로 / 최종 검증이 모두 이 클래스 하나로 충돌을 봄
//   MoveIt OMPL 과 같은 PlanningScene 객체를 쓰므로 판정 기준이 같음
#pragma once

#include "omy_motion_cpp/kinematics.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace omy_motion
{

class CollisionChecker
{
public:
  CollisionChecker(const moveit::core::RobotModelConstPtr & model, const std::string & group = "arm");

  planning_scene::PlanningScenePtr scene() const { return scene_; }
  const moveit::core::JointModelGroup * group() const { return jmg_; }
  // link0 기준 world 상자 (OMY 는 world_fixed 가 원점이라 같은 좌표)
  void addBox(const std::string & id, const Eigen::Vector3d & size, const Eigen::Vector3d & center,
              double padding = 0.0);
  void removeObject(const std::string & id);
  // world 에 있는 물체를 q 자세에서 link 에 붙임 (touch_links 와의 접촉은 허용)
  void attach(const std::string & id, const std::string & link, const std::vector<std::string> & touch_links,
              const Vector6d & q);
  // 붙인 물체를 q 자세에서의 위치 그대로 world 로 되돌림
  void detach(const std::string & id, const Vector6d & q);

  // 물체와 링크들 사이 접촉 허용 여부 (허용 충돌 행렬). 집기 직전 손가락-물체 접근에 사용
  void allowCollision(const std::string & id, const std::vector<std::string> & links, bool allowed);
  // 그리퍼 링크 이름 (rh_p12_rn_*)
  std::vector<std::string> gripperLinks() const;

  // 그리퍼 (rh_r1_joint, mimic 포함) 각도. 충돌 판정에 같이 반영
  void setGripper(double angle) { gripper_ = angle; }
  double gripper() const { return gripper_; }

  bool isValid(const Vector6d & q) const;
  // a → b 직선을 resolution 간격 (관절 거리, metric 과 같은 단위) 으로 나눠 검사
  //   OMPL DiscreteMotionValidator 와 같이 가운데부터 이분 순서로 검사 (막힌 구간을 빨리 찾음)
  bool isMotionValid(const Vector6d & a, const Vector6d & b) const;
  // 부딪힌 링크 쌍 (보고용)
  std::set<std::pair<std::string, std::string>> contacts(const Vector6d & q) const;

  // MoveIt JointModelGroup::distance 와 같은 거리 (관절별 |차이| 의 합)
  static double distance(const Vector6d & a, const Vector6d & b) { return (a - b).cwiseAbs().sum(); }
  // 관절 범위 크기의 합 (OMPL getMaximumExtent)
  double maximumExtent() const { return jmg_->getMaximumExtent(); }

  // 간선 검사 간격. 기본 = MoveIt 설정 longest_valid_segment_fraction (0.005) x 최대 크기
  void setResolution(double r) { resolution_ = r; }
  double resolution() const { return resolution_; }

  Vector6d lower() const { return lower_; }
  Vector6d upper() const { return upper_; }

  size_t stateChecks() const { return checks_; }
  void resetCounter() { checks_ = 0; }

private:
  void setState(const Vector6d & q) const;

  planning_scene::PlanningScenePtr scene_;
  const moveit::core::JointModelGroup * jmg_;
  mutable moveit::core::RobotState state_;
  std::vector<std::string> gripper_joints_;
  double gripper_ = 0.0;
  double resolution_;
  Vector6d lower_, upper_;
  mutable size_t checks_ = 0;
};

}  // namespace omy_motion
