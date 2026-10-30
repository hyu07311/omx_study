// OMY-F3M 6축 체인의 FK / 기하 야코비안 / DLS 역기구학
//   omy_bringup/scripts/omy_ik_demo.py 의 Kinematics, ik_with_restarts 를 Eigen 으로 1:1 이식
//   기구학 정보는 URDF 문자열에서 직접 읽음 (치수 하드코딩 없음)
#pragma once

#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <vector>

namespace omy_motion
{

constexpr int kDof = 6;
using Vector6d = Eigen::Matrix<double, kDof, 1>;
using Matrix6d = Eigen::Matrix<double, 6, kDof>;

// 6주차 ik() 기본값 그대로
struct IkOptions
{
  double damping = 0.05;
  int max_iter = 500;
  double tol_pos = 1e-4;   // m
  double tol_rot = 1e-3;   // rad
  double max_step = 0.2;   // rad (|dq| 상한)
};

struct IkResult
{
  bool success = false;
  Vector6d q = Vector6d::Zero();
  int iterations = 0;
  double pos_err = 0.0;    // m
  double rot_err = 0.0;    // rad
};

// r_current 를 r_target 으로 돌리는 회전의 축각 벡터 (axis * angle)
Eigen::Vector3d rotationError(const Eigen::Matrix3d & r_target, const Eigen::Matrix3d & r_current);

// 각도를 -pi ~ pi 로 (Python 의 (q + pi) % 2pi - pi 와 같은 floor 나머지)
Vector6d wrap(const Vector6d & q);

class Kinematics
{
public:
  // base → tip 체인. 회전관절이 정확히 6개가 아니면 예외
  Kinematics(const std::string & urdf_xml, const std::string & base = "link0",
             const std::string & tip = "end_effector_link");

  const std::vector<std::string> & jointNames() const { return joint_names_; }
  const Vector6d & lower() const { return lower_; }
  const Vector6d & upper() const { return upper_; }

  Eigen::Isometry3d fk(const Vector6d & q) const;
  // FK 한 번으로 끝점 변환과 야코비안 6x6 (열 i = [z_i x (p_e - p_i); z_i])
  Eigen::Isometry3d fkJacobian(const Vector6d & q, Matrix6d & jac) const;
  Matrix6d jacobian(const Vector6d & q) const;

  Vector6d clip(const Vector6d & q) const { return q.cwiseMax(lower_).cwiseMin(upper_); }

  // 위치 오차 (m), 자세 오차 (rad)
  std::pair<double, double> poseError(const Vector6d & q, const Eigen::Isometry3d & target) const;

  // 감쇠 최소자승 (Levenberg-Marquardt 형) 반복. 수렴 못하면 success=false
  IkResult ik(const Eigen::Isometry3d & target, const Vector6d & q0,
              const IkOptions & opt = IkOptions()) const;

  // ik_with_restarts 와 같은 순서로 초기값을 바꿔 가며 풀고 q_now 에 가장 가까운 해
  //   1) q_now  2) joint6 만 +-90, +-180 도 돌린 자세  3) random_seeds (위에서 못 풀었을 때만, 해 3개면 중단)
  //   random_seeds 가 비어 있으면 내부 난수 (mt19937_64, seed) 로 tries 개 생성
  std::optional<Vector6d> ikWithRestarts(const Eigen::Isometry3d & target, const Vector6d & q_now,
                                         const std::vector<Vector6d> & random_seeds = {},
                                         int tries = 30, unsigned seed = 0,
                                         const IkOptions & opt = IkOptions()) const;

  // 샘플 범위: 관절 범위를 +-pi 로 자름 (IK 무작위 시드, RRT 샘플에 같이 사용)
  Vector6d sampleLower() const { return lower_.cwiseMax(-M_PI); }
  Vector6d sampleUpper() const { return upper_.cwiseMin(M_PI); }

private:
  struct Element
  {
    Eigen::Isometry3d origin;
    bool revolute;
    Eigen::Vector3d axis;
  };
  std::vector<Element> chain_;
  std::vector<std::string> joint_names_;
  Vector6d lower_, upper_;
};

// 손끝이 아래를 향하는 목표 자세 (end_effector_link 의 -y 가 아래, yaw 는 z 축 회전)
Eigen::Isometry3d gripperDownPose(double x, double y, double z, double yaw = 0.0);

// R = Rz(yaw) Ry(pitch) Rx(roll) (URDF rpy 규칙)
Eigen::Matrix3d rpyToMatrix(double r, double p, double y);

}  // namespace omy_motion
