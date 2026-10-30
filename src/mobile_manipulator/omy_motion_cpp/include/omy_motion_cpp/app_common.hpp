// 앱 (벤치마크 / 파이프라인) 공통 - 7주차 Python 코드의 상수, 파라미터 읽기
#pragma once

#include "omy_motion_cpp/csv.hpp"
#include "omy_motion_cpp/kinematics.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <string>
#include <vector>

namespace omy_motion
{

// omy_ik_demo.py READY_POSE
inline Vector6d readyPose()
{
  Vector6d q;
  q << 0.286, 0.454, 0.361, 0.755, 1.571, -1.857;
  return q;
}

// omy_moveit_compare.py 장애물 기본값
inline const Eigen::Vector3d kObstacleSize(0.10, 0.10, 0.30);
inline const Eigen::Vector3d kObstacleCenter(0.42, -0.10, 0.20);
constexpr double kObstaclePadding = 0.005;
constexpr const char * kObstacleId = "compare_obstacle";

inline std::string resultsDir(const std::string & sub)
{
  return std::string(std::getenv("HOME")) + "/robot_ws/motion_cpp_results/" + sub;
}

inline rclcpp::Node::SharedPtr makeNode(const std::string & name)
{
  return std::make_shared<rclcpp::Node>(
    name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
}

template<typename T>
T param(const rclcpp::Node::SharedPtr & node, const std::string & name, T def)
{
  return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() : def;
}

inline Eigen::Vector3d paramVec3(const rclcpp::Node::SharedPtr & node, const std::string & name,
                                 const Eigen::Vector3d & def)
{
  if (!node->has_parameter(name)) {
    return def;
  }
  const auto v = node->get_parameter(name).as_double_array();
  return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
}

// export_ik_reference.py 가 저장한 ik_with_restarts 무작위 시드 (없으면 빈 목록 → 내부 난수)
inline std::vector<Vector6d> loadRestartSeeds()
{
  std::vector<Vector6d> seeds;
  try {
    const CsvTable t = readCsv(resultsDir("ik") + "/restart_seeds.csv");
    for (size_t r = 0; r < t.rows.size(); ++r) {
      Vector6d q;
      for (int k = 0; k < kDof; ++k) {
        q[k] = t.num(r, "q" + std::to_string(k));
      }
      seeds.push_back(q);
    }
  } catch (const std::exception &) {
  }
  return seeds;
}

}  // namespace omy_motion
