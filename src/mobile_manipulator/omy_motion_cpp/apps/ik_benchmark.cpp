// 1단계 - IK 성공률·시간: Python (numpy) vs C++ (Eigen) vs MoveIt KDL / LMA
//   목표 300 개 = export_ik_reference.py 가 내보낸 7주차 실험 A 목표 그대로 (targets.csv)
//   모두 같은 프로세스 안에서 함수 호출 시간만 잼 (ROS 서비스 왕복 없음)
//     C++ single    Kinematics::ik 한 번 (Python kin.ik 와 같은 알고리즘)
//     C++ restarts  ikWithRestarts (Python ik_with_restarts 와 같은 시드 순서, 같은 무작위 시드 30 개)
//     MoveIt        kinematics 플러그인 searchPositionIK (timeout, 같은 시작 자세)
//   성공 = 결과를 우리 FK 로 다시 계산해 위치 1 mm, 자세 0.01 rad 이내 (7주차와 같음)
//
//   ros2 launch omy_motion_cpp benchmark.launch.py app:=ik_benchmark
// 결과 - <data_dir>/cpp_results.csv (목표별), summary.csv (전체 표)
#include "omy_motion_cpp/csv.hpp"
#include "omy_motion_cpp/kinematics.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <sstream>

using namespace omy_motion;

namespace
{

constexpr double kPosTol = 1e-3;
constexpr double kRotTol = 1e-2;

struct Row
{
  int i;
  std::string start, method;
  bool success, solver_ok;
  double ms, pos_err, rot_err;
  Vector6d q;
};

struct Summary
{
  std::string method, start, timeout;
  int n;
  double success_pct, median_ms, p90_ms, max_ms, max_pos_err_mm, max_rot_err_deg;
  int solver_ok_but_wrong;
};

Summary summarize(const std::string & method, const std::string & start, const std::string & timeout,
                  const std::vector<Row> & rows)
{
  Summary s{method, start, timeout, static_cast<int>(rows.size()), 0, 0, 0, 0, 0, 0, 0};
  std::vector<double> ms;
  int ok = 0;
  for (const auto & r : rows) {
    ms.push_back(r.ms);
    if (r.success) {
      ++ok;
      s.max_pos_err_mm = std::max(s.max_pos_err_mm, r.pos_err * 1000);
      s.max_rot_err_deg = std::max(s.max_rot_err_deg, r.rot_err * 180 / M_PI);
    } else if (r.solver_ok) {
      ++s.solver_ok_but_wrong;
    }
  }
  s.success_pct = 100.0 * ok / rows.size();
  s.median_ms = median(ms);
  s.p90_ms = percentile(ms, 90);
  s.max_ms = *std::max_element(ms.begin(), ms.end());
  return s;
}

Vector6d readVec(const CsvTable & t, size_t row, const std::string & prefix)
{
  Vector6d v;
  for (int k = 0; k < kDof; ++k) {
    v[k] = t.num(row, prefix + std::to_string(k));
  }
  return v;
}

std::string startLabel(const std::string & s) { return s == "far" ? "무작위 시작" : "가까운 시작"; }

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "omy_ik_benchmark_cpp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto param = [&](const std::string & name, auto def) {
    using T = decltype(def);
    return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() : def;
  };
  const std::string data_dir = param("data_dir", std::string(std::getenv("HOME")) + "/robot_ws/motion_cpp_results/ik");
  const double timeout = param("timeout", 0.05);
  const std::string solvers = param("solvers", std::string("kdl,lma"));
  const int warmup = static_cast<int>(param("warmup", 10L));

  // ---------- 입력 ----------
  const std::string urdf = node->get_parameter("robot_description").as_string();
  const Kinematics kin(urdf);
  const CsvTable targets_csv = readCsv(data_dir + "/targets.csv");
  const CsvTable seeds_csv = readCsv(data_dir + "/restart_seeds.csv");
  std::vector<Eigen::Isometry3d> targets;
  std::map<std::string, std::vector<Vector6d>> starts;
  for (size_t r = 0; r < targets_csv.rows.size(); ++r) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        t.matrix()(i, j) = targets_csv.num(r, "T" + std::to_string(i) + std::to_string(j));
      }
    }
    targets.push_back(t);
    starts["far"].push_back(readVec(targets_csv, r, "far"));
    starts["near"].push_back(readVec(targets_csv, r, "near"));
  }
  std::vector<Vector6d> restart_seeds;
  for (size_t r = 0; r < seeds_csv.rows.size(); ++r) {
    restart_seeds.push_back(readVec(seeds_csv, r, "q"));
  }
  // FK 일치 확인 - targets.csv 의 목표 자세는 Python FK(g) 결과
  double fk_diff = 0.0;
  for (size_t r = 0; r < targets.size(); ++r) {
    fk_diff = std::max(fk_diff, (kin.fk(readVec(targets_csv, r, "g")).matrix() - targets[r].matrix()).cwiseAbs().maxCoeff());
  }
  RCLCPP_INFO(node->get_logger(), "%zu targets, %zu restart seeds, max |FK_cpp - FK_python| = %.2e",
              targets.size(), restart_seeds.size(), fk_diff);

  auto evaluate = [&](int i, const std::string & start, const std::string & method, bool solver_ok, double ms,
                      const Vector6d & q) {
    Row row{i, start, method, false, solver_ok, ms, NAN, NAN, q};
    if (solver_ok) {
      std::tie(row.pos_err, row.rot_err) = kin.poseError(q, targets[i]);
      row.success = row.pos_err < kPosTol && row.rot_err < kRotTol;
    }
    return row;
  };

  std::vector<Row> rows;
  std::vector<Summary> summaries;
  auto run = [&](const std::string & method, const std::string & timeout_label,
                 const std::function<bool(const Eigen::Isometry3d &, const Vector6d &, Vector6d &)> & solve) {
    for (const std::string start : {"far", "near"}) {
      Vector6d q;
      for (int w = 0; w < warmup; ++w) {
        solve(targets[w % targets.size()], starts[start][w % targets.size()], q);
      }
      std::vector<Row> part;
      for (size_t i = 0; i < targets.size(); ++i) {
        Stopwatch sw;
        const bool ok = solve(targets[i], starts[start][i], q);
        const double ms = sw.ms();
        part.push_back(evaluate(static_cast<int>(i), start, method, ok, ms, q));
      }
      summaries.push_back(summarize(method, startLabel(start), timeout_label, part));
      RCLCPP_INFO(node->get_logger(), "%-18s %s: %5.1f %%, median %.4f ms", method.c_str(),
                  startLabel(start).c_str(), summaries.back().success_pct, summaries.back().median_ms);
      rows.insert(rows.end(), part.begin(), part.end());
    }
  };

  // ---------- C++ (우리 코드) ----------
  run("C++ ik (single)", "-", [&](const Eigen::Isometry3d & t, const Vector6d & s, Vector6d & q) {
    const IkResult r = kin.ik(t, s);
    q = r.q;
    return r.success;
  });
  run("C++ ikWithRestarts", "-", [&](const Eigen::Isometry3d & t, const Vector6d & s, Vector6d & q) {
    const auto r = kin.ikWithRestarts(t, s, restart_seeds);
    if (r) {
      q = *r;
    }
    return r.has_value();
  });

  // ---------- MoveIt 플러그인 ----------
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();
  pluginlib::ClassLoader<kinematics::KinematicsBase> plugin_loader("moveit_core", "kinematics::KinematicsBase");
  const std::map<std::string, std::pair<std::string, std::string>> plugins{
    {"kdl", {"KDL", "kdl_kinematics_plugin/KDLKinematicsPlugin"}},
    {"lma", {"LMA", "lma_kinematics_plugin/LMAKinematicsPlugin"}},
  };
  std::stringstream ss(solvers);
  std::string key;
  std::ostringstream tlabel;
  tlabel << timeout;
  while (std::getline(ss, key, ',')) {
    const auto it = plugins.find(key);
    if (it == plugins.end()) {
      RCLCPP_WARN(node->get_logger(), "unknown solver %s", key.c_str());
      continue;
    }
    auto solver = plugin_loader.createSharedInstance(it->second.second);
    if (!solver->initialize(node, *model, "arm", "link0", {"end_effector_link"}, 0.005)) {
      RCLCPP_ERROR(node->get_logger(), "failed to initialize %s", it->second.second.c_str());
      continue;
    }
    run("MoveIt " + it->second.first, tlabel.str(),
        [&](const Eigen::Isometry3d & t, const Vector6d & s, Vector6d & q) {
          std::vector<double> seed(s.data(), s.data() + kDof), sol;
          moveit_msgs::msg::MoveItErrorCodes err;
          const bool ok = solver->searchPositionIK(tf2::toMsg(t), seed, timeout, sol, err);
          if (ok) {
            q = Eigen::Map<const Vector6d>(sol.data());
          }
          return ok;
        });
  }

  // ---------- Python 결과 (export_ik_reference.py) 와 목표별 비교 ----------
  std::ifstream py_file(data_dir + "/python_results.csv");
  if (py_file) {
    py_file.close();
    const CsvTable py = readCsv(data_dir + "/python_results.csv");
    std::map<std::tuple<std::string, std::string, int>, bool> py_ok;
    std::map<std::pair<std::string, std::string>, std::vector<Row>> py_rows;
    for (size_t r = 0; r < py.rows.size(); ++r) {
      const std::string start = py.str(r, "start"), method = py.str(r, "method");
      const int i = std::stoi(py.str(r, "i"));
      Row row{i, start, method, py.str(r, "success") == "1", py.str(r, "success") == "1", py.num(r, "ms"),
              py.num(r, "pos_err"), py.num(r, "rot_err"), Vector6d::Zero()};
      py_ok[{start, method, i}] = row.success;
      py_rows[{method, start}].push_back(row);
    }
    for (const auto & [key2, part] : py_rows) {
      const std::string label = key2.first == "single" ? "Python ik (single)" : "Python ik_with_restarts";
      summaries.push_back(summarize(label, startLabel(key2.second), "-", part));
    }
    for (const auto & [cpp_method, py_method] : std::vector<std::pair<std::string, std::string>>{
           {"C++ ik (single)", "single"}, {"C++ ikWithRestarts", "restarts"}}) {
      int same = 0, total = 0;
      for (const auto & r : rows) {
        const auto it = py_ok.find({r.start, py_method, r.i});
        if (r.method == cpp_method && it != py_ok.end()) {
          ++total;
          same += it->second == r.success;
        }
      }
      RCLCPP_INFO(node->get_logger(), "%s vs Python %s: per-target success agreement %d / %d",
                  cpp_method.c_str(), py_method.c_str(), same, total);
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "no python_results.csv - run export_ik_reference.py without --skip-python");
  }

  // ---------- 저장 ----------
  {
    std::ofstream f(data_dir + "/cpp_results.csv");
    f << "i,start,method,success,solver_ok,ms,pos_err,rot_err,q0,q1,q2,q3,q4,q5\n" << std::setprecision(10);
    for (const auto & r : rows) {
      f << r.i << ',' << r.start << ',' << r.method << ',' << r.success << ',' << r.solver_ok << ',' << r.ms << ','
        << r.pos_err << ',' << r.rot_err;
      for (int k = 0; k < kDof; ++k) {
        f << ',' << r.q[k];
      }
      f << '\n';
    }
  }
  {
    std::ofstream f(data_dir + "/summary.csv");
    f << "method,start,timeout_s,n,success_pct,median_ms,p90_ms,max_ms,max_pos_err_mm,max_rot_err_deg,"
         "solver_ok_but_wrong\n";
    for (const auto & s : summaries) {
      f << s.method << ',' << s.start << ',' << s.timeout << ',' << s.n << ',' << s.success_pct << ','
        << s.median_ms << ',' << s.p90_ms << ',' << s.max_ms << ',' << s.max_pos_err_mm << ','
        << s.max_rot_err_deg << ',' << s.solver_ok_but_wrong << '\n';
    }
  }
  std::printf("\n%-26s %-10s %8s %7s %10s %10s %10s %10s %9s %6s\n", "방식", "시작", "timeout", "성공%",
              "중앙ms", "90%ms", "최대ms", "위치mm", "자세°", "틀린해");
  for (const auto & s : summaries) {
    std::printf("%-26s %-10s %8s %7.1f %10.4f %10.4f %10.3f %10.4f %9.4f %6d\n", s.method.c_str(), s.start.c_str(),
                s.timeout.c_str(), s.success_pct, s.median_ms, s.p90_ms, s.max_ms, s.max_pos_err_mm,
                s.max_rot_err_deg, s.solver_ok_but_wrong);
  }
  RCLCPP_INFO(node->get_logger(), "saved to %s", data_dir.c_str());
  rclcpp::shutdown();
  return 0;
}
