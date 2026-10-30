// 2단계 - 관절공간 RRT-Connect: 우리 C++ 구현 vs MoveIt OMPL RRTConnect
//   장면 = 7주차 장애물 장면 (상자 0.10 x 0.10 x 0.30 m, 중심 (0.42, -0.10, 0.20)) + 비교용 빈 장면
//   동작 = 7주차 transfer: (0.45, 0, 0.24) yaw 0 → (0.35, -0.20, 0.24) yaw 90°
//     시작 = ikWithRestarts(시작 목표, READY_POSE), 목표 = ikWithRestarts(끝 목표, 시작)  (7주차와 같은 방식)
//   두 플래너 모두 같은 PlanningScene 객체 (계획용 = 상자 + padding 5 mm) 로 충돌 판정
//   MoveIt 은 시간 매개화 등 request adapter 없이 planner 만 (num_planning_attempts 1, 계획 시간 5 s)
//   간선 검사 간격 = longest_valid_segment_fraction x 상태공간 크기 (68.07 rad) - 두 플래너에 같은 값
//     0.005 (omy_moveit_config 설정 = 0.34 rad), 0.0005 (10 배 촘촘) 두 가지로 비교
//   검증 = padding 없는 원래 상자 scene 에서 경로를 관절 0.005 rad 간격으로 다시 검사한 충돌 점 수
//
//   ros2 launch omy_motion_cpp benchmark.launch.py app:=rrt_benchmark params:='trials:=100'
//     scenes:=[obstacle;free]  segment_fractions:=[0.005;0.0005]  planning_time:=5.0  padding:=0.005
// 결과 - ~/robot_ws/motion_cpp_results/rrt/  trials.csv, summary.csv, path_<장면>_<fraction>_<플래너>.csv
#include "omy_motion_cpp/app_common.hpp"
#include "omy_motion_cpp/collision_checker.hpp"
#include "omy_motion_cpp/rrt_connect.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>

using namespace omy_motion;

namespace
{

constexpr double kCheckStep = 0.005;   // 검증 간격 (rad)

struct Trial
{
  std::string scene, planner;
  double fraction;   // longest_valid_segment_fraction
  int trial;
  bool success;
  double plan_ms, simplify_ms, total_ms;
  size_t nodes;
  std::vector<Vector6d> path;
  size_t waypoints = 0;
  double length = 0.0;
  size_t checked = 0, collisions = 0;
  std::string contacts;   // 검증에서 부딪힌 링크 쌍
};

std::string fractionLabel(double f)
{
  std::ostringstream ss;
  ss << f;
  return ss.str();
}

void writePath(const std::string & file, const Kinematics & kin, const std::vector<Vector6d> & path)
{
  std::ofstream f(file);
  f << "k,x,y,z,q0,q1,q2,q3,q4,q5\n" << std::setprecision(8);
  const auto dense = densify(path, 0.02);
  for (size_t k = 0; k < dense.size(); ++k) {
    const Eigen::Vector3d p = kin.fk(dense[k]).translation();
    f << k << ',' << p.x() << ',' << p.y() << ',' << p.z();
    for (int j = 0; j < kDof; ++j) {
      f << ',' << dense[k][j];
    }
    f << '\n';
  }
}

planning_interface::MotionPlanRequest moveitRequest(const moveit::core::RobotModelConstPtr & model,
                                                    const Vector6d & q0, const Vector6d & q1, double planning_time)
{
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setJointGroupPositions("arm", q0.data());
  start.update();
  moveit::core::RobotState goal(start);
  goal.setJointGroupPositions("arm", q1.data());
  goal.update();
  planning_interface::MotionPlanRequest req;
  req.group_name = "arm";
  req.pipeline_id = "ompl";
  req.planner_id = "RRTConnect";
  req.num_planning_attempts = 1;
  req.allowed_planning_time = planning_time;
  // 작업 공간 (관절공간 계획이라 결과에는 영향 없음, 경고만 없앰)
  req.workspace_parameters.header.frame_id = model->getModelFrame();
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -1.5;
  req.workspace_parameters.min_corner.z = -0.5;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 1.5;
  req.workspace_parameters.max_corner.z = 1.5;
  moveit::core::robotStateToRobotStateMsg(start, req.start_state);
  req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(goal, model->getJointModelGroup("arm"), 1e-4));
  return req;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = makeNode("omy_rrt_benchmark");
  const int trials = static_cast<int>(param(node, "trials", 100L));
  const double planning_time = param(node, "planning_time", 5.0);
  const double padding = param(node, "padding", kObstaclePadding);
  const int shortcut_iterations = static_cast<int>(param(node, "shortcut_iterations", 200L));
  const auto scenes = param(node, "scenes", std::vector<std::string>{"obstacle", "free"});
  const auto fractions = param(node, "segment_fractions", std::vector<double>{0.005, 0.0005});
  const Eigen::Vector3d box_size = paramVec3(node, "obstacle_size", kObstacleSize);
  const Eigen::Vector3d box_center = paramVec3(node, "obstacle_center", kObstacleCenter);
  const std::string out_dir = param(node, "output_dir", resultsDir("rrt"));
  std::filesystem::create_directories(out_dir);

  const Kinematics kin(node->get_parameter("robot_description").as_string());
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();

  // 시작/목표 관절각 (7주차 run_all 과 같은 방식)
  const auto seeds = loadRestartSeeds();
  const auto q0 = kin.ikWithRestarts(gripperDownPose(0.45, 0.0, 0.24, 0.0), readyPose(), seeds);
  const auto q1 = q0 ? kin.ikWithRestarts(gripperDownPose(0.35, -0.20, 0.24, M_PI / 2), *q0, seeds) : std::nullopt;
  if (!q0 || !q1) {
    throw std::runtime_error("IK failed for transfer start/goal");
  }
  RCLCPP_INFO(node->get_logger(), "start [%s]", (std::ostringstream() << q0->transpose()).str().c_str());
  RCLCPP_INFO(node->get_logger(), "goal  [%s]", (std::ostringstream() << q1->transpose()).str().c_str());
  const auto req = moveitRequest(model, *q0, *q1, planning_time);

  std::vector<Trial> all;
  // 한 (간선 간격, 장면) 조합에서 두 플래너를 trials 번씩
  auto run_case = [&](double fraction, const std::string & scene_name,
                      const planning_pipeline::PlanningPipeline & pipeline) {
    CollisionChecker plan_checker(model), check_checker(model);
    if (scene_name == "obstacle") {
      plan_checker.addBox(kObstacleId, box_size, box_center, padding);
      check_checker.addBox(kObstacleId, box_size, box_center, 0.0);
    }
    plan_checker.setResolution(fraction * plan_checker.maximumExtent());
    RCLCPP_INFO(node->get_logger(), "[%s, fraction %g] start valid %d, goal valid %d, edge resolution %.4f rad",
                scene_name.c_str(), fraction, plan_checker.isValid(*q0), plan_checker.isValid(*q1),
                plan_checker.resolution());

    auto finish = [&](Trial & t) {
      if (t.success) {
        const auto dense = densify(t.path, kCheckStep);
        t.checked = dense.size();
        std::set<std::pair<std::string, std::string>> pairs;
        for (const auto & q : dense) {
          if (!check_checker.isValid(q)) {
            ++t.collisions;
            const auto c = check_checker.contacts(q);
            pairs.insert(c.begin(), c.end());
          }
        }
        for (const auto & [a, b] : pairs) {
          t.contacts += (t.contacts.empty() ? "" : " ") + a + "-" + b;
        }
        t.length = pathLength(t.path);
        t.waypoints = t.path.size();
      }
      all.push_back(t);
    };

    for (int k = 0; k < trials; ++k) {
      RrtOptions opt;
      opt.timeout_s = planning_time;
      opt.seed = static_cast<unsigned>(k);
      opt.shortcut_iterations = shortcut_iterations;
      RrtConnect rrt(plan_checker, opt);
      const RrtResult r = rrt.plan(*q0, *q1);
      Trial t{scene_name, "ours", fraction, k, r.success, r.plan_ms, r.simplify_ms, r.plan_ms + r.simplify_ms,
              r.nodes, r.path};
      finish(t);
    }
    for (int k = 0; k < trials; ++k) {
      planning_interface::MotionPlanResponse res;
      Stopwatch sw;
      const bool ok = pipeline.generatePlan(plan_checker.scene(), req, res) &&
                      res.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      const double ms = sw.ms();
      Trial t{scene_name, "moveit", fraction, k, ok, ms, 0.0, ms, 0, {}};
      if (ok) {
        for (size_t w = 0; w < res.trajectory_->getWayPointCount(); ++w) {
          Vector6d q;
          res.trajectory_->getWayPoint(w).copyJointGroupPositions("arm", q.data());
          t.path.push_back(q);
        }
      }
      finish(t);
    }
  };

  for (const double fraction : fractions) {
    // OMPL 은 간선 간격을 플러그인 초기화 때 읽음 → 파라미터를 바꾸고 pipeline 을 새로 만듦
    node->set_parameter(rclcpp::Parameter("ompl.arm.longest_valid_segment_fraction", fraction));
    planning_pipeline::PlanningPipeline pipeline(model, node, "ompl", "ompl_interface/OMPLPlanner",
                                                 std::vector<std::string>{});
    pipeline.displayComputedMotionPlans(false);
    pipeline.publishReceivedRequests(false);
    for (const auto & scene_name : scenes) {
      run_case(fraction, scene_name, pipeline);
    }
  }

  // ---------- 저장 / 요약 ----------
  {
    std::ofstream f(out_dir + "/trials.csv");
    f << "scene,fraction,planner,trial,success,plan_ms,simplify_ms,total_ms,nodes,waypoints,length_rad,checked,"
         "collisions,contacts\n";
    for (const auto & t : all) {
      f << t.scene << ',' << t.fraction << ',' << t.planner << ',' << t.trial << ',' << t.success << ','
        << t.plan_ms << ',' << t.simplify_ms << ',' << t.total_ms << ',' << t.nodes << ',' << t.waypoints << ','
        << t.length << ',' << t.checked << ',' << t.collisions << ',' << t.contacts << '\n';
    }
  }
  const double extent = CollisionChecker(model).maximumExtent();
  std::ofstream sf(out_dir + "/summary.csv");
  sf << "scene,fraction,resolution_rad,planner,n,success_pct,median_ms,p90_ms,max_ms,median_plan_ms,"
        "median_simplify_ms,median_length_rad,collision_free_pct,max_collisions\n";
  std::printf("\n%-9s %-8s %-7s %4s %6s %9s %9s %9s %10s %10s %8s %7s\n", "장면", "fraction", "플래너", "n",
              "성공%", "중앙ms", "90%ms", "최대ms", "중앙단순화", "중앙길이", "충돌0%", "최대충돌");
  for (const double fraction : fractions) {
    for (const auto & scene_name : scenes) {
      for (const std::string planner : {"ours", "moveit"}) {
        std::vector<double> total, plan, simplify, len;
        int ok = 0, n = 0, clean = 0;
        size_t max_col = 0;
        std::vector<const Trial *> mine;
        for (const auto & t : all) {
          if (t.scene == scene_name && t.planner == planner && t.fraction == fraction) {
            mine.push_back(&t);
          }
        }
        for (const auto * t : mine) {
          ++n;
          total.push_back(t->total_ms);
          if (t->success) {
            ++ok;
            plan.push_back(t->plan_ms);
            simplify.push_back(t->simplify_ms);
            len.push_back(t->length);
            clean += t->collisions == 0;
            max_col = std::max(max_col, t->collisions);
          }
        }
        if (n == 0) {
          continue;
        }
        // 대표 경로 = 길이가 중앙값에 가장 가까운 시도
        const double med_len = median(len);
        const Trial * rep = nullptr;
        for (const auto * t : mine) {
          if (t->success && (!rep || std::abs(t->length - med_len) < std::abs(rep->length - med_len))) {
            rep = t;
          }
        }
        if (rep) {
          writePath(out_dir + "/path_" + scene_name + "_" + fractionLabel(fraction) + "_" + planner + ".csv", kin,
                    rep->path);
        }
        const double pct = 100.0 * ok / n;
        const double clean_pct = 100.0 * clean / std::max(ok, 1);
        const double max_ms = *std::max_element(total.begin(), total.end());
        sf << scene_name << ',' << fraction << ',' << fraction * extent << ',' << planner << ',' << n << ',' << pct
           << ',' << median(total) << ',' << percentile(total, 90) << ',' << max_ms << ',' << median(plan) << ','
           << median(simplify) << ',' << med_len << ',' << clean_pct << ',' << max_col << '\n';
        std::printf("%-9s %-8g %-7s %4d %6.1f %9.3f %9.3f %9.3f %10.3f %10.3f %8.1f %7zu\n", scene_name.c_str(),
                    fraction, planner.c_str(), n, pct, median(total), percentile(total, 90), max_ms,
                    median(simplify), med_len, clean_pct, max_col);
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "saved to %s", out_dir.c_str());
  rclcpp::shutdown();
  return 0;
}
