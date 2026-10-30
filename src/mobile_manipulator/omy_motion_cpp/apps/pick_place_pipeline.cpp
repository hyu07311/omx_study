// 3단계 - 픽앤플레이스 파이프라인 (이동 = 우리 RRT-Connect, 하강/상승 = 우리 Cartesian 경로)
//   READY → 큐브 위 → 하강 → 잡기 → 상승 → 놓을 곳 위 (yaw 90°) → 하강 → 놓기 → 상승 → READY
//   위치는 omy_ik_demo.py 와 같음 (큐브 (0.45, 0, 0.12), 놓는 곳 (0.35, -0.20), 위 높이 0.24 = CUBE_Z + 0.12)
//   장면 = 테이블 (table.world) + 빨간 큐브 + 장애물 벽
//     7주차 상자 (0.42, -0.10, 0.20) 는 큐브 위에서 손목 (link6) 과 겹쳐 하강 자체가 불가능 (7주차 기록과 같음)
//     → 집는 자세와 놓는 자세의 손목·손가락 사이 빈 공간 (y -0.149 ~ -0.104) 에 얇은 벽을 세움
//       0.12 x 0.02 x 0.20 m, 중심 (0.44, -0.127, 0.20): 두 자세와는 padding 포함 7 mm 이상 떨어지고
//       관절 직선 이동 (7주차 방식 A) 은 벽을 통과 → RRT-Connect 로 돌아가야 함
//   간선 검사 간격 = segment_fraction (기본 0.0005 x 68.07 = 0.034 rad, 2단계 결과에 따라 10 배 촘촘하게)
//     잡은 뒤 큐브는 그리퍼에 붙인 물체 (attached object) 로 충돌 판정에 포함
//     계획할 때만 상자를 padding 만큼 키움, 검증은 원래 크기 (7주차와 같은 규칙)
//   검증 = 시간 매개화까지 끝난 전체 궤적을 0.01 s 마다 원래 크기 scene 에서 다시 검사 → 충돌 점 수 0 확인
//
//   ros2 launch omy_motion_cpp benchmark.launch.py app:=pick_place_pipeline params:='runs:=20'
//     obstacle_center:=[x;y;z]  obstacle_size:=[sx;sy;sz]  segment_fraction:=0.0005
//     velocity_scaling:=0.3  cartesian_scaling:=0.1
//     execute:=true  → RViz mock (demo_mock.launch.py) 에서 /arm_controller, /gripper_controller 로 실행 (1 회)
// 결과 - ~/robot_ws/motion_cpp_results/pick_place/  segments.csv, summary.csv, trajectory.csv (첫 성공 실행)
#include "omy_motion_cpp/app_common.hpp"
#include "omy_motion_cpp/cartesian_path.hpp"
#include "omy_motion_cpp/collision_checker.hpp"
#include "omy_motion_cpp/rrt_connect.hpp"
#include "omy_motion_cpp/trajectory.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <thread>

using namespace omy_motion;

namespace
{

// omy_ik_demo.py 값
constexpr double kCubeX = 0.45, kCubeY = 0.0, kCubeZ = 0.12;
constexpr double kPlaceX = 0.35, kPlaceY = -0.20;
constexpr double kAboveZ = kCubeZ + 0.12;
constexpr double kGraspZ = kCubeZ - 0.005;
constexpr double kPlaceZ = kGraspZ + 0.005;
constexpr double kGripperOpen = 0.0;
constexpr double kGripperClose = 1.1;
const char * kCubeId = "red_cube";
const char * kTableId = "table";
const Eigen::Vector3d kWallSize(0.12, 0.02, 0.20);
const Eigen::Vector3d kWallCenter(0.44, -0.127, 0.20);
const std::vector<std::string> kArmJoints{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

enum class Kind { kRrt, kCartesian, kGripper };

struct Segment
{
  std::string name;
  Kind kind;
  bool success = false;
  double plan_ms = 0.0;
  double fraction = 1.0;          // Cartesian
  double max_dev_mm = 0.0;        // Cartesian
  std::vector<Vector6d> path;
  TimedTrajectory traj;
  double gripper = kGripperOpen;  // 이 구간 동안의 그리퍼 각도
  bool holding = false;           // 이 구간 동안 큐브를 들고 있는지
  size_t checked = 0, collisions = 0;
  // RRT 구간: 시작-목표를 관절 직선으로 바로 이었을 때 충돌 점 수 (장애물이 우회를 강제하는지 확인)
  size_t direct_checked = 0, direct_collisions = 0;
  std::string note;
};

const char * kindName(Kind k)
{
  return k == Kind::kRrt ? "rrt_connect" : k == Kind::kCartesian ? "cartesian" : "gripper";
}

void setupScene(CollisionChecker & c, const Eigen::Vector3d & box_size, const Eigen::Vector3d & box_center,
                double padding)
{
  // table.world / fake_d405_scene.py 와 같은 배치
  c.addBox(kTableId, Eigen::Vector3d(0.4, 0.6, 0.1), Eigen::Vector3d(0.45, 0.0, 0.05));
  c.addBox(kCubeId, Eigen::Vector3d(0.04, 0.04, 0.04), Eigen::Vector3d(kCubeX, kCubeY, kCubeZ));
  c.addBox(kObstacleId, box_size, box_center, padding);
  // 큐브를 잡으러 내려갈 때 열린 손가락이 큐브와 닿는 것은 허용 (집기 접근의 일반적인 처리)
  c.allowCollision(kCubeId, c.gripperLinks(), true);
  // 큐브는 테이블 위에 놓여 있음
  c.allowCollision(kCubeId, {kTableId}, true);
}

struct Pipeline
{
  const Kinematics & kin;
  CollisionChecker & checker;
  const std::vector<Vector6d> & seeds;
  JointLimits limits = omyJointLimits();
  double velocity_scaling, cartesian_scaling, planning_time;
  int rrt_retries;
  unsigned seed;
  rclcpp::Logger log;

  std::vector<Segment> segments;
  Vector6d q;

  bool move(const std::string & name, const Vector6d & goal, bool holding)
  {
    Segment s{name, Kind::kRrt};
    s.gripper = checker.gripper();
    s.holding = holding;
    const auto direct = densify({q, goal}, 0.01);
    s.direct_checked = direct.size();
    s.direct_collisions = countCollisions(checker, direct);
    Stopwatch sw;
    for (int attempt = 0; attempt < rrt_retries && !s.success; ++attempt) {
      RrtOptions opt;
      opt.timeout_s = planning_time;
      opt.seed = seed * 100 + static_cast<unsigned>(segments.size()) * 10 + attempt;
      RrtConnect rrt(checker, opt);
      const RrtResult r = rrt.plan(q, goal);
      s.success = r.success;
      s.path = r.path;
      if (!r.success) {
        s.note = "rrt failed (attempt " + std::to_string(attempt + 1) + ")";
      }
    }
    s.plan_ms = sw.ms();
    for (const auto & [label, qq] : {std::pair<const char *, Vector6d>{"start", q}, {"goal", goal}}) {
      if (!checker.isValid(qq)) {
        s.note += std::string(", ") + label + " in collision:";
        for (const auto & [l1, l2] : checker.contacts(qq)) {
          s.note += " " + l1 + "-" + l2;
        }
      }
    }
    if (s.success) {
      s.traj = timeParameterize(s.path, limits, velocity_scaling, true);
      q = goal;
    }
    segments.push_back(s);
    return s.success;
  }

  bool linear(const std::string & name, double x, double y, double z, double yaw, bool holding)
  {
    Segment s{name, Kind::kCartesian};
    s.gripper = checker.gripper();
    s.holding = holding;
    const CartesianResult r = cartesianPath(kin, checker, q, gripperDownPose(x, y, z, yaw));
    s.plan_ms = r.ms;
    s.fraction = r.fraction;
    s.max_dev_mm = r.max_dev_mm;
    s.path = r.path;
    s.success = r.fraction >= 1.0 - 1e-9;
    if (!s.success) {
      s.note = "fraction " + std::to_string(r.fraction) + " (" + r.failure + ")";
      if (r.failure == "collision") {
        for (const auto & [l1, l2] : checker.contacts(r.q_fail)) {
          s.note += " " + l1 + "-" + l2;
        }
      }
    } else {
      s.traj = timeParameterize(s.path, limits, cartesian_scaling, false);
      q = s.path.back();
    }
    segments.push_back(s);
    return s.success;
  }

  void gripper(const std::string & name, double angle, bool attach)
  {
    Segment s{name, Kind::kGripper};
    s.success = true;
    s.path = {q};
    s.traj = timeParameterize({q}, limits, 1.0, true);
    if (attach) {
      checker.attach(kCubeId, "rh_p12_rn_base", checker.gripperLinks(), q);
    } else {
      checker.detach(kCubeId, q);
    }
    checker.setGripper(angle);
    s.gripper = angle;
    s.holding = attach;
    segments.push_back(s);
  }

  std::optional<Vector6d> ik(double x, double y, double z, double yaw, const Vector6d & near) const
  {
    return kin.ikWithRestarts(gripperDownPose(x, y, z, yaw), near, seeds);
  }

  bool run(const Vector6d & start)
  {
    q = start;
    checker.setGripper(kGripperOpen);
    const auto above_pick = ik(kCubeX, kCubeY, kAboveZ, 0.0, readyPose());
    if (!above_pick) {
      RCLCPP_ERROR(log, "IK failed above pick");
      return false;
    }
    if (!move("move_to_pick", *above_pick, false) ||
        !linear("descend_pick", kCubeX, kCubeY, kGraspZ, 0.0, false)) {
      return false;
    }
    gripper("grasp", kGripperClose, true);
    if (!linear("ascend_pick", kCubeX, kCubeY, kAboveZ, 0.0, true)) {
      return false;
    }
    const auto above_place = ik(kPlaceX, kPlaceY, kAboveZ, M_PI / 2, q);
    if (!above_place) {
      RCLCPP_ERROR(log, "IK failed above place");
      return false;
    }
    if (!move("move_to_place", *above_place, true) ||
        !linear("descend_place", kPlaceX, kPlaceY, kPlaceZ, M_PI / 2, true)) {
      return false;
    }
    gripper("release", kGripperOpen, false);
    if (!linear("ascend_place", kPlaceX, kPlaceY, kAboveZ, M_PI / 2, false)) {
      return false;
    }
    return move("move_home", readyPose(), false);
  }
};

// 원래 크기 scene 에서 구간마다 같은 잡기/놓기 상태를 재현하며 궤적 점을 다시 검사
size_t validate(std::vector<Segment> & segments, CollisionChecker & exact)
{
  size_t total = 0;
  bool holding = false;
  for (auto & s : segments) {
    if (s.kind == Kind::kGripper) {
      if (s.holding && !holding) {
        exact.attach(kCubeId, "rh_p12_rn_base", exact.gripperLinks(), s.path.front());
      } else if (!s.holding && holding) {
        exact.detach(kCubeId, s.path.front());
      }
      holding = s.holding;
    }
    exact.setGripper(s.gripper);
    if (!s.success) {
      continue;
    }
    s.checked = s.traj.q.size();
    s.collisions = countCollisions(exact, s.traj.q);
    total += s.collisions;
  }
  return total;
}

// ---------- 실행 (RViz mock) ----------
class Executor
{
public:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GC = control_msgs::action::GripperCommand;

  explicit Executor(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    arm_ = rclcpp_action::create_client<FJT>(node, "/arm_controller/follow_joint_trajectory");
    grip_ = rclcpp_action::create_client<GC>(node, "/gripper_controller/gripper_cmd");
    sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          joints_[msg->name[i]] = msg->position[i];
        }
      });
    markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pick_place_path", rclcpp::QoS(1).transient_local());
  }

  std::optional<Vector6d> currentArm()
  {
    for (int k = 0; k < 100; ++k) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::all_of(kArmJoints.begin(), kArmJoints.end(), [&](const auto & j) { return joints_.count(j); })) {
          Vector6d q;
          for (int i = 0; i < kDof; ++i) {
            q[i] = joints_[kArmJoints[i]];
          }
          return q;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return std::nullopt;
  }

  bool arm(const TimedTrajectory & traj)
  {
    if (!arm_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "arm controller not available");
      return false;
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = kArmJoints;
    // 0.01 s 점을 0.05 s 간격으로 줄여 보냄 (끝점은 항상 포함)
    for (size_t i = 0; i < traj.t.size(); ++i) {
      if (i % 5 != 0 && i + 1 != traj.t.size()) {
        continue;
      }
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(traj.q[i].data(), traj.q[i].data() + kDof);
      p.velocities.assign(traj.qd[i].data(), traj.qd[i].data() + kDof);
      p.time_from_start = rclcpp::Duration::from_seconds(traj.t[i]);
      goal.trajectory.points.push_back(p);
    }
    auto handle = arm_->async_send_goal(goal).get();
    if (!handle) {
      return false;
    }
    const auto result = arm_->async_get_result(handle).get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void gripper(double angle)
  {
    if (!grip_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(node_->get_logger(), "gripper controller not available");
      return;
    }
    GC::Goal goal;
    goal.command.position = angle;
    goal.command.max_effort = 2.0;
    auto handle = grip_->async_send_goal(goal).get();
    if (handle) {
      // 물체를 쥐면 goal 이 끝나지 않을 수 있음 → 최대 3 s 만 기다림 (omy_ik_demo 와 같음)
      auto fut = grip_->async_get_result(handle);
      fut.wait_for(std::chrono::seconds(3));
    }
  }

  void publishPath(const Kinematics & kin, const std::vector<Segment> & segments, const Eigen::Vector3d & box_size,
                   const Eigen::Vector3d & box_center)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    for (const auto & s : segments) {
      if (s.kind == Kind::kGripper || !s.success) {
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.ns = s.name;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.003;
      m.pose.orientation.w = 1.0;
      m.color.a = 1.0;
      (s.kind == Kind::kRrt ? m.color.g : m.color.b) = 0.9;
      for (const auto & qq : s.traj.q) {
        const Eigen::Vector3d p = kin.fk(qq).translation();
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    }
    visualization_msgs::msg::Marker box;
    box.header.frame_id = "world";
    box.ns = "obstacle";
    box.id = id++;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.pose.position.x = box_center.x();
    box.pose.position.y = box_center.y();
    box.pose.position.z = box_center.z();
    box.pose.orientation.w = 1.0;
    box.scale.x = box_size.x();
    box.scale.y = box_size.y();
    box.scale.z = box_size.z();
    box.color.r = 1.0;
    box.color.g = 0.6;
    box.color.b = 0.1;
    box.color.a = 0.35;
    arr.markers.push_back(box);
    markers_->publish(arr);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FJT>::SharedPtr arm_;
  rclcpp_action::Client<GC>::SharedPtr grip_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  std::mutex mutex_;
  std::map<std::string, double> joints_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = makeNode("omy_pick_place_pipeline");
  const int runs = static_cast<int>(param(node, "runs", 20L));
  const bool execute = param(node, "execute", false);
  const double padding = param(node, "padding", kObstaclePadding);
  const Eigen::Vector3d box_size = paramVec3(node, "obstacle_size", kWallSize);
  const Eigen::Vector3d box_center = paramVec3(node, "obstacle_center", kWallCenter);
  const double segment_fraction = param(node, "segment_fraction", 0.0005);
  const std::string out_dir = param(node, "output_dir", resultsDir("pick_place"));
  std::filesystem::create_directories(out_dir);

  const Kinematics kin(node->get_parameter("robot_description").as_string());
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();
  const auto seeds = loadRestartSeeds();
  RCLCPP_INFO(node->get_logger(),
              "obstacle %.3f x %.3f x %.3f at (%.3f, %.3f, %.3f), planning padding %.3f m, segment fraction %g",
              box_size.x(), box_size.y(), box_size.z(), box_center.x(), box_center.y(), box_center.z(), padding,
              segment_fraction);

  std::shared_ptr<Executor> exec;
  std::thread spin;
  if (execute) {
    exec = std::make_shared<Executor>(node);
    spin = std::thread([node] { rclcpp::spin(node); });
  }

  std::ofstream seg_csv(out_dir + "/segments.csv");
  seg_csv << "run,segment,kind,success,plan_ms,fraction,waypoints,duration_s,max_dev_mm,checked,collisions,"
             "direct_checked,direct_collisions,note\n";
  std::vector<double> total_ms;
  int ok_runs = 0, clean_runs = 0;
  bool saved_traj = false;
  const int n_runs = execute ? 1 : runs;
  for (int run = 0; run < n_runs; ++run) {
    CollisionChecker plan_checker(model), exact(model);
    setupScene(plan_checker, box_size, box_center, padding);
    setupScene(exact, box_size, box_center, 0.0);
    plan_checker.setResolution(segment_fraction * plan_checker.maximumExtent());
    Pipeline p{kin, plan_checker, seeds, omyJointLimits(), param(node, "velocity_scaling", 0.3),
               param(node, "cartesian_scaling", 0.1), param(node, "planning_time", 5.0),
               static_cast<int>(param(node, "rrt_retries", 3L)), static_cast<unsigned>(run), node->get_logger(),
               {}, Vector6d::Zero()};

    Vector6d start = readyPose();
    if (execute) {
      const auto now = exec->currentArm();
      if (!now) {
        RCLCPP_ERROR(node->get_logger(), "no /joint_states - is demo_mock.launch.py running?");
        break;
      }
      // 지금 자세에서 READY 까지도 같은 RRT-Connect 로
      p.q = *now;
      plan_checker.setGripper(kGripperOpen);
      if (!p.move("move_to_ready", readyPose(), false)) {
        RCLCPP_ERROR(node->get_logger(), "could not plan to READY");
        break;
      }
    }
    Stopwatch sw;
    const bool ok = execute ? p.run(p.q) : p.run(start);
    const double ms = sw.ms();
    const size_t collisions = validate(p.segments, exact);
    total_ms.push_back(ms);
    ok_runs += ok;
    clean_runs += ok && collisions == 0;
    double duration = 0.0;
    for (const auto & s : p.segments) {
      duration += s.traj.duration();
      seg_csv << run << ',' << s.name << ',' << kindName(s.kind) << ',' << s.success << ',' << s.plan_ms << ','
              << s.fraction << ',' << s.path.size() << ',' << s.traj.duration() << ',' << s.max_dev_mm << ','
              << s.checked << ',' << s.collisions << ',' << s.direct_checked << ',' << s.direct_collisions << ','
              << s.note << '\n';
      if (s.kind == Kind::kRrt && s.success) {
        RCLCPP_INFO(node->get_logger(), "run %d: %-14s direct joint line %zu / %zu points in collision, RRT path %zu",
                    run, s.name.c_str(), s.direct_collisions, s.direct_checked, s.collisions);
      }
      if (!s.success) {
        RCLCPP_WARN(node->get_logger(), "run %d: %s failed - %s", run, s.name.c_str(), s.note.c_str());
      }
    }
    RCLCPP_INFO(node->get_logger(), "run %2d: %s, plan %.1f ms, motion %.1f s, collisions %zu", run,
                ok ? "success" : "FAILED", ms, duration, collisions);

    if (ok && !saved_traj) {
      saved_traj = true;
      std::ofstream f(out_dir + "/trajectory.csv");
      f << "t,segment,kind,collision_free,x,y,z,q0,q1,q2,q3,q4,q5\n" << std::setprecision(7);
      double t0 = 0.0;
      for (const auto & s : p.segments) {
        for (size_t i = 0; i < s.traj.t.size(); ++i) {
          const Eigen::Vector3d tip = kin.fk(s.traj.q[i]).translation();
          f << t0 + s.traj.t[i] << ',' << s.name << ',' << kindName(s.kind) << ',' << 1 << ',' << tip.x() << ','
            << tip.y() << ',' << tip.z();
          for (int j = 0; j < kDof; ++j) {
            f << ',' << s.traj.q[i][j];
          }
          f << '\n';
        }
        t0 += s.traj.duration();
      }
    }

    if (execute && ok) {
      if (collisions != 0) {
        RCLCPP_ERROR(node->get_logger(), "validation found %zu colliding points - not executing", collisions);
        break;
      }
      exec->publishPath(kin, p.segments, box_size, box_center);
      for (const auto & s : p.segments) {
        RCLCPP_INFO(node->get_logger(), "execute %s (%.2f s)", s.name.c_str(), s.traj.duration());
        if (s.kind == Kind::kGripper) {
          exec->gripper(s.gripper);
        } else if (!exec->arm(s.traj)) {
          RCLCPP_ERROR(node->get_logger(), "execution of %s failed", s.name.c_str());
          break;
        }
      }
    }
  }

  std::ofstream sf(out_dir + "/summary.csv");
  sf << "runs,success_pct,collision_free_pct,median_plan_ms,p90_plan_ms,max_plan_ms\n"
     << n_runs << ',' << 100.0 * ok_runs / n_runs << ',' << 100.0 * clean_runs / std::max(ok_runs, 1) << ','
     << median(total_ms) << ',' << percentile(total_ms, 90) << ','
     << (total_ms.empty() ? NAN : *std::max_element(total_ms.begin(), total_ms.end())) << '\n';
  std::printf("\n픽앤플레이스 %d 회: 성공 %d (%.0f %%), 충돌 0 인 실행 %d / %d, 전체 계획 시간 중앙 %.1f ms, 90%% %.1f ms\n",
              n_runs, ok_runs, 100.0 * ok_runs / n_runs, clean_runs, ok_runs, median(total_ms),
              percentile(total_ms, 90));
  RCLCPP_INFO(node->get_logger(), "saved to %s", out_dir.c_str());
  rclcpp::shutdown();
  if (spin.joinable()) {
    spin.join();
  }
  return 0;
}
