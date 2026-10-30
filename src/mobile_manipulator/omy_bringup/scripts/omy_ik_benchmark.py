#!/usr/bin/env python3
"""
7주차 - MoveIt /compute_ik 성공률·시간 측정 (6주차 실험 A 와 같은 목표 300 개)
  목표 생성은 6주차 ik_study_experiments.py 의 experiment_a 와 같은 난수 순서
    rng = default_rng(1), 관절 범위(±π 로 자름) 안 무작위 관절각 300 개 → FK → 목표 자세
    무작위 시작 = 같은 범위의 무작위 관절각 300 개, 가까운 시작 = 목표 관절각 + N(0, 0.3 rad)
  지금 떠 있는 move_group 의 IK 솔버로 측정 (솔버 이름은 move_group 파라미터에서 읽음)
  솔버를 바꾸려면 move_group 을 다시 띄움
    ros2 launch omy_moveit_config demo_mock.launch.py kinematics:=kdl|lma|trac_ik start_rviz:=false
  충돌은 보지 않음 (avoid_collisions false, 실험 A 와 같이 순수 IK 만 비교)
  성공 판정은 MoveIt 의 응답 + 받은 관절각을 우리 FK 로 다시 계산해 위치 1 mm, 자세 0.01 rad 이내인지

  ros2 run omy_bringup omy_ik_benchmark.py                    # 지금 솔버 측정 + 우리 ik_with_restarts
  ros2 run omy_bringup omy_ik_benchmark.py --ros-args -p timeout:=0.005 -p ours:=false
결과 - ~/robot_ws/ik_benchmark/ (-p output_dir:=...)
  <솔버>_<timeout>.csv   목표마다 성공, 시간, 위치/자세 오차
  summary.csv            방식 · 시작 · timeout 별 성공률, 시간 (같은 줄은 덮어씀) → 실행할 때마다 전체 표 출력
"""
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omy_ik_demo import ARM_JOINTS  # noqa: E402
from omy_ik_demo import ik_with_restarts  # noqa: E402
from omy_ik_demo import Kinematics  # noqa: E402
from omy_ik_demo import rotation_error  # noqa: E402
from omy_moveit_compare import to_pose  # noqa: E402

TIP = 'end_effector_link'
POS_TOL = 1e-3      # m
ROT_TOL = 1e-2      # rad
SOLVER_NAMES = {
    'kdl_kinematics_plugin/KDLKinematicsPlugin': 'KDL',
    'lma_kinematics_plugin/LMAKinematicsPlugin': 'LMA',
    'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin': 'TRAC-IK',
}
SUMMARY_FIELDS = ['method', 'start', 'timeout_s', 'n', 'success_pct', 'median_ms', 'p90_ms', 'max_ms',
                  'max_pos_err_mm', 'max_rot_err_deg', 'moveit_ok_but_wrong']


def experiment_a_targets(kin, n=300, seed=1):
    """6주차 experiment_a 와 같은 난수 순서로 목표·시작 자세 생성"""
    rng = np.random.default_rng(seed)
    lo, hi = np.maximum(kin.lower, -math.pi), np.minimum(kin.upper, math.pi)
    goals = [rng.uniform(lo, hi) for _ in range(n)]
    targets = [kin.fk(q)[0] for q in goals]
    seeds_far = [rng.uniform(lo, hi) for _ in range(n)]
    seeds_near = [g + rng.normal(0, 0.3, 6) for g in goals]
    return goals, targets, seeds_far, seeds_near


def pose_error(kin, q, target):
    t, _ = kin.fk(q)
    return (float(np.linalg.norm(target[:3, 3] - t[:3, 3])),
            float(np.linalg.norm(rotation_error(target[:3, :3], t[:3, :3]))))


def summarize(method, start, timeout, rows):
    ok = [r for r in rows if r['success']]
    ms = np.array([r['ms'] for r in rows])
    return dict(method=method, start=start, timeout_s=timeout, n=len(rows),
                success_pct=round(100.0 * len(ok) / len(rows), 1),
                median_ms=round(float(np.median(ms)), 2),
                p90_ms=round(float(np.percentile(ms, 90)), 2),
                max_ms=round(float(ms.max()), 2),
                max_pos_err_mm=round(max([r['pos_err'] for r in ok], default=0.0) * 1000, 4),
                max_rot_err_deg=round(math.degrees(max([r['rot_err'] for r in ok], default=0.0)), 4),
                moveit_ok_but_wrong=sum(1 for r in rows if r.get('moveit_ok') and not r['success']))


class IKBenchmark(Node):

    def __init__(self):
        super().__init__('omy_ik_benchmark')
        self.declare_parameter('n', 300)
        self.declare_parameter('seed', 1)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('ours', True)
        self.declare_parameter('output_dir', os.path.expanduser('~/robot_ws/ik_benchmark'))
        self.urdf_xml = None
        self.create_subscription(
            String, '/robot_description', lambda msg: setattr(self, 'urdf_xml', msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.param_client = self.create_client(GetParameters, '/move_group/get_parameters')

    @staticmethod
    def wait(future, timeout=30.0):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise TimeoutError('no response')
            time.sleep(0.0002)
        return future.result()

    def solver_name(self):
        self.param_client.wait_for_service(timeout_sec=10.0)
        req = GetParameters.Request(names=['robot_description_kinematics.arm.kinematics_solver'])
        value = self.wait(self.param_client.call_async(req)).values[0].string_value
        return SOLVER_NAMES.get(value, value or 'unknown')

    def compute_ik(self, target, seed, timeout):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.robot_state = RobotState(
            joint_state=JointState(name=list(ARM_JOINTS), position=[float(v) for v in seed]))
        req.ik_request.avoid_collisions = False
        req.ik_request.ik_link_name = TIP
        req.ik_request.pose_stamped = PoseStamped(pose=to_pose(target))
        req.ik_request.pose_stamped.header.frame_id = 'link0'
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int(round(timeout % 1 * 1e9)))
        t0 = time.perf_counter()
        res = self.wait(self.ik_client.call_async(req))
        ms = (time.perf_counter() - t0) * 1000
        if res.error_code.val != 1:
            return None, ms
        names = list(res.solution.joint_state.name)
        return np.array([res.solution.joint_state.position[names.index(j)] for j in ARM_JOINTS]), ms

    def run(self):
        while self.urdf_xml is None:
            time.sleep(0.05)
        kin = Kinematics(self.urdf_xml)
        n, seed = self.get_parameter('n').value, self.get_parameter('seed').value
        timeout = float(self.get_parameter('timeout').value)
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        _, targets, seeds_far, seeds_near = experiment_a_targets(kin, n, seed)
        # 가까운 시작은 관절 범위를 벗어날 수 있음 → 범위 안으로 (실험 A 의 ik_stats 도 clip)
        seeds_near = [np.clip(s, kin.lower, kin.upper) for s in seeds_near]

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/compute_ik not available (move_group running?)')
        solver = self.solver_name()
        self.get_logger().info(f'solver {solver}, timeout {timeout} s, {n} targets (seed {seed})')

        summaries, detail = [], []
        for start, seeds in (('무작위 시작', seeds_far), ('가까운 시작', seeds_near)):
            rows = []
            for i, (target, s) in enumerate(zip(targets, seeds)):
                q, ms = self.compute_ik(target, s, timeout)
                row = dict(i=i, start=start, ms=ms, moveit_ok=q is not None, success=False,
                           pos_err=math.nan, rot_err=math.nan)
                if q is not None:
                    row['pos_err'], row['rot_err'] = pose_error(kin, q, target)
                    row['success'] = row['pos_err'] < POS_TOL and row['rot_err'] < ROT_TOL
                rows.append(row)
            summaries.append(summarize(solver, start, timeout, rows))
            detail += rows
            self.get_logger().info(f'{solver} {start}: {summaries[-1]["success_pct"]} %, '
                                   f'median {summaries[-1]["median_ms"]} ms')

        name = f'{solver.lower().replace("-", "_")}_{timeout:g}'
        with open(os.path.join(out_dir, f'{name}.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['i', 'start', 'success', 'moveit_ok', 'ms', 'pos_err', 'rot_err'])
            w.writeheader()
            w.writerows(detail)

        if self.get_parameter('ours').value:
            rows = []
            for target, s in zip(targets, seeds_far):
                t0 = time.perf_counter()
                try:
                    q = ik_with_restarts(kin, target, s)
                    pos_err, rot_err = pose_error(kin, q, target)
                except ValueError:
                    q, pos_err, rot_err = None, math.nan, math.nan
                rows.append(dict(ms=(time.perf_counter() - t0) * 1000, pos_err=pos_err, rot_err=rot_err,
                                 success=q is not None and pos_err < POS_TOL and rot_err < ROT_TOL))
            summaries.append(summarize('ik_with_restarts (우리 코드)', '무작위 시작', '-', rows))

        table = update_summary(os.path.join(out_dir, 'summary.csv'), summaries)
        print_summary(table)
        self.get_logger().info(f'saved to {out_dir}')


def update_summary(path, new_rows):
    rows = []
    if os.path.exists(path):
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
    key = lambda r: (r['method'], r['start'], str(r['timeout_s']))  # noqa: E731
    new_keys = {key(r) for r in new_rows}
    rows = [r for r in rows if key(r) not in new_keys] + [{k: str(v) for k, v in r.items()} for r in new_rows]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        w.writeheader()
        w.writerows(rows)
    return rows


def print_summary(rows):
    print()
    print(f'{"방식":28s} {"시작":8s} {"timeout":>8s} {"성공 %":>7s} {"중앙 ms":>8s} {"90% ms":>8s} '
          f'{"최대 ms":>8s} {"위치오차 mm":>11s} {"자세오차°":>9s} {"잘못된 해":>8s}')
    for r in sorted(rows, key=lambda r: (r['method'], r['start'], r['timeout_s'])):
        print(f'{r["method"]:28s} {r["start"]:8s} {r["timeout_s"]:>8s} {r["success_pct"]:>7s} '
              f'{r["median_ms"]:>8s} {r["p90_ms"]:>8s} {r["max_ms"]:>8s} {r["max_pos_err_mm"]:>11s} '
              f'{r["max_rot_err_deg"]:>9s} {r["moveit_ok_but_wrong"]:>8s}')
    print()


def main():
    rclpy.init()
    node = IKBenchmark()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
