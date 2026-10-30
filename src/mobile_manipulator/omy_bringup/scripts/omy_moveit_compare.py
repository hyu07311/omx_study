#!/usr/bin/env python3
"""
7주차 - 같은 손끝 동작을 세 방식으로 실행해 손끝 경로 비교 (RViz mock + move_group)
  A  우리 IK           ik_with_restarts → 목표 관절각 1점 → JTC (끝점 속도·가속도 0 → 관절 5차 보간)
  B  MoveIt plan       /compute_ik 로 목표 관절각 → /plan_kinematic_path (OMPL RRTConnect + TOTG 시간 계획)
  C  MoveIt Cartesian  /compute_cartesian_path (손끝 직선을 5 mm 간격 점으로 나눠 점마다 IK)
B, C 는 MoveIt 이 정한 시간 계획의 모양은 그대로 두고 전체 시간만 A 와 같게 늘림 (같은 시간으로 비교)
실행 중 손끝을 100 Hz 로 기록 → CSV, 직선 이탈 그래프, RViz 마커 (/compare_paths)
  손끝 위치 = /joint_states (100 Hz) + URDF FK. TF world → end_effector_link 도 같이 기록하지만
  robot_state_publisher 가 TF 를 20 Hz 로만 내보내서 (publish_frequency) 경로는 FK 값으로 그림.
  도착 오차는 TF 로 계산 (FK 와 독립된 확인)
실행한 경로는 기록한 관절각을 /check_state_validity 로 다시 검사해 충돌 점 수를 같이 보고

동작 (5주차 데모, 6주차 numpy 실험 B 와 같은 값)
  하강    (0.45, 0, 0.24) → (0.45, 0, 0.115)                1.5 s
  옮기기  (0.45, 0, 0.24) → (0.35, -0.20, 0.24), yaw 0 → 90°  3 s
시작 자세도 실험 B 와 같게 ik_with_restarts(시작 목표, READY_POSE)

장애물 (-p obstacle:=true) - /apply_planning_scene 으로 상자를 넣고 다시 비교
  상자 기본값 0.10 x 0.10 x 0.30 m, 중심 (0.42, -0.10, 0.20) (-p obstacle_size / obstacle_center 로 변경)
  A  실행하지 않음. 관절 5차 보간 경로를 0.01 s 마다 /check_state_validity 로 검사해 충돌 구간만 표시
  B  상자를 피하는 경로를 계획해 실행 (RRTConnect 는 무작위라 실행할 때마다 경로가 다름)
     num_planning_attempts (기본 5) 번 병렬로 풀어 MoveIt 이 가장 짧은 경로를 고름 (1 이면 크게 돌아가는 경우가 많음)
  C  직선이 상자를 지나면 fraction 이 떨어짐 → 0.9 미만이면 실행하지 않고 풀린 부분까지만 표시
  시작 자세로 되돌아갈 때도 MoveIt 으로 계획 (관절 직선 이동은 상자를 통과할 수 있음)
  obstacle:=false 로 실행하면 상자를 지움
  계획할 때만 상자를 obstacle_padding (기본 0.005 m) 만큼 키움. 충돌 검사는 원래 크기로.
    이유 - MoveIt 은 TOTG 시간 계획 뒤의 궤적 점만 충돌 검사하는데, 점 사이 (0.1 s 간격) 구간에서
    모서리가 깎여 상자를 0.6~3.5 mm 파고드는 경우가 있었음 (padding 0, 6 번 중 5 번)
    -p obstacle_padding:=0.0 이면 그 현상을 그대로 볼 수 있음
    1 cm 이상이면 옮기기 목표 자세 (손가락 ~ 상자 면 약 1 cm) 자체가 막혀 IK 가 실패함
  하강은 목표 자세에서 손목 (link4~6, 카메라) 이 상자와 겹쳐서 B, C 모두 풀 수 없음 → motions:=transfer 권장

  ros2 launch omy_moveit_config demo_mock.launch.py
  ros2 run omy_bringup omy_moveit_compare.py
  ros2 run omy_bringup omy_moveit_compare.py --ros-args -p obstacle:=true -p motions:=transfer
결과 - ~/robot_ws/moveit_compare/<날짜_시각>[_obstacle]/ (-p output_dir:=... 로 변경)
  traj_<동작>_<방식>.csv   t, 손끝 x y z (FK), tf_x tf_y tf_z (TF, 20 Hz 갱신), 직선 이탈 mm, 충돌, joint1~6
  metrics.csv              방식별 성공, 계획 시간, 실행 시간, 직선 이탈 최대, 도착 오차, 관절 이동량, 충돌 점 수
  deviation_<동작>.png     시간 vs 직선 이탈 (점선 = 6주차 numpy 로 예측한 A, 빨간 영역 = A 충돌 구간)
"""
import csv
import datetime
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
import tf2_ros

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from omy_ik_demo import ARM_JOINTS  # noqa: E402
from omy_ik_demo import BASE_LINK  # noqa: E402
from omy_ik_demo import gripper_down_pose  # noqa: E402
from omy_ik_demo import ik_with_restarts  # noqa: E402
from omy_ik_demo import Kinematics  # noqa: E402
from omy_ik_demo import READY_POSE  # noqa: E402
from omy_ik_demo import rotation_error  # noqa: E402

WORLD = 'world'
TIP = 'end_effector_link'
GROUP = 'arm'
OBSTACLE_ID = 'compare_obstacle'

MOTIONS = {
    'descend': dict(label='하강', p0=(0.45, 0.0, 0.24), p1=(0.45, 0.0, 0.115),
                    yaw0=0.0, yaw1=0.0, T=1.5),
    'transfer': dict(label='옮기기 (yaw 90°)', p0=(0.45, 0.0, 0.24), p1=(0.35, -0.20, 0.24),
                     yaw0=0.0, yaw1=math.pi / 2, T=3.0),
}
METHODS = {
    'A': dict(label='A 우리 IK (관절 5차)', color=(0.90, 0.15, 0.15)),
    'B': dict(label='B MoveIt plan (RRTConnect)', color=(0.10, 0.75, 0.20)),
    'C': dict(label='C MoveIt Cartesian', color=(0.20, 0.45, 1.00)),
}
CARTESIAN_STEP = 0.005      # m
CARTESIAN_MIN_FRACTION = 0.9
RECORD_HZ = 100.0
CHECK_DT = 0.01             # A 충돌 검사 간격 (s)
CHECK_EVERY = 3             # 실행한 경로는 기록 3 개마다 1 번 검사 (약 33 Hz)


class PlanFailed(RuntimeError):
    """계획 실패. partial 에 풀린 부분까지의 관절 경로 (C 의 fraction < 0.9 등)"""

    def __init__(self, message, partial=None):
        super().__init__(message)
        self.partial = partial


# ===================== 계산 유틸 =====================
def s_quintic(tau):
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def line_deviation(points, p0, p1):
    """각 점에서 직선 p0-p1 까지 거리 (6주차 실험 B 와 같은 식)"""
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    rel = points - p0
    return np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)


def matrix_to_quaternion(r):
    """회전행렬 → (x, y, z, w)"""
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return (q[0], q[1], q[2], (r[k, j] - r[j, k]) / s)


def quaternion_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_pose(t):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in t[:3, 3])
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = (float(v) for v in matrix_to_quaternion(t[:3, :3]))
    return pose


def duration(seconds):
    return Duration(sec=int(seconds), nanosec=int(round((seconds - int(seconds)) * 1e9)))


def seconds(d):
    return d.sec + d.nanosec * 1e-9


def arm_state(q):
    state = RobotState()
    state.joint_state = JointState(name=list(ARM_JOINTS), position=[float(v) for v in q])
    return state


def trajectory_positions(traj):
    names = list(traj.joint_trajectory.joint_names)
    return np.array([[p.positions[names.index(j)] for j in ARM_JOINTS]
                     for p in traj.joint_trajectory.points])


def stretch_to(traj, total):
    """MoveIt 궤적 시간을 total 초로 늘림 (속도 /k, 가속도 /k²). 이미 더 길면 그대로"""
    points = traj.joint_trajectory.points
    original = seconds(points[-1].time_from_start)
    if original <= 0.0:
        # 시간 정보가 없으면 5차 프로파일로 점마다 시간 배정 (속도는 비워 JTC 가 보간)
        n = len(points)
        for i, p in enumerate(points):
            tau = np.interp(i / (n - 1), s_quintic(np.linspace(0, 1, 201)), np.linspace(0, 1, 201))
            p.time_from_start = duration(total * tau)
            p.velocities, p.accelerations = [], []
        return original
    if original >= total:
        return original
    k = total / original
    for p in points:
        p.time_from_start = duration(seconds(p.time_from_start) * k)
        p.velocities = [v / k for v in p.velocities]
        p.accelerations = [a / k ** 2 for a in p.accelerations]
    return original


def time_spans(ts, mask):
    """mask 가 True 인 연속 구간 [(시작, 끝), ...]"""
    spans, start = [], None
    for t, m in zip(ts, mask):
        if m and start is None:
            start = t
        if not m and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, ts[-1]))
    return spans


# ===================== ROS 노드 =====================
class MoveItCompare(Node):

    def __init__(self):
        super().__init__('omy_moveit_compare')
        self.declare_parameter('motions', 'descend,transfer')
        self.declare_parameter('methods', 'A,B,C')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('obstacle', False)
        self.declare_parameter('obstacle_size', [0.10, 0.10, 0.30])
        self.declare_parameter('obstacle_center', [0.42, -0.10, 0.20])
        self.declare_parameter('obstacle_padding', 0.005)
        self.obstacle = self.get_parameter('obstacle').value
        self.padding = float(self.get_parameter('obstacle_padding').value)
        self.scene_padding = None   # 지금 planning scene 에 들어 있는 상자의 padding

        cb = ReentrantCallbackGroup()
        self.urdf_xml = None
        self.joint_positions = {}
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 50,
                                 callback_group=cb)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory',
                                           callback_group=cb)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path',
                                              callback_group=cb)
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path',
                                                   callback_group=cb)
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity',
                                                  callback_group=cb)
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                               callback_group=cb)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/compare_paths',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.markers = {}

        # 실행 중에만 기록 (self.samples 가 None 이 아니면)
        self.samples = None
        self.record_t0 = 0.0
        self.kin = None
        self.create_timer(1.0 / RECORD_HZ, self._record, callback_group=cb)

    # ---------- 콜백 ----------
    def _on_description(self, msg):
        self.urdf_xml = msg.data

    def _on_joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def tip_transform(self):
        tf = self.tf_buffer.lookup_transform(WORLD, TIP, rclpy.time.Time())
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quaternion_to_matrix(r.x, r.y, r.z, r.w)

    def tip_fk(self, q):
        return (self.base_to_world @ self.kin.fk(q)[0][:, 3])[:3]

    def _record(self):
        if self.samples is None or self.kin is None:
            return
        try:
            p_tf, _ = self.tip_transform()
        except tf2_ros.TransformException:
            return
        q = self.current_arm()
        self.samples.append([time.monotonic() - self.record_t0, *self.tip_fk(q), *p_tf, *q])

    # ---------- 기다리기 (executor 는 다른 스레드에서 돎) ----------
    @staticmethod
    def wait(future, timeout=60.0):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise TimeoutError('no response')
            time.sleep(0.002)
        return future.result()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{client.srv_name} not available (move_group running?)')
        return self.wait(client.call_async(request))

    def send_goal(self, client, goal):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('action server not available')
        handle = self.wait(client.send_goal_async(goal))
        if not handle.accepted:
            return None
        return self.wait(handle.get_result_async()).result

    def current_arm(self):
        return [self.joint_positions[j] for j in ARM_JOINTS]

    def wait_ready(self):
        self.get_logger().info('waiting for /robot_description, /joint_states, TF ...')
        while rclpy.ok():
            if self.urdf_xml is not None and all(j in self.joint_positions for j in ARM_JOINTS):
                try:
                    self.tip_transform()
                    break
                except tf2_ros.TransformException:
                    pass
            time.sleep(0.1)
        kin = Kinematics(self.urdf_xml)
        assert kin.joint_names == ARM_JOINTS, kin.joint_names
        # Kinematics 는 link0 기준 → world 기준으로 (OMY 는 world_fixed 가 원점이라 단위행렬)
        tf = self.tf_buffer.lookup_transform(WORLD, BASE_LINK, rclpy.time.Time()).transform
        self.base_to_world = np.eye(4)
        self.base_to_world[:3, :3] = quaternion_to_matrix(
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
        self.base_to_world[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
        self.kin = kin

    # ---------- 장애물 / 충돌 검사 ----------
    def use_scene(self, for_planning):
        """계획용 (padding 포함) / 검사용 (원래 크기) 상자로 바꿈"""
        padding = self.padding if for_planning else 0.0
        if self.obstacle and self.scene_padding != padding:
            self.set_obstacle(True, padding)

    def set_obstacle(self, enabled, padding=0.0):
        size = [float(v) for v in self.get_parameter('obstacle_size').value]
        padded = [v + 2 * padding for v in size]
        center = [float(v) for v in self.get_parameter('obstacle_center').value]
        obj = CollisionObject(id=OBSTACLE_ID)
        obj.header.frame_id = WORLD
        obj.operation = CollisionObject.ADD if enabled else CollisionObject.REMOVE
        if enabled:
            obj.pose.position = Point(x=center[0], y=center[1], z=center[2])
            obj.pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=padded)]
            box_pose = Pose()
            box_pose.orientation.w = 1.0
            obj.primitive_poses = [box_pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        ok = self.call(self.scene_client, ApplyPlanningScene.Request(scene=scene)).success
        # 지울 때는 상자가 원래 없으면 실패로 돌아오므로 무시
        if enabled and not ok:
            raise RuntimeError('/apply_planning_scene failed')
        self.scene_padding = padding if enabled else None
        if enabled and 'obstacle' not in self.markers:
            marker = Marker(type=Marker.CUBE, action=Marker.ADD, ns='obstacle')
            marker.header.frame_id = WORLD
            marker.pose = obj.pose
            marker.scale.x, marker.scale.y, marker.scale.z = size
            marker.color = ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.35)
            self.markers['obstacle'] = marker
            self.get_logger().info(f'obstacle box {size} at {center}, planning padding {self.padding} m')
        self.publish_markers()

    def check_states(self, qs):
        """관절각 목록 → (충돌 여부 배열, 부딪힌 링크 쌍 집합). 상자는 원래 크기로 검사"""
        self.use_scene(for_planning=False)
        hits, contacts = [], set()
        for q in qs:
            req = GetStateValidity.Request(group_name=GROUP, robot_state=arm_state(q))
            res = self.call(self.validity_client, req)
            hits.append(not res.valid)
            contacts |= {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}
        return np.array(hits, bool), contacts

    # ---------- 실행 ----------
    def move_joints(self, q, secs):
        """FollowJointTrajectory 한 점 (끝점 속도·가속도 0 → JTC 가 5차 보간)"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = duration(secs)
        goal.trajectory.points = [point]
        result = self.send_goal(self.arm_client, goal)
        return result is not None and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def execute(self, traj):
        result = self.send_goal(self.execute_client, ExecuteTrajectory.Goal(trajectory=traj))
        return result is not None and result.error_code.val == 1

    def plan_joint_goal(self, q_start, q_goal, scaling=1.0):
        self.use_scene(for_planning=True)
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = GROUP
        r.start_state = arm_state(q_start)
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-4,
                            tolerance_below=1e-4, weight=1.0)
            for j, v in zip(ARM_JOINTS, q_goal)])]
        r.pipeline_id = 'ompl'
        r.planner_id = 'RRTConnect'
        r.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        r.allowed_planning_time = self.get_parameter('planning_time').value
        r.max_velocity_scaling_factor = scaling
        r.max_acceleration_scaling_factor = scaling
        # -2 (INVALID_MOTION_PLAN) = TOTG 로 다듬은 경로가 상자에 닿아 move_group 이 거부 → 다시 계획
        for attempt in range(3):
            res = self.call(self.plan_client, req).motion_plan_response
            if res.error_code.val != MoveItErrorCodes.INVALID_MOTION_PLAN:
                break
            self.get_logger().warn(f'planned path touched an obstacle after time parameterization, '
                                   f'replanning ({attempt + 1}/3)')
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PlanFailed(f'/plan_kinematic_path failed (error {res.error_code.val})')
        return res.trajectory

    def go_to(self, q, secs):
        """장애물이 있으면 MoveIt 으로 계획해서 이동, 없으면 관절 직선 이동"""
        if not self.obstacle:
            return self.move_joints(q, secs)
        for attempt in range(3):
            try:
                return self.execute(self.plan_joint_goal(self.current_arm(), q, scaling=0.5))
            except PlanFailed as e:
                self.get_logger().warn(f'go_to attempt {attempt + 1} failed: {e}')
        raise PlanFailed('could not plan back to start pose')

    def recorded(self, run):
        """run() 을 실행하면서 손끝을 기록 → (성공, 실행 시간, 기록 배열)"""
        self.samples = []
        self.record_t0 = time.monotonic()
        ok = run()
        exec_time = time.monotonic() - self.record_t0
        time.sleep(0.3)   # 끝난 뒤 정지 상태까지 조금 더 기록
        samples, self.samples = np.array(self.samples), None
        return ok, exec_time, samples

    # ---------- 세 방식 ----------
    def plan_a(self, m, q0):
        q1 = ik_with_restarts(self.kin, gripper_down_pose(*m['p1'], m['yaw1']), q0)
        return (lambda: self.move_joints(q1, m['T'])), dict(q_goal=q1)

    def compute_ik(self, target, seed):
        self.use_scene(for_planning=True)
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.robot_state = arm_state(seed)
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = TIP
        req.ik_request.pose_stamped = PoseStamped(pose=to_pose(target))
        req.ik_request.pose_stamped.header.frame_id = WORLD
        req.ik_request.timeout = duration(0.5)
        res = self.call(self.ik_client, req)
        if res.error_code.val != 1:
            raise PlanFailed(f'/compute_ik failed (error {res.error_code.val})')
        names = list(res.solution.joint_state.name)
        return np.array([res.solution.joint_state.position[names.index(j)] for j in ARM_JOINTS])

    def plan_b(self, m, q0):
        q1 = self.compute_ik(gripper_down_pose(*m['p1'], m['yaw1']), q0)
        traj = self.plan_joint_goal(q0, q1)
        moveit_time = stretch_to(traj, m['T'])
        return (lambda: self.execute(traj)), dict(
            q_goal=q1, waypoints=len(traj.joint_trajectory.points), moveit_duration=moveit_time)

    def plan_c(self, m, q0):
        self.use_scene(for_planning=True)
        req = GetCartesianPath.Request()
        req.header.frame_id = WORLD
        req.start_state = arm_state(q0)
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = [to_pose(gripper_down_pose(*m['p1'], m['yaw1']))]
        req.max_step = CARTESIAN_STEP
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        res = self.call(self.cartesian_client, req)
        info = dict(fraction=res.fraction, waypoints=len(res.solution.joint_trajectory.points))
        if res.fraction < 1.0:
            self.get_logger().warn(f'cartesian path fraction {res.fraction:.3f}')
        if res.fraction < CARTESIAN_MIN_FRACTION:
            partial = trajectory_positions(res.solution) if info['waypoints'] else None
            raise PlanFailed(f'cartesian path fraction {res.fraction:.3f} < {CARTESIAN_MIN_FRACTION}',
                             partial)
        traj = res.solution
        info['moveit_duration'] = stretch_to(traj, m['T'])
        return (lambda: self.execute(traj)), info

    # ---------- 한 번 실행 ----------
    def run_one(self, name, m, method, q0, out_dir):
        """→ (metrics 한 줄, 그래프용 (t, 이탈 mm, 충돌 mask 또는 None) 또는 None)"""
        p0, p1 = np.array(m['p0']), np.array(m['p1'])
        target = gripper_down_pose(*m['p1'], m['yaw1'])
        row = dict(motion=name, method=method, executed=False, success=False, plan_ms=None,
                   exec_s=None, max_dev_mm=None, final_pos_err_mm=None, final_rot_err_deg=None,
                   joint_travel_rad=None, collisions='', note='')

        self.go_to(q0, 2.0)
        time.sleep(0.3)
        t0 = time.perf_counter()
        try:
            run, info = getattr(self, f'plan_{method.lower()}')(m, q0)
        except (PlanFailed, ValueError, TimeoutError) as e:
            row['plan_ms'] = (time.perf_counter() - t0) * 1000
            row['note'] = str(e)
            self.get_logger().error(f'[{name} {method}] plan failed: {e}')
            partial = getattr(e, 'partial', None)
            if partial is not None and len(partial) > 1:
                pts = np.array([self.tip_fk(q) for q in partial])
                self.add_path_marker(name, method, pts, alpha=0.5)
                row['note'] += f' (partial path {np.linalg.norm(pts[-1] - p0) * 1000:.0f} mm)'
            return row, None
        row['plan_ms'] = (time.perf_counter() - t0) * 1000
        if 'fraction' in info:
            row['note'] += f'fraction {info["fraction"]:.3f} '
        if 'moveit_duration' in info:
            row['note'] += f'MoveIt {info["moveit_duration"]:.2f} s → {m["T"]} s '
        if 'waypoints' in info:
            row['note'] += f'{info["waypoints"]} pts'

        if method == 'A' and self.obstacle:
            return self.check_a(name, m, q0, info['q_goal'], row)

        ok, exec_time, samples = self.recorded(run)
        p_end, r_end = self.tip_transform()
        row.update(executed=True, success=ok, exec_s=exec_time)
        if len(samples) < 2:
            row['note'] += ' (no samples)'
            return row, None

        pts, qs = samples[:, 1:4], samples[:, 7:13]
        dev = line_deviation(pts, p0, p1) * 1000
        idx = np.arange(0, len(qs), CHECK_EVERY)
        hit_checked, contacts = self.check_states(qs[idx])
        hit = np.zeros(len(qs), bool)
        hit[idx] = hit_checked
        row.update(max_dev_mm=float(dev.max()),
                   final_pos_err_mm=float(np.linalg.norm(p_end - p1) * 1000),
                   final_rot_err_deg=float(math.degrees(np.linalg.norm(
                       rotation_error(target[:3, :3], r_end)))),
                   joint_travel_rad=float(np.abs(np.diff(qs, axis=0)).sum()),
                   collisions=f'{int(hit_checked.sum())}/{len(idx)}')
        if contacts:
            row['note'] += f' contacts {sorted(contacts)}'

        with open(os.path.join(out_dir, f'traj_{name}_{method}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'tf_x', 'tf_y', 'tf_z', 'dev_mm', 'collision', *ARM_JOINTS])
            for s, d, h, checked in zip(samples, dev, hit, np.isin(np.arange(len(qs)), idx)):
                w.writerow([f'{s[0]:.4f}', *(f'{v:.5f}' for v in s[1:7]), f'{d:.3f}',
                            int(h) if checked else '', *(f'{v:.5f}' for v in s[7:])])
        self.add_path_marker(name, method, pts)
        return row, (samples[:, 0], dev, None)

    def check_a(self, name, m, q0, q1, row):
        """장애물 모드의 A - 실행하지 않고 관절 5차 보간 경로를 점마다 충돌 검사"""
        ts = np.arange(0.0, m['T'] + 1e-9, CHECK_DT)
        qs = np.array([q0 + (q1 - q0) * s_quintic(t / m['T']) for t in ts])
        pts = np.array([self.tip_fk(q) for q in qs])
        dev = line_deviation(pts, np.array(m['p0']), np.array(m['p1'])) * 1000
        hit, contacts = self.check_states(qs)
        spans = time_spans(ts, hit)
        row.update(max_dev_mm=float(dev.max()),
                   joint_travel_rad=float(np.abs(q1 - q0).sum()),
                   collisions=f'{int(hit.sum())}/{len(ts)}')
        row['note'] = ('not executed (collision check only)'
                       + (f', in collision {", ".join(f"{a:.2f}-{b:.2f} s" for a, b in spans)}'
                          if spans else ', no collision')
                       + (f', contacts {sorted(contacts)}' if contacts else ''))

        with open(os.path.join(self.out_dir, f'traj_{name}_A.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'dev_mm', 'collision', *ARM_JOINTS])
            for t, p, d, h, q in zip(ts, pts, dev, hit, qs):
                w.writerow([f'{t:.4f}', *(f'{v:.5f}' for v in p), f'{d:.3f}', int(h),
                            *(f'{v:.5f}' for v in q)])
        self.add_path_marker(name, 'A', pts, alpha=0.6)
        self.add_collision_marker(name, pts[hit])
        return row, (ts, dev, hit)

    # ---------- RViz 마커 ----------
    def add_path_marker(self, motion, method, pts, alpha=1.0):
        marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns=f'{motion}/{method}')
        marker.header.frame_id = WORLD
        marker.scale.x = 0.003
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=METHODS[method]['color'][0], g=METHODS[method]['color'][1],
                                 b=METHODS[method]['color'][2], a=alpha)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        self.markers[marker.ns] = marker
        self.publish_markers()

    def add_collision_marker(self, motion, pts):
        if len(pts) == 0:
            return
        marker = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD, ns=f'{motion}/A_collision')
        marker.header.frame_id = WORLD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.008
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        self.markers[marker.ns] = marker
        self.publish_markers()

    def add_reference_marker(self, motion, m):
        marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns=f'{motion}/straight')
        marker.header.frame_id = WORLD
        marker.scale.x = 0.0015
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.8)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (m['p0'], m['p1'])]
        self.markers[marker.ns] = marker
        self.publish_markers()

    def publish_markers(self):
        """이전 실행의 마커까지 지우고 (DELETEALL) 지금 마커 전체를 다시 보냄"""
        stamp = self.get_clock().now().to_msg()
        for i, marker in enumerate(self.markers.values()):
            marker.id = i
            marker.header.stamp = stamp
        clear = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[clear, *self.markers.values()]))

    # ---------- 6주차 numpy 예측 (방식 A, 관절 5차 보간) ----------
    def predict_a(self, m, q0, q1):
        ts = np.arange(0.0, m['T'] + 1e-9, 0.01)
        pts = np.array([self.tip_fk(q0 + (q1 - q0) * s_quintic(t / m['T'])) for t in ts])
        return ts, line_deviation(pts, np.array(m['p0']), np.array(m['p1'])) * 1000

    # ---------- 전체 ----------
    def run_all(self):
        self.wait_ready()
        motions = [s.strip() for s in self.get_parameter('motions').value.split(',') if s.strip()]
        methods = [s.strip().upper() for s in self.get_parameter('methods').value.split(',') if s.strip()]
        self.out_dir = self.get_parameter('output_dir').value or os.path.join(
            os.path.expanduser('~/robot_ws/moveit_compare'),
            datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + ('_obstacle' if self.obstacle else ''))
        os.makedirs(self.out_dir, exist_ok=True)

        self.publish_markers()   # 이전 실행 마커 지우기
        self.set_obstacle(self.obstacle)
        self.get_logger().info('ready pose')
        self.go_to(READY_POSE, 3.0)
        rows = []
        for name in motions:
            m = MOTIONS[name]
            q0 = ik_with_restarts(self.kin, gripper_down_pose(*m['p0'], m['yaw0']),
                                  np.array(READY_POSE))
            q1 = ik_with_restarts(self.kin, gripper_down_pose(*m['p1'], m['yaw1']), q0)
            if self.obstacle:
                hit, contacts = self.check_states([q0, q1])
                if hit.any():
                    self.get_logger().warn(
                        f'{name}: start in collision {hit[0]}, goal in collision {hit[1]} {sorted(contacts)}'
                        ' - change obstacle_center / obstacle_size')
            self.add_reference_marker(name, m)
            curves = {}
            for method in methods:
                self.get_logger().info(f'=== {m["label"]} / {METHODS[method]["label"]}')
                row, curve = self.run_one(name, m, method, q0, self.out_dir)
                rows.append(row)
                if curve is not None:
                    curves[method] = curve
                self.get_logger().info(
                    f'executed {row["executed"]}, success {row["success"]}, '
                    f'max dev {fmt(row["max_dev_mm"])} mm, final err {fmt(row["final_pos_err_mm"], 2)} mm, '
                    f'plan {fmt(row["plan_ms"])} ms, exec {fmt(row["exec_s"], 2)} s, '
                    f'collisions {row["collisions"]}  {row["note"]}')
            self.plot(name, m, curves, self.predict_a(m, q0, q1), self.out_dir)

        with open(os.path.join(self.out_dir, 'metrics.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print_table(rows)
        self.get_logger().info(f'saved to {self.out_dir}')
        self.go_to(READY_POSE, 2.0)

    def plot(self, name, m, curves, prediction, out_dir):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.rcParams['font.family'] = ['Noto Sans CJK KR', 'Noto Sans CJK JP', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False

        fig, ax = plt.subplots(figsize=(7, 4), dpi=150)
        for method, (ts, dev, hit) in curves.items():
            label = f'{METHODS[method]["label"]}  최대 {dev.max():.1f} mm'
            if hit is not None:   # 장애물 모드 A - 실행 안 한 예측 경로
                label += f'  (실행 안 함, 충돌 {hit.sum()}/{len(hit)}점)'
                for i, (a, b) in enumerate(time_spans(ts, hit)):
                    ax.axvspan(a, b, color='red', alpha=0.12, label='A 충돌 구간' if i == 0 else None)
            ax.plot(ts, dev, color=METHODS[method]['color'], lw=1.8, label=label)
        ax.plot(prediction[0], prediction[1], 'k--', lw=1.0,
                label=f'A 예측 (6주차 numpy)  최대 {prediction[1].max():.1f} mm')
        ax.set_xlabel('시간 (s)')
        ax.set_ylabel('직선에서 벗어난 거리 (mm)')
        title = f'{m["label"]}  {m["p0"]} → {m["p1"]}, {m["T"]} s'
        if self.obstacle:
            title += '  + 장애물'
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'deviation_{name}.png'))
        plt.close(fig)


def fmt(v, digits=1):
    return '-' if v is None else f'{v:.{digits}f}'


def print_table(rows):
    print()
    print(f'{"동작":10s} {"방식":4s} {"실행":4s} {"성공":4s} {"계획 ms":>8s} {"실행 s":>7s} {"이탈 최대 mm":>12s} '
          f'{"도착 오차 mm":>12s} {"자세 오차°":>10s} {"Σ|Δq| rad":>10s} {"충돌":>8s}  비고')
    for r in rows:
        print(f'{r["motion"]:10s} {r["method"]:4s} {"O" if r["executed"] else "-":4s} '
              f'{"O" if r["success"] else "X":4s} '
              f'{fmt(r["plan_ms"]):>8s} {fmt(r["exec_s"], 2):>7s} {fmt(r["max_dev_mm"]):>12s} '
              f'{fmt(r["final_pos_err_mm"], 2):>12s} {fmt(r["final_rot_err_deg"], 2):>10s} '
              f'{fmt(r["joint_travel_rad"], 3):>10s} {r["collisions"]:>8s}  {r["note"]}')
    print()


def main():
    rclpy.init()
    node = MoveItCompare()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_all()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
