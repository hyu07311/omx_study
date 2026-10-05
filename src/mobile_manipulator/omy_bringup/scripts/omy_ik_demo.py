#!/usr/bin/env python3
"""
OMY-F3M 야코비안 기반 수치 역기구학 데모 (ur6_ik_demo.py 의 IK 를 그대로 사용)
  1) 준비 자세 (그리퍼가 아래, D405 가 테이블을 내려다봄)
  2) 빨간 큐브 위 → 하강 → 그리퍼 닫기 → 상승 → 옆으로 옮겨 내려놓기
  3) 원위치

ur6 버전과 다른 점
  - 베이스가 바닥 고정 (주행 없음), 기준 좌표계 = link0
  - 손끝 방향이 end_effector_link 의 -y (OMY 플랜지 규칙)
  - 관절 범위가 ±2π 로 넓어서 국소해/큰 각도 해가 잘 나옴
    → 무작위 초기값으로 여러 번 풀고, 현재 자세에서 가장 가까운 해를 선택
  - 그리퍼는 rh_r1_joint 각도 (0 = 열림, 1.135 rad = 닫힘)

기구학 정보는 /robot_description 토픽의 URDF에서 직접 읽음 (치수 하드코딩 없음)
"""
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from urdf_parser_py.urdf import URDF

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_LINK = 'link0'
TIP_LINK = 'end_effector_link'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# 손끝 (0.40, 0, 0.30) 그리퍼 아래 방향의 IK 해
# q2 + q3 + q4 = pi/2 이면 손목이 수평, q5 = pi/2 이면 손끝(joint6 축)이 수직
# 손끝을 아래로 향하면 높이는 약 0.39 m 가 한계 (joint4 최대 높이 0.638 - 손끝 길이 0.244)
READY_POSE = [0.286, 0.454, 0.361, 0.755, 1.571, -1.857]

GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.1
# Gazebo 에서는 그리퍼가 effort 제어 - 최대 토크 (Nm). 물체에 닿으면 이 힘으로 쥐고 멈춤
GRIPPER_EFFORT = 2.0

# table.world 의 빨간 큐브 (한 변 0.04 m, 중심 높이 0.12 m)
CUBE_X, CUBE_Y, CUBE_Z = 0.45, 0.0, 0.12
# 집는 높이 (end_effector_link). 손가락은 닫히면서 호를 그리며 내려가서, 이 높이면 닿는 순간
# 손끝 아래면이 약 0.107 m (테이블 0.10 m 위 7 mm) → 큐브 옆면 0.10 ~ 0.14 m 중 3 cm 를 쥠
GRASP_Z = CUBE_Z - 0.005


# ===================== 기구학 유틸 =====================
def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_to_matrix(axis, angle):
    """로드리게스 회전 공식"""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def homogeneous(rot, pos):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = pos
    return t


def rotation_error(r_target, r_current):
    """r_current 를 r_target 으로 돌리는 회전을 축각 벡터(axis * angle)로 반환"""
    r = r_target @ r_current.T
    cos_angle = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-4:
        # 180도 근처는 sin 이 0 에 가까워 아래 식이 불안정 → 고유값 1 인 고유벡터가 회전축
        w, v = np.linalg.eig(r)
        axis = np.real(v[:, np.argmin(np.abs(w - 1.0))])
        return axis / np.linalg.norm(axis) * angle
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return vee / (2.0 * math.sin(angle)) * angle


class Kinematics:
    """URDF 체인 (base → tip) 으로 FK / 야코비안 / IK 계산"""

    def __init__(self, urdf_xml, base=BASE_LINK, tip=TIP_LINK):
        robot = URDF.from_xml_string(urdf_xml)
        self.chain = []          # (origin 변환, 관절 종류, 축, 관절 이름)
        self.joint_names = []
        lower, upper = [], []
        for name in robot.get_chain(base, tip, joints=True, links=False):
            joint = robot.joint_map[name]
            xyz = joint.origin.xyz if joint.origin and joint.origin.xyz else [0, 0, 0]
            rpy = joint.origin.rpy if joint.origin and joint.origin.rpy else [0, 0, 0]
            origin = homogeneous(rpy_to_matrix(*rpy), xyz)
            axis = np.array(joint.axis if joint.axis else [1, 0, 0], dtype=float)
            self.chain.append((origin, joint.type, axis, name))
            if joint.type in ('revolute', 'continuous'):
                self.joint_names.append(name)
                lower.append(joint.limit.lower if joint.type == 'revolute' else -math.pi)
                upper.append(joint.limit.upper if joint.type == 'revolute' else math.pi)
        self.lower = np.array(lower)
        self.upper = np.array(upper)

    def fk(self, q):
        """끝점 변환행렬과 각 회전관절의 (월드 축, 월드 위치) 목록"""
        t = np.eye(4)
        joints = []
        i = 0
        for origin, jtype, axis, _ in self.chain:
            t = t @ origin
            if jtype in ('revolute', 'continuous'):
                joints.append((t[:3, :3] @ axis, t[:3, 3].copy()))
                t = t @ homogeneous(axis_angle_to_matrix(axis, q[i]), [0, 0, 0])
                i += 1
        return t, joints

    def jacobian(self, q):
        """기하 야코비안 6xN, 회전관절 i 열 = [z_i x (p_e - p_i) ; z_i]"""
        t, joints = self.fk(q)
        p_end = t[:3, 3]
        jac = np.zeros((6, len(joints)))
        for i, (z, p) in enumerate(joints):
            jac[:3, i] = np.cross(z, p_end - p)
            jac[3:, i] = z
        return jac

    def ik(self, target, q0, damping=0.05, max_iter=500,
           tol_pos=1e-4, tol_rot=1e-3, max_step=0.2):
        q = np.clip(np.array(q0, dtype=float), self.lower, self.upper)
        for _ in range(max_iter):
            t, _ = self.fk(q)
            err = np.concatenate([target[:3, 3] - t[:3, 3],
                                  rotation_error(target[:3, :3], t[:3, :3])])
            if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_rot:
                return q
            jac = self.jacobian(q)
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(6), err)
            # 한 번에 너무 크게 움직이지 않도록 제한
            norm = np.linalg.norm(dq)
            if norm > max_step:
                dq *= max_step / norm
            q = np.clip(q + dq, self.lower, self.upper)
        raise ValueError(
            f'IK did not converge (pos err {np.linalg.norm(err[:3]) * 1000:.1f} mm, '
            f'rot err {math.degrees(np.linalg.norm(err[3:])):.2f} deg)')


def wrap(q):
    """각도를 -pi ~ pi 로 (±2π 범위라 같은 자세가 여러 각도로 표현됨)"""
    return (np.asarray(q) + math.pi) % (2 * math.pi) - math.pi


def ik_with_restarts(kin, target, q_now, tries=30, seed=0):
    """
    여러 초기값으로 IK 를 풀고, 성공한 해 중 현재 자세와 가장 가까운 것을 반환
      1) 현재 자세
      2) 현재 자세에서 joint6 만 ±90°, 180° 돌린 자세 (손끝 yaw 만 바뀌는 목표는 대부분 여기서 풀림)
      3) 무작위 자세 - 위에서 못 풀었을 때만, 해를 3개 찾으면 중단
    """
    rng = np.random.default_rng(seed)
    q_now = np.asarray(q_now, dtype=float)
    seeds = [q_now]
    for dq6 in (math.pi / 2, -math.pi / 2, math.pi, -math.pi):
        q = q_now.copy()
        q[5] += dq6
        seeds.append(np.clip(q, kin.lower, kin.upper))
    n_fixed = len(seeds)
    seeds += [rng.uniform(np.maximum(kin.lower, -math.pi), np.minimum(kin.upper, math.pi))
              for _ in range(tries)]

    solutions = []
    for i, q0 in enumerate(seeds):
        if i >= n_fixed and len(solutions) >= 3:
            break
        if i == n_fixed and solutions:
            break
        try:
            q = kin.ik(target, q0)
        except ValueError:
            continue
        # joint3 (±150°) 을 제외한 관절은 현재 각도에 가장 가까운 2π 배수로 맞춤
        q = np.clip(np.where(np.abs(kin.upper) > math.pi, q_now + wrap(q - q_now), q),
                    kin.lower, kin.upper)
        solutions.append(q)
    if not solutions:
        raise ValueError('IK failed for all initial values')
    return min(solutions, key=lambda q: np.linalg.norm(q - q_now))


def gripper_down_pose(x, y, z, yaw=0.0):
    """손끝이 아래를 향하는 목표 자세 (end_effector_link 의 -y 가 아래, yaw 는 z축 회전)"""
    return homogeneous(rpy_to_matrix(0.0, 0.0, yaw) @ rpy_to_matrix(math.pi / 2, 0.0, 0.0),
                       [x, y, z])


# ===================== ROS 노드 =====================
class OMYDemo(Node):

    def __init__(self):
        super().__init__('omy_ik_demo')
        self.urdf_xml = None
        self.joint_positions = {}
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info('waiting for /robot_description and /joint_states ...')
        while rclpy.ok() and (self.urdf_xml is None
                              or not all(j in self.joint_positions for j in ARM_JOINTS)):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.kin = Kinematics(self.urdf_xml, base=BASE_LINK, tip=TIP_LINK)
        assert self.kin.joint_names == ARM_JOINTS, self.kin.joint_names

    def _on_description(self, msg):
        self.urdf_xml = msg.data

    def _on_joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def current_arm(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        return [self.joint_positions[j] for j in ARM_JOINTS]

    def _send_and_wait(self, client, goal, timeout_sec=None):
        client.wait_for_server()
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            # 그리퍼는 물체를 쥔 채로 계속 힘을 주므로 goal 이 끝나지 않을 수 있음 → 다음 동작으로 진행
            self.get_logger().warn(f'no result within {timeout_sec} s, continuing')
            return None
        return result_future.result()

    def move_arm(self, positions, seconds=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int(seconds % 1 * 1e9))
        goal.trajectory.points = [point]
        self.get_logger().info(
            'arm -> ' + ', '.join(f'{p:+.3f}' for p in positions))
        self._send_and_wait(self.arm_client, goal)

    def move_to(self, x, y, z, yaw=0.0, seconds=2.0):
        target = gripper_down_pose(x, y, z, yaw)
        q = ik_with_restarts(self.kin, target, self.current_arm())
        t, _ = self.kin.fk(q)
        self.get_logger().info(
            f'IK target ({x:.3f}, {y:.3f}, {z:.3f}) yaw {math.degrees(yaw):.0f} deg '
            f'-> FK ({t[0, 3]:.4f}, {t[1, 3]:.4f}, {t[2, 3]:.4f})')
        self.move_arm(q, seconds)

    def gripper(self, angle):
        goal = GripperCommand.Goal()
        goal.command.position = float(angle)
        goal.command.max_effort = GRIPPER_EFFORT
        self.get_logger().info(f'gripper -> {angle:.3f} rad')
        result = self._send_and_wait(self.gripper_client, goal, timeout_sec=3.0)
        if result is not None:
            r = result.result
            self.get_logger().info(
                f'gripper at {r.position:.3f} rad, '
                f'{"stalled (holding object)" if r.stalled else "reached goal"}')


def main():
    rclpy.init()
    demo = OMYDemo()

    demo.get_logger().info('1) ready pose')
    demo.gripper(GRIPPER_OPEN)
    demo.move_arm(READY_POSE, 3.0)

    demo.get_logger().info('2) pick red cube and place it to the right')
    demo.move_to(CUBE_X, CUBE_Y, CUBE_Z + 0.12)
    demo.move_to(CUBE_X, CUBE_Y, GRASP_Z, seconds=1.5)
    demo.gripper(GRIPPER_CLOSE)
    demo.move_to(CUBE_X, CUBE_Y, CUBE_Z + 0.12, seconds=1.5)
    # 손목을 90° 돌리면서 옮기므로 천천히 (빠르면 관성 때문에 물체를 놓침)
    demo.move_to(0.35, -0.20, CUBE_Z + 0.12, yaw=math.pi / 2, seconds=3.0)
    demo.move_to(0.35, -0.20, GRASP_Z + 0.005, yaw=math.pi / 2, seconds=1.5)
    demo.gripper(GRIPPER_OPEN)
    demo.move_to(0.35, -0.20, CUBE_Z + 0.12, yaw=math.pi / 2, seconds=1.5)

    demo.get_logger().info('3) home')
    demo.move_arm(READY_POSE, 2.0)
    demo.move_arm([0.0] * 6, 3.0)

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
