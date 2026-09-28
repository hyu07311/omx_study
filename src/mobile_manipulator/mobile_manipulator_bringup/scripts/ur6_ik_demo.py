#!/usr/bin/env python3
"""
UR식 6축 모바일 매니퓰레이터 - 야코비안 기반 수치 역기구학 데모
  1) 베이스를 앞으로 조금 이동
  2) 준비 자세 → 목표 위치/자세로 팔 이동 → 하강 → 그리퍼 닫기 → 상승 → 다른 위치로 이동
  3) 원위치

기구학 정보는 /robot_description 토픽의 URDF에서 직접 읽음 (치수 하드코딩 없음)
좌표계는 base_footprint 기준 (x 앞, y 왼쪽, z 위), 단위 m / rad

IK = damped least squares
    dq = J^T (J J^T + lambda^2 I)^-1 e
    e  = [목표위치 - 현재위치 ; 자세오차(축각 벡터)]
"""
import math
import time

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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_LINK = 'base_footprint'
TIP_LINK = 'end_effector_link'
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# 팔이 곧게 선 0 자세는 특이점(팔꿈치가 펴짐)이라 IK 초기값으로 나쁨 → 먼저 이 자세로 이동
# q2 + q3 + q4 = pi/2 이면 wrist2 축이 수평, q5 = -pi/2 이면 그리퍼가 아래를 향함
READY_POSE = [0.0, 0.6, 1.2, math.pi / 2 - 1.8, -math.pi / 2, 0.0]


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


def gripper_down_pose(x, y, z, yaw=0.0):
    """손가락이 아래를 향하는 목표 자세 (end_effector_link 의 z 가 위, yaw 는 z축 회전)"""
    return homogeneous(rpy_to_matrix(0.0, 0.0, yaw), [x, y, z])


# ===================== ROS 노드 =====================
class UR6Demo(Node):

    def __init__(self):
        super().__init__('ur6_ik_demo')
        self.urdf_xml = None
        self.joint_positions = {}
        self.create_subscription(
            String, '/robot_description', self._on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info('waiting for /robot_description and /joint_states ...')
        while rclpy.ok() and (self.urdf_xml is None
                              or not all(j in self.joint_positions for j in ARM_JOINTS)):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.kin = Kinematics(self.urdf_xml)
        assert self.kin.joint_names == ARM_JOINTS, self.kin.joint_names

    def _on_description(self, msg):
        self.urdf_xml = msg.data

    def _on_joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def current_arm(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        return [self.joint_positions[j] for j in ARM_JOINTS]

    def drive(self, linear, angular, seconds):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def _send_and_wait(self, client, goal):
        client.wait_for_server()
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
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
        q = self.kin.ik(target, self.current_arm())
        t, _ = self.kin.fk(q)
        self.get_logger().info(
            f'IK target ({x:.3f}, {y:.3f}, {z:.3f}) yaw {math.degrees(yaw):.0f} deg '
            f'-> FK ({t[0, 3]:.4f}, {t[1, 3]:.4f}, {t[2, 3]:.4f})')
        self.move_arm(q, seconds)

    def gripper(self, opening):
        goal = GripperCommand.Goal()
        goal.command.position = float(opening)
        goal.command.max_effort = 10.0
        self.get_logger().info(f'gripper -> {opening:.3f}')
        self._send_and_wait(self.gripper_client, goal)


def main():
    rclpy.init()
    demo = UR6Demo()

    demo.get_logger().info('1) drive forward')
    demo.drive(0.2, 0.0, 2.0)

    demo.get_logger().info('2) pick motion')
    demo.gripper(0.03)
    demo.move_arm(READY_POSE, 3.0)
    demo.move_to(0.40, 0.0, 0.20)
    demo.move_to(0.40, 0.0, 0.05, seconds=1.5)
    demo.gripper(0.0)
    demo.move_to(0.40, 0.0, 0.20, seconds=1.5)
    demo.move_to(0.30, -0.25, 0.20, yaw=math.pi / 2)

    demo.get_logger().info('3) home')
    demo.move_arm(READY_POSE, 2.0)
    demo.move_arm([0.0] * 6, 3.0)

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
