#!/usr/bin/env python3
"""
SCARA 모바일 매니퓰레이터 데모
  1) 베이스를 앞으로 조금 이동
  2) 역기구학으로 목표 (x, y, z)에 팔 이동 → quill 하강 → 그리퍼 닫기 → quill 상승
  3) 팔 원위치

좌표계: base_footprint 기준 (x 앞, y 왼쪽, z 위), 단위 m
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint

# common.xacro 와 같은 값
WHEEL_RADIUS = 0.05
BASE_HEIGHT = 0.10
ARM_MOUNT_X = 0.05
COLUMN_HEIGHT = 0.25
LINK1_LENGTH = 0.20
LINK1_HEIGHT = 0.03
LINK2_LENGTH = 0.15
QUILL_BOTTOM = 0.15
QUILL_STROKE = 0.18
GRIPPER_BASE_Z = 0.02
FINGER_Z = 0.06

# joint3 = 0 일 때 end_effector_link 의 높이
TIP_Z_AT_ZERO = (WHEEL_RADIUS + BASE_HEIGHT + COLUMN_HEIGHT + LINK1_HEIGHT
                 - QUILL_BOTTOM - GRIPPER_BASE_Z - FINGER_Z)

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']


def scara_ik(x, y, z, yaw=0.0, elbow_up=True):
    """base_footprint 기준 목표점 → [q1, q2, q3, q4]. 도달 불가면 ValueError."""
    px = x - ARM_MOUNT_X
    py = y
    l1, l2 = LINK1_LENGTH, LINK2_LENGTH

    # 코사인 법칙으로 팔꿈치 각도
    c2 = (px * px + py * py - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if abs(c2) > 1.0:
        raise ValueError(f'({x:.3f}, {y:.3f}) is out of reach')
    q2 = math.acos(c2) * (-1 if elbow_up else 1)
    q1 = math.atan2(py, px) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))

    # quill 은 아래로 갈수록 양수
    q3 = TIP_Z_AT_ZERO - z
    if not 0.0 <= q3 <= QUILL_STROKE:
        raise ValueError(f'z={z:.3f} is out of quill range')

    # 그리퍼 방향 = q1 + q2 + q4
    q4 = math.atan2(math.sin(yaw - q1 - q2), math.cos(yaw - q1 - q2))
    return [q1, q2, q3, q4]


class ScaraDemo(Node):

    def __init__(self):
        super().__init__('scara_demo')
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

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
            'arm -> ' + ', '.join(f'{n}={p:.3f}' for n, p in zip(ARM_JOINTS, positions)))
        self._send_and_wait(self.arm_client, goal)

    def move_to(self, x, y, z, yaw=0.0, seconds=2.0):
        self.move_arm(scara_ik(x, y, z, yaw), seconds)

    def gripper(self, opening):
        goal = GripperCommand.Goal()
        goal.command.position = float(opening)
        goal.command.max_effort = 10.0
        self.get_logger().info(f'gripper -> {opening:.3f}')
        self._send_and_wait(self.gripper_client, goal)


def main():
    rclpy.init()
    demo = ScaraDemo()

    demo.get_logger().info('1) drive forward')
    demo.drive(0.2, 0.0, 2.0)

    demo.get_logger().info('2) pick motion')
    demo.gripper(0.03)
    demo.move_to(0.30, 0.15, 0.15)
    demo.move_to(0.30, 0.15, 0.03, seconds=1.5)
    demo.gripper(0.0)
    demo.move_to(0.30, 0.15, 0.15, seconds=1.5)
    demo.move_to(0.25, -0.20, 0.15, yaw=math.pi / 2)

    demo.get_logger().info('3) home')
    demo.move_arm([0.0, 0.0, 0.0, 0.0])
    demo.gripper(0.0)

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
