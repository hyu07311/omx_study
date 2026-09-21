#!/usr/bin/env python3
"""
MoveIt의 move_group 액션 서버(/move_action)에 직접 관절 목표(joint goal)를 보내는 예제.
moveit_py 없이도 rclpy만으로 MoveIt을 제어할 수 있음을 보여준다.

사전 준비:
  ros2 launch open_manipulator_x_moveit_config demo.launch.py
가 이미 실행 중이어야 한다 (move_group 액션 서버가 떠 있어야 함).

실행:
  python3 joint_goal_example.py
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.msg import PlanningOptions


# ---- 여기 값들을 바꿔가며 실험해보세요 ----
PLANNING_GROUP = "arm"        # SRDF에 정의된 플래닝 그룹 이름 (arm / gripper 등)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
TARGET_POSITIONS = [0.3, -0.3, 0.3, 0.3]   # 라디안 단위, 4개 관절 목표값
TOLERANCE = 0.01
PLANNER_ID = "RRTConnectkConfigDefault"
# -----------------------------------------


class JointGoalSender(Node):
    def __init__(self):
        super().__init__("joint_goal_sender")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self):
        self.get_logger().info("move_action 서버를 기다리는 중...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "move_action 서버를 찾을 수 없습니다. "
                "demo.launch.py(move_group)가 실행 중인지 확인하세요."
            )
            return False

        # 관절마다 JointConstraint 하나씩 생성
        joint_constraints = []
        for name, pos in zip(JOINT_NAMES, TARGET_POSITIONS):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = TOLERANCE
            jc.tolerance_below = TOLERANCE
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints

        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.planner_id = PLANNER_ID
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        options = PlanningOptions()
        options.plan_only = False  # False = 계획 후 바로 실행까지

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        self.get_logger().info(
            f"목표 전송: group={PLANNING_GROUP}, joints={JOINT_NAMES}, "
            f"target={TARGET_POSITIONS}"
        )

        send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다.")
            return False

        self.get_logger().info("목표 승인됨. 결과 대기 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # error_code.val == 1 이면 SUCCESS (moveit_msgs/MoveItErrorCodes)
        if result.error_code.val == 1:
            self.get_logger().info("성공적으로 이동 완료!")
            return True
        else:
            self.get_logger().error(
                f"실패. error_code = {result.error_code.val} "
                "(1=성공, 그 외는 moveit_msgs/MoveItErrorCodes 참고)"
            )
            return False

    def _feedback_cb(self, feedback_msg):
        state = feedback_msg.feedback.state
        if state:
            self.get_logger().info(f"진행 상태: {state}")


def main():
    rclpy.init()
    node = JointGoalSender()
    try:
        node.send_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
