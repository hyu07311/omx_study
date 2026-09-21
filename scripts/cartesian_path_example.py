#!/usr/bin/env python3
"""
compute_cartesian_path 서비스를 이용해, 엔드이펙터가 직선 경로로
아래로 내려갔다가(approach) 다시 올라오는(retreat) 궤적을 계획하고 실행한다.

일반 관절 보간 경로(joint_goal_example.py)와 달리, 엔드이펙터가
공간상 '직선'으로 움직인다는 점이 다르다. 픽앤플레이스에서 물체에
접근/후퇴할 때 실제로 쓰이는 방식이다.

사전 준비: fake hardware + move_group이 떠 있어야 한다.

실행 전 반드시 확인:
  END_EFFECTOR_LINK 값이 실제 SRDF의 tip_link와 일치하는지
  (아래 grep 명령으로 확인 후 필요시 값을 바꿔서 실행)
    grep -A2 "group_state\|tip_link\|end_effector" \
      $(ros2 pkg prefix open_manipulator_x_moveit_config)/share/open_manipulator_x_moveit_config/config/*.srdf
"""

import sys
import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose

# ---- 환경에 맞게 확인/수정 ----
PLANNING_GROUP = "arm"
BASE_FRAME = "world"                 # RViz Fixed Frame과 동일하게
END_EFFECTOR_LINK = "end_effector_link"   # SRDF의 tip_link 이름으로 교체 필요
DESCEND_DISTANCE = 0.05              # 5cm 아래로 내려가기
# --------------------------------


class CartesianPathDemo(Node):
    def __init__(self):
        super().__init__("cartesian_path_demo")
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def get_current_pose(self):
        """TF를 이용해 엔드이펙터의 현재 자세를 구한다."""
        self.get_logger().info("현재 엔드이펙터 위치 조회 중 (TF)...")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self._tf_buffer.can_transform(
                BASE_FRAME, END_EFFECTOR_LINK, rclpy.time.Time()
            ):
                break
        try:
            t = self._tf_buffer.lookup_transform(
                BASE_FRAME, END_EFFECTOR_LINK, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f"TF 조회 실패: {e}")
            return None

        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return pose

    def plan_cartesian(self, waypoints):
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_cartesian_path 서비스를 찾을 수 없습니다.")
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.group_name = PLANNING_GROUP
        request.link_name = END_EFFECTOR_LINK
        request.waypoints = waypoints
        request.max_step = 0.01          # 1cm 간격으로 보간
        request.jump_threshold = 0.0     # 0 = 비활성화 (권장값)
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.get_logger().info(
            f"경로 계획 완료: fraction={response.fraction:.2f} "
            f"(1.0이면 요청한 경로 전체를 직선으로 계획 성공)"
        )
        if response.fraction < 0.9:
            self.get_logger().warn(
                "경로의 90% 미만만 계획됨 - 장애물이나 관절 한계에 걸렸을 수 있음"
            )
        return response.solution

    def execute(self, trajectory):
        if not self._exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/execute_trajectory 액션 서버를 찾을 수 없습니다.")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("실행 목표가 거부되었습니다.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("실행 성공!")
            return True
        else:
            self.get_logger().error(f"실행 실패: error_code={result.error_code.val}")
            return False


def main():
    rclpy.init()
    node = CartesianPathDemo()

    current_pose = node.get_current_pose()
    if current_pose is None:
        node.get_logger().error("현재 위치를 못 구해서 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"현재 위치: x={current_pose.position.x:.3f}, "
        f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}"
    )

    # 웨이포인트 구성: 현재 위치 -> 5cm 아래(approach) -> 다시 원위치(retreat)
    down_pose = copy.deepcopy(current_pose)
    down_pose.position.z -= DESCEND_DISTANCE

    up_pose = copy.deepcopy(current_pose)

    waypoints = [down_pose, up_pose]

    trajectory = node.plan_cartesian(waypoints)
    if trajectory is not None and len(trajectory.joint_trajectory.points) > 0:
        node.execute(trajectory)
    else:
        node.get_logger().error("유효한 궤적이 계획되지 않았습니다.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
