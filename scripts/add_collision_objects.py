#!/usr/bin/env python3
"""
MoveIt의 Planning Scene에 충돌 객체(테이블 + 작은 물체)를 추가하는 예제.
moveit_py 없이 /apply_planning_scene 서비스를 직접 호출한다.

사전 준비: move_group이 떠 있어야 한다.
  ros2 launch open_manipulator_x_bringup hardware.launch.py use_fake_hardware:=true
  ros2 launch open_manipulator_x_moveit_config move_group.launch.py
  ros2 launch open_manipulator_x_moveit_config moveit_rviz.launch.py

실행:
  python3 add_collision_objects.py
"""

import sys

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# 로봇 베이스 기준 좌표계 이름 (URDF에 따라 다를 수 있음 - 확인 필요)
FRAME_ID = "world"


def make_box(object_id, frame_id, size, position):
    """size=(x,y,z) 크기, position=(x,y,z) 중심 좌표로 박스 CollisionObject 생성"""
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame_id

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class SceneEditor(Node):
    def __init__(self):
        super().__init__("scene_editor")
        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def apply(self, objects):
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/apply_planning_scene 서비스를 찾을 수 없습니다.")
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result is not None and result.success:
            self.get_logger().info(f"{len(objects)}개 객체 추가 성공")
            return True
        else:
            self.get_logger().error("Planning Scene 적용 실패")
            return False


def main():
    rclpy.init()
    node = SceneEditor()

    # 1) 테이블 (팔 앞쪽, 바닥보다 살짝 위)
    table = make_box(
        object_id="table",
        frame_id=FRAME_ID,
        size=(0.5, 0.5, 0.02),      # 가로 50cm x 세로 50cm x 두께 2cm
        position=(0.25, 0.0, 0.0),  # 팔 베이스에서 앞으로 25cm, 바닥 높이(0)
    )

    # 2) 집을 대상 물체 (테이블 위 작은 박스)
    target_object = make_box(
        object_id="target_object",
        frame_id=FRAME_ID,
        size=(0.03, 0.03, 0.03),
        position=(0.25, 0.0, 0.03),  # 테이블 표면 위
    )

    try:
        node.apply([table, target_object])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
