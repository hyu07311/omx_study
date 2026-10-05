#!/usr/bin/env python3
"""
OMY-F3M D405 카메라 기반 집기 데모
  1) 준비 자세에서 D405 로 테이블을 내려다봄
  2) 컬러 영상에서 목표 색(red / blue) 픽셀을 찾고, 같은 픽셀의 포인트클라우드 좌표를 모음
     (D405 는 컬러와 깊이가 같은 센서라 영상 픽셀 (u, v) = 포인트클라우드의 v 행 u 열)
  3) TF 로 link0 좌표로 바꿔서 윗면 중심과 yaw(윗면 점들의 주축 방향)를 계산
  4) omy_ik_demo 의 IK 로 집어서 옆으로 옮김

포인트클라우드의 rgb 필드는 Gazebo Classic 플러그인에서 R/B 가 뒤바뀌어 나오므로 색은 영상에서만 판단함

실행
  ros2 run omy_bringup omy_pick_demo.py                         # 빨간 큐브
  ros2 run omy_bringup omy_pick_demo.py --ros-args -p color:=blue
  ros2 run omy_bringup omy_pick_demo.py --ros-args -p save:=true   # 인식에 쓴 프레임 + detection.png 저장
"""
import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

from omy_ik_demo import GRIPPER_CLOSE
from omy_ik_demo import GRIPPER_OPEN
from omy_ik_demo import OMYDemo
from omy_ik_demo import READY_POSE
from save_d405 import new_capture_dir
from save_d405 import save_snapshot

# 영상에서 색 판정 (rgb8 기준)
COLOR_MASKS = {
    'red': lambda r, g, b: (r > 120) & (g < 60) & (b < 60),
    'blue': lambda r, g, b: (b > 120) & (r < 60) & (g < 60),
}
CUBE_SIZE = 0.04
# 윗면 기준으로 집는 높이 (end_effector_link). omy_ik_demo 의 GRASP_Z 와 같은 관계 (윗면 - 0.025)
GRASP_BELOW_TOP = 0.025
PLACE = (0.35, -0.20)


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class OMYPickDemo(OMYDemo):

    def __init__(self):
        super().__init__()
        self.declare_parameter('color', 'red')
        self.color = self.get_parameter('color').value
        self.declare_parameter('save', False)
        self.declare_parameter('output_dir', '~/robot_ws/d405_captures')
        self.save = self.get_parameter('save').value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.image = None
        self.cloud = None
        self.create_subscription(
            Image, '/d405/color/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/d405/color/points', self._on_cloud, qos_profile_sensor_data)

    def _on_image(self, msg):
        self.image = msg

    def _on_cloud(self, msg):
        self.cloud = msg

    def capture(self):
        """팔이 멈춘 뒤의 영상과 포인트클라우드를 새로 받음
        첫 프레임은 팔이 움직이는 중에 찍기 시작했을 수 있으므로 버리고 두 번째 프레임을 씀"""
        for _ in range(2):
            self.image = None
            self.cloud = None
            while rclpy.ok() and (self.image is None or self.cloud is None):
                rclpy.spin_once(self, timeout_sec=0.1)
        return self.image, self.cloud

    def detect(self):
        """목표 큐브 윗면 중심 (link0 기준) 과 yaw"""
        image, cloud = self.capture()
        rgb = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
        mask = COLOR_MASKS[self.color](
            rgb[:, :, 0].astype(int), rgb[:, :, 1].astype(int), rgb[:, :, 2].astype(int))
        if mask.sum() < 50:
            raise RuntimeError(f'{self.color} object not found in image ({mask.sum()} px)')

        xyz = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'])
        xyz = xyz.reshape(cloud.height, cloud.width, 3)[mask]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]

        # 포인트클라우드를 찍은 시각의 카메라 위치로 변환
        tf = self.tf_buffer.lookup_transform(
            'link0', cloud.header.frame_id, Time.from_msg(cloud.header.stamp),
            timeout=RclpyDuration(seconds=2.0))
        rot = quaternion_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        points = xyz @ rot.T + np.array([t.x, t.y, t.z])

        # 위에서 내려다보므로 대부분 윗면 점 → 가장 높은 면 근처만 사용
        top_z = np.percentile(points[:, 2], 95)
        top = points[points[:, 2] > top_z - 0.005]
        center = top.mean(axis=0)

        # yaw - 윗면 점들을 a, a+90° 방향으로 투영했을 때 폭이 가장 작은 각도 = 큐브 변 방향
        # (변 방향이면 폭 = 한 변 0.04 m, 대각선 방향이면 0.057 m). 정사각형이라 0 ~ 90° 만 보면 됨
        xy = top[:, :2] - center[:2]
        angles = np.radians(np.arange(0.0, 90.0, 1.0))
        widths = [max(np.ptp(xy @ [math.cos(a), math.sin(a)]),
                      np.ptp(xy @ [-math.sin(a), math.cos(a)])) for a in angles]
        yaw = float(angles[int(np.argmin(widths))])
        # 손목을 적게 돌리도록 -45° ~ 45° 로
        if yaw > math.pi / 4:
            yaw -= math.pi / 2

        self.get_logger().info(
            f'{self.color} cube: {mask.sum()} px, top center '
            f'({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}), yaw {math.degrees(yaw):.1f} deg')
        if self.save:
            self.save_detection(image, cloud, mask, center, yaw)
        return center, yaw

    def save_detection(self, image, cloud, mask, center, yaw):
        """인식에 쓴 프레임(color, depth, points) + 인식 결과를 그린 detection.png 저장"""
        path = new_capture_dir(self.get_parameter('output_dir').value)
        saved = save_snapshot(path, image, cloud=cloud)

        rgb = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        overlay = bgr.copy()
        overlay[mask] = (0, 255, 0)
        bgr = cv2.addWeighted(overlay, 0.5, bgr, 0.5, 0)
        vs, us = np.nonzero(mask)
        u, v = int(us.mean()), int(vs.mean())
        cv2.drawMarker(bgr, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.rectangle(bgr, (int(us.min()), int(vs.min())), (int(us.max()), int(vs.max())),
                      (255, 255, 255), 1)
        lines = [f'{self.color} cube  {mask.sum()} px',
                 f'top center (link0)  x {center[0]:.3f}  y {center[1]:.3f}  z {center[2]:.3f} m',
                 f'yaw {math.degrees(yaw):.1f} deg']
        for i, text in enumerate(lines):
            cv2.putText(bgr, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(bgr, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(path, 'detection.png'), bgr)
        self.get_logger().info(f'saved to {path}: detection.png, ' + ', '.join(saved))


def main():
    rclpy.init()
    demo = OMYPickDemo()

    demo.get_logger().info('1) look at the table')
    demo.gripper(GRIPPER_OPEN)
    demo.move_arm(READY_POSE, 3.0)

    demo.get_logger().info('2) detect with D405')
    (x, y, top_z), yaw = demo.detect()
    grasp_z = top_z - GRASP_BELOW_TOP

    demo.get_logger().info('3) pick and place')
    demo.move_to(x, y, grasp_z + 0.12, yaw=yaw)
    demo.move_to(x, y, grasp_z, yaw=yaw, seconds=1.5)
    demo.gripper(GRIPPER_CLOSE)
    demo.move_to(x, y, grasp_z + 0.12, yaw=yaw, seconds=1.5)
    demo.move_to(*PLACE, grasp_z + 0.12, yaw=math.pi / 2, seconds=3.0)
    demo.move_to(*PLACE, grasp_z + 0.005, yaw=math.pi / 2, seconds=1.5)
    demo.gripper(GRIPPER_OPEN)
    demo.move_to(*PLACE, grasp_z + 0.12, yaw=math.pi / 2, seconds=1.5)

    demo.get_logger().info('4) home')
    demo.move_arm(READY_POSE, 2.0)

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
