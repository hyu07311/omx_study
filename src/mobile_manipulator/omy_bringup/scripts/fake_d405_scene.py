#!/usr/bin/env python3
"""
Gazebo 없이 RViz(mock) 모드에서 쓰는 가짜 D405 + 간단한 장면

  장면    table.world 와 같은 바닥, 테이블, 빨간/파란 큐브 → /scene_markers (RViz MarkerArray)
  카메라  camera_depth_optical_frame 의 TF 에서 픽셀마다 광선을 쏴서(ray casting) 상자와 만나는 점을 계산
          → Gazebo 의 D405 플러그인과 같은 토픽으로 발행
            /d405/color/image_raw (rgb8), /d405/color/depth/image_raw (32FC1, m),
            /d405/color/points (영상과 같은 순서의 organized 포인트클라우드), camera_info
  집기    그리퍼(rh_r1_joint)가 닫혔을 때 손끝(end_effector_link) 근처에 큐브가 있으면 손에 붙이고,
          그리퍼가 열리면 그 자리 아래(테이블 또는 바닥) 위에 내려놓음 (물리 없음, 판정만)
  초기화  ros2 service call /reset_scene std_srvs/srv/Trigger

한계
  - 로봇 자신(손가락 등)은 카메라에 찍히지 않음
  - 물리가 없으므로 미끄러짐, 쥐는 힘, 큐브끼리의 충돌은 없음
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

WORLD = 'world'
CAMERA_FRAME = 'camera_depth_optical_frame'
TIP_FRAME = 'end_effector_link'

# d405.xacro 와 같은 값
HFOV = math.radians(87.0)
MIN_DEPTH, MAX_DEPTH = 0.07, 0.5

# 그리퍼 판정 (rh_r1_joint, 0 = 열림, 1.135 = 닫힘)
GRIP_CLOSED = 0.5
GRIP_OPEN = 0.3
GRASP_RADIUS = 0.03   # 손끝 기준점과 큐브 중심 거리가 이 안이면 집힘

TABLE_TOP = 0.10


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


class Box:
    """회전된 직육면체 (center, 회전행렬 rot, 크기 size, 색 rgb 0~1)"""

    def __init__(self, name, center, size, rgb, yaw=0.0, movable=False):
        self.name = name
        self.size = np.array(size, dtype=float)
        self.rgb = np.array(rgb, dtype=float)
        self.movable = movable
        self.initial = (np.array(center, dtype=float), yaw)
        self.reset()

    def reset(self):
        self.center = self.initial[0].copy()
        self.rot = yaw_matrix(self.initial[1])

    def intersect(self, origin, dirs):
        """slab 방법. 광선 파라미터 t (없으면 inf) 와 바깥 법선(월드)"""
        half = self.size / 2
        o = self.rot.T @ (origin - self.center)
        d = dirs @ self.rot                     # 각 광선 방향을 상자 좌표로
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (-half - o) / d
            t2 = (half - o) / d
        t_near = np.minimum(t1, t2)
        t_far = np.maximum(t1, t2)
        t_near = np.where(np.isnan(t_near), -np.inf, t_near)
        t_far = np.where(np.isnan(t_far), np.inf, t_far)
        t_enter = t_near.max(axis=1)
        t_exit = t_far.min(axis=1)
        hit = (t_exit >= t_enter) & (t_enter > 0)
        t = np.where(hit, t_enter, np.inf)
        axis = t_near.argmax(axis=1)
        sign = -np.sign(d[np.arange(len(d)), axis])
        normals = self.rot[:, axis].T * sign[:, None]
        return t, normals


class FakeD405Scene(Node):

    def __init__(self):
        super().__init__('fake_d405_scene')
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('rate', 5.0)
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # table.world 와 같은 배치
        self.boxes = [
            Box('table', [0.45, 0.0, 0.05], [0.4, 0.6, 0.1], [0.55, 0.38, 0.22]),
            Box('red_cube', [0.45, 0.0, 0.12], [0.04, 0.04, 0.04], [0.85, 0.05, 0.05], movable=True),
            Box('blue_cube', [0.38, 0.18, 0.12], [0.04, 0.04, 0.04], [0.05, 0.1, 0.85],
                yaw=0.5, movable=True),
        ]
        self.ground_rgb = np.array([0.6, 0.6, 0.6])
        self.sky_rgb = np.array([0.7, 0.75, 0.8])
        self.light = np.array([0.3, 0.2, 1.0]) / np.linalg.norm([0.3, 0.2, 1.0])

        # 픽셀 광선 (optical 좌표, z = 1 이므로 광선 파라미터 t = 깊이)
        self.fx = self.width / 2 / math.tan(HFOV / 2)
        self.fy = self.fx
        self.cx, self.cy = (self.width - 1) / 2, (self.height - 1) / 2
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.rays = np.stack([(u - self.cx) / self.fx, (v - self.cy) / self.fy,
                              np.ones_like(u, dtype=float)], axis=-1).reshape(-1, 3)

        self.gripper = 0.0
        self.held = None          # (Box, 손끝 기준 상대 변환 4x4)
        self.lock = threading.Lock()

        # 렌더링(약 0.2 s)이 TF / joint_states 처리를 막지 않도록
        #   TF 는 별도 스레드, 카메라 타이머는 별도 콜백 그룹 (MultiThreadedExecutor 로 실행)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        camera_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.image_pub = self.create_publisher(Image, '/d405/color/image_raw', 2)
        self.depth_pub = self.create_publisher(Image, '/d405/color/depth/image_raw', 2)
        self.points_pub = self.create_publisher(PointCloud2, '/d405/color/points', 2)
        self.info_pub = self.create_publisher(CameraInfo, '/d405/color/camera_info', 2)
        self.marker_pub = self.create_publisher(MarkerArray, '/scene_markers', 2)
        self.create_service(Trigger, '/reset_scene', self._on_reset)

        self.create_timer(0.05, self._update_grasp)
        self.create_timer(0.2, self._publish_markers)
        self.create_timer(1.0 / self.get_parameter('rate').value, self._publish_camera,
                          callback_group=camera_group)
        self.get_logger().info(
            f'fake D405 {self.width}x{self.height}, fx {self.fx:.1f}, depth {MIN_DEPTH}~{MAX_DEPTH} m')

    # ---------------- TF ----------------
    def _lookup(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(WORLD, frame, Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        m = np.eye(4)
        m[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        m[:3, 3] = [t.x, t.y, t.z]
        return m, tf.header.stamp

    # ---------------- 집기 판정 ----------------
    def _on_joint_states(self, msg):
        if 'rh_r1_joint' not in msg.name:
            return
        previous = self.gripper
        self.gripper = msg.position[msg.name.index('rh_r1_joint')]
        # 닫히는/열리는 순간 바로 판정 (mock 에서는 그리퍼가 닫히자마자 팔이 움직이기 시작함)
        if (previous <= GRIP_CLOSED < self.gripper) or (previous >= GRIP_OPEN > self.gripper):
            self._update_grasp()

    def _update_grasp(self):
        with self.lock:
            self._update_grasp_locked()

    def _update_grasp_locked(self):
        tip = self._lookup(TIP_FRAME)
        if tip is None:
            return
        tip = tip[0]
        if self.held is None and self.gripper > GRIP_CLOSED:
            for box in self.boxes:
                if box.movable and np.linalg.norm(box.center - tip[:3, 3]) < GRASP_RADIUS:
                    pose = np.eye(4)
                    pose[:3, :3] = box.rot
                    pose[:3, 3] = box.center
                    self.held = (box, np.linalg.inv(tip) @ pose)
                    self.get_logger().info(f'grasped {box.name}')
                    break
        elif self.held is not None and self.gripper < GRIP_OPEN:
            box = self.held[0]
            self.held = None
            # 아래 지지면 위에 똑바로 내려놓음 (yaw 만 유지)
            on_table = abs(box.center[0] - 0.45) < 0.2 and abs(box.center[1]) < 0.3
            box.center[2] = (TABLE_TOP if on_table else 0.0) + box.size[2] / 2
            box.rot = yaw_matrix(math.atan2(box.rot[1, 0], box.rot[0, 0]))
            self.get_logger().info(
                f'released {box.name} at ({box.center[0]:.3f}, {box.center[1]:.3f}, {box.center[2]:.3f})')
        if self.held is not None:
            box, rel = self.held
            pose = tip @ rel
            box.rot = pose[:3, :3]
            box.center = pose[:3, 3]

    def _on_reset(self, request, response):
        with self.lock:
            self.held = None
            for box in self.boxes:
                box.reset()
        response.success = True
        response.message = 'scene reset'
        return response

    # ---------------- 출력 ----------------
    def _publish_markers(self):
        markers = MarkerArray()
        for i, box in enumerate(self.boxes):
            m = Marker()
            m.header.frame_id = WORLD
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'scene'
            m.id = i
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = box.center.tolist()
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = matrix_to_quaternion(box.rot)
            m.scale.x, m.scale.y, m.scale.z = box.size.tolist()
            m.color.r, m.color.g, m.color.b = box.rgb.tolist()
            m.color.a = 1.0
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _publish_camera(self):
        cam = self._lookup(CAMERA_FRAME)
        if cam is None:
            return
        cam, stamp = cam
        origin = cam[:3, 3]
        dirs = self.rays @ cam[:3, :3].T            # 월드 방향 (정규화 안 함, t = 깊이)

        n = len(dirs)
        t_best = np.full(n, np.inf)
        color = np.tile(self.sky_rgb, (n, 1))
        # 바닥 z = 0
        with np.errstate(divide='ignore', invalid='ignore'):
            t_ground = np.where(dirs[:, 2] < 0, -origin[2] / dirs[:, 2], np.inf)
        hit = t_ground < t_best
        t_best[hit] = t_ground[hit]
        color[hit] = self.ground_rgb * (0.4 + 0.6 * self.light[2])
        with self.lock:
            boxes = [Box(b.name, b.center.copy(), b.size, b.rgb) for b in self.boxes]
            for copy, b in zip(boxes, self.boxes):
                copy.rot = b.rot.copy()
        for box in boxes:
            t, normals = box.intersect(origin, dirs)
            hit = t < t_best
            t_best[hit] = t[hit]
            shade = 0.4 + 0.6 * np.clip(normals[hit] @ self.light, 0.0, 1.0)
            color[hit] = box.rgb * shade[:, None]

        depth = np.where((t_best >= MIN_DEPTH) & (t_best <= MAX_DEPTH), t_best, np.nan)
        points = (self.rays * depth[:, None]).astype(np.float32)
        rgb8 = (np.clip(color, 0, 1) * 255).astype(np.uint8)

        image = Image(height=self.height, width=self.width, encoding='rgb8',
                      step=self.width * 3, data=rgb8.tobytes())
        image.header.stamp = stamp
        image.header.frame_id = CAMERA_FRAME
        depth_msg = Image(height=self.height, width=self.width, encoding='32FC1',
                          step=self.width * 4, data=depth.astype(np.float32).tobytes())
        depth_msg.header = image.header

        # x, y, z (float32) + rgb (float32 자리에 0x00RRGGBB)
        packed = (rgb8[:, 0].astype(np.uint32) << 16) | (rgb8[:, 1].astype(np.uint32) << 8) \
            | rgb8[:, 2].astype(np.uint32)
        cloud_data = np.empty(n, dtype=[('x', np.float32), ('y', np.float32),
                                        ('z', np.float32), ('rgb', np.uint32)])
        cloud_data['x'], cloud_data['y'], cloud_data['z'] = points.T
        cloud_data['rgb'] = packed
        cloud = PointCloud2(
            height=self.height, width=self.width, is_dense=False, is_bigendian=False,
            point_step=16, row_step=16 * self.width, data=cloud_data.tobytes(),
            fields=[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)])
        cloud.header = image.header

        info = CameraInfo(height=self.height, width=self.width, distortion_model='plumb_bob',
                          d=[0.0] * 5,
                          k=[self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0],
                          r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                          p=[self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0,
                             0.0, 0.0, 1.0, 0.0])
        info.header = image.header

        self.image_pub.publish(image)
        self.depth_pub.publish(depth_msg)
        self.points_pub.publish(cloud)
        self.info_pub.publish(info)


def main():
    rclpy.init()
    node = FakeD405Scene()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
