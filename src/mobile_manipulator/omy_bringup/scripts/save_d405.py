#!/usr/bin/env python3
"""
D405 카메라 영상 저장 (Gazebo, RViz mock 의 가짜 D405 둘 다)

  snapshot (기본) - 지금 프레임을 한 번 저장하고 종료
      ros2 run omy_bringup save_d405.py
  video          - Ctrl+C 까지 컬러/깊이 동영상 녹화
      ros2 run omy_bringup save_d405.py --ros-args -p mode:=video

저장 위치  ~/robot_ws/d405_captures/<날짜_시각>/   (-p output_dir:=... 로 변경)
  color.png         컬러 영상
  depth_mm.png      16비트 깊이 (mm, 0 = 측정 없음) - 숫자 그대로 다시 읽을 수 있음
  depth_vis.png     깊이를 색으로 표시 (가까울수록 빨강, 범위는 그 프레임의 최소~최대 깊이)
  points.ply        포인트클라우드 xyz + 색 (camera_depth_optical_frame 기준)
  camera_info.json  해상도, 내부 파라미터, 프레임, 시각
  (video) color.mp4, depth_vis.mp4

포인트클라우드는 영상과 같은 순서(organized)라서 색은 컬러 영상에서 가져옴
(Gazebo Classic 플러그인은 포인트클라우드 rgb 의 R/B 를 바꿔서 내보내기 때문)
"""
import datetime
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

DEPTH_RANGE = (0.07, 0.5)   # d405.xacro 의 min_depth, max_depth


# ===================== 변환 / 저장 함수 (omy_pick_demo.py 에서도 사용) =====================
def image_to_rgb(msg):
    """sensor_msgs/Image (rgb8, bgr8) → H x W x 3 RGB uint8"""
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1] if msg.encoding == 'bgr8' else img


def depth_to_meters(msg):
    """sensor_msgs/Image (32FC1 m, 16UC1 mm) → H x W float32 m (측정 없음 = nan)"""
    if msg.encoding == '16UC1':
        depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) / 1000.0
        return np.where(depth > 0, depth, np.nan).astype(np.float32)
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()


def cloud_to_xyz(msg):
    """organized PointCloud2 → H x W x 3 float32"""
    xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
    return xyz.reshape(msg.height, msg.width, 3)


def depth_range(depth):
    """보기용 색 범위 - 측정된 깊이의 1 ~ 99 % (값이 하나뿐이면 센서 범위)"""
    valid = depth[np.isfinite(depth) & (depth > 0)]
    if valid.size == 0:
        return DEPTH_RANGE
    near, far = np.percentile(valid, [1, 99])
    return (float(near), float(far)) if far - near > 1e-3 else DEPTH_RANGE


def depth_visual(depth, rng=None):
    """깊이(m) → 컬러맵 BGR 이미지 (가까울수록 빨강, 측정 없음 = 검정)"""
    near, far = rng if rng is not None else depth_range(depth)
    valid = np.isfinite(depth) & (depth > 0)
    scaled = np.zeros(depth.shape, np.uint8)
    scaled[valid] = (255 * (far - np.clip(depth[valid], near, far)) / (far - near)).astype(np.uint8)
    vis = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    vis[~valid] = 0
    return vis


def write_ply(path, xyz, rgb):
    """xyz (N x 3 float), rgb (N x 3 uint8) → binary PLY"""
    data = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                     ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    data['x'], data['y'], data['z'] = xyz.T
    data['red'], data['green'], data['blue'] = rgb.T
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(xyz)}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\n'
              'end_header\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data.tobytes())


def new_capture_dir(output_dir):
    path = os.path.join(os.path.expanduser(output_dir),
                        datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(path, exist_ok=True)
    return path


def save_snapshot(path, image, depth=None, cloud=None, info=None):
    """영상/깊이/포인트클라우드/카메라 정보를 path 폴더에 저장. 저장한 파일 이름 목록 반환"""
    saved = []
    rgb = image_to_rgb(image)
    cv2.imwrite(os.path.join(path, 'color.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    saved.append('color.png')

    depth_m = None
    if depth is not None:
        depth_m = depth_to_meters(depth)
    elif cloud is not None:
        depth_m = cloud_to_xyz(cloud)[:, :, 2]
    if depth_m is not None:
        mm = np.where(np.isfinite(depth_m), np.round(depth_m * 1000), 0)
        cv2.imwrite(os.path.join(path, 'depth_mm.png'), np.clip(mm, 0, 65535).astype(np.uint16))
        vis_range = depth_range(depth_m)
        cv2.imwrite(os.path.join(path, 'depth_vis.png'), depth_visual(depth_m, vis_range))
        saved += ['depth_mm.png', 'depth_vis.png']

    if cloud is not None:
        xyz = cloud_to_xyz(cloud).reshape(-1, 3)
        colors = rgb.reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        write_ply(os.path.join(path, 'points.ply'), xyz[valid], colors[valid])
        saved.append(f'points.ply ({valid.sum()} points)')

    meta = {
        'frame_id': image.header.frame_id,
        'stamp': image.header.stamp.sec + image.header.stamp.nanosec * 1e-9,
        'width': image.width,
        'height': image.height,
        'depth_png_unit': 'mm (0 = no data)',
        'sensor_depth_range_m': list(DEPTH_RANGE),
    }
    if depth_m is not None:
        meta['depth_vis_range_m'] = [round(v, 4) for v in vis_range]
    if info is not None:
        meta.update({'fx': info.k[0], 'fy': info.k[4], 'cx': info.k[2], 'cy': info.k[5],
                     'distortion_model': info.distortion_model, 'd': list(info.d)})
    with open(os.path.join(path, 'camera_info.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    saved.append('camera_info.json')
    return saved


# ===================== 노드 =====================
class SaveD405(Node):

    def __init__(self):
        super().__init__('save_d405')
        self.declare_parameter('mode', 'snapshot')
        self.declare_parameter('output_dir', '~/robot_ws/d405_captures')
        self.mode = self.get_parameter('mode').value
        self.output_dir = self.get_parameter('output_dir').value
        if self.mode not in ('snapshot', 'video'):
            raise ValueError("mode must be 'snapshot' or 'video'")

        self.image = None
        self.depth = None
        self.cloud = None
        self.info = None
        self.done = False
        self.create_subscription(Image, '/d405/color/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/d405/color/depth/image_raw', self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/d405/color/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        if self.mode == 'snapshot':
            self.create_subscription(PointCloud2, '/d405/color/points', self._on_cloud,
                                     qos_profile_sensor_data)
        else:
            self.path = new_capture_dir(self.output_dir)
            self.writers = {}
            self.buffer = []          # fps 를 재기 위해 처음 몇 프레임을 모아 둠
            self.frames = 0
        self.get_logger().info(f'{self.mode}: waiting for /d405 topics ...')

    def _on_info(self, msg):
        self.info = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_cloud(self, msg):
        self.cloud = msg
        self._try_snapshot()

    def _on_image(self, msg):
        self.image = msg
        if self.mode == 'snapshot':
            self._try_snapshot()
        else:
            self._add_video_frame(msg)

    def _try_snapshot(self):
        if self.done or self.image is None or self.cloud is None or self.depth is None:
            return
        path = new_capture_dir(self.output_dir)
        saved = save_snapshot(path, self.image, self.depth, self.cloud, self.info)
        self.get_logger().info(f'saved to {path}: ' + ', '.join(saved))
        self.done = True

    def _add_video_frame(self, msg):
        color = cv2.cvtColor(image_to_rgb(msg), cv2.COLOR_RGB2BGR)
        if self.depth is not None:
            depth_m = depth_to_meters(self.depth)
            # 동영상은 색이 프레임마다 바뀌지 않도록 첫 프레임의 범위로 고정
            if getattr(self, 'vis_range', None) is None:
                self.vis_range = depth_range(depth_m)
            depth = depth_visual(depth_m, self.vis_range)
        else:
            depth = np.zeros_like(color)
        if not self.writers:
            self.buffer.append((time.monotonic(), color, depth))
            if len(self.buffer) < 6:
                return
            fps = (len(self.buffer) - 1) / (self.buffer[-1][0] - self.buffer[0][0])
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            size = (msg.width, msg.height)
            self.writers = {
                'color': cv2.VideoWriter(os.path.join(self.path, 'color.mp4'), fourcc, fps, size),
                'depth': cv2.VideoWriter(os.path.join(self.path, 'depth_vis.mp4'), fourcc, fps, size),
            }
            self.get_logger().info(f'recording {fps:.1f} fps to {self.path} (Ctrl+C to stop)')
            frames = [(c, d) for _, c, d in self.buffer]
            self.buffer = []
        else:
            frames = [(color, depth)]
        for c, d in frames:
            self.writers['color'].write(c)
            self.writers['depth'].write(d)
            self.frames += 1

    def close(self):
        if self.mode == 'video' and self.writers:
            for w in self.writers.values():
                w.release()
            print(f'saved {self.frames} frames to {self.path} (color.mp4, depth_vis.mp4)')


def main():
    rclpy.init()
    node = SaveD405()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
