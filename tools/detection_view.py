#!/usr/bin/env python3
"""작업자 검출 + ArUco 를 **한 창**으로 — amr_detection 의 오버레이 위에 마커를 그려 넣는다.

    python3 tools/detection_view.py --ns /robot3

무엇을 보나
  · 왼쪽: amr_detection 이 낸 RGB + 작업자 bbox (target/image/compressed 그대로)
  · 그 위에 aruco_detector 가 낸 마커 — aruco/pose(tvec·rvec)를 카메라 행렬로 다시 투영해
    네 모서리를 그린다. **검출기가 실제로 발행한 값**을 그리는 것이라, 보이면 노드가 본 것이다
  · 상단 줄: aruco/id · aruco/distance (id -1 이면 빨강 '마커 없음')
  · 오른쪽: depth 컬러맵 (amr_detection 오버레이의 오른쪽 절반)

왜 카메라를 직접 구독하지 않나 — 검출기 둘이 이미 RPi 의 카메라 스트림을 당기고 있다.
디스커버리 서버 구성에서는 구독자마다 RPi 가 따로 보내므로 셋째 구독자는 무선 부하다.
이 뷰어는 PC 에서 나는 오버레이(JPEG)만 받는다 — 무선에 아무 부담이 없다.

follow_rig 의 자체 창과 겹치지 않게 하려면:
    ros2 launch mro_navigation follow_rig.launch.py namespace:=/robot3 show_window:=false

키  q / ESC 끝내기
"""
import argparse
import math
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import PoseStamped

from mro_perception.core.aruco import object_points
from mro_perception.core.camera import corrected_K, read_fx_scale

GREEN, RED, YELLOW, WHITE = (0, 200, 0), (0, 0, 230), (0, 220, 255), (255, 255, 255)


def yaml_fx_scale():
    """perception.yaml 의 camera_fx_scale — 검출기가 쓴 값과 같아야 투영이 맞는다. 못 읽으면 None."""
    try:
        import yaml
        from pathlib import Path
        p = Path(__file__).resolve().parents[1] / 'ros2_ws/src/mro_perception/config/perception.yaml'
        return float(yaml.safe_load(p.read_text(encoding='utf-8'))['/**']['ros__parameters']['camera_fx_scale'])
    except Exception:
        return None


def quat_to_rvec(q):
    """PoseStamped 의 쿼터니언 → OpenCV rvec (aruco.rvec_to_quat 의 역)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(R)
    return rvec


class DetectionView(Node):
    def __init__(self, ns, size_m, stale_s, fx_scale):
        super().__init__('detection_view')
        self.size, self.stale_s = size_m, stale_s
        # 검출기와 같은 배율이어야 투영이 제자리에 떨어진다. perception.yaml 값을 기본으로 넘긴다
        self.fx_scale = fx_scale if fx_scale else read_fx_scale(self)
        self.K = self.D = None
        self.frame = None
        self.aid, self.adist, self.pose = -1, float('nan'), None
        self.pose_at = self.id_at = 0.0
        self.create_subscription(CameraInfo, f'{ns}/oakd/rgb/camera_info', self.info_cb, 10)
        self.create_subscription(CompressedImage, f'{ns}/target/image/compressed', self.img_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Int32, f'{ns}/aruco/id', self.id_cb, 10)
        self.create_subscription(Float32, f'{ns}/aruco/distance', self.dist_cb, 10)
        self.create_subscription(PoseStamped, f'{ns}/aruco/pose', self.pose_cb, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_cb(self, msg):
        if self.K is None:
            self.K = corrected_K(msg, self.fx_scale)
            d = np.array(msg.d, dtype=np.float64)
            self.D = d.reshape(-1, 1) if d.size else np.zeros((5, 1))

    def img_cb(self, msg):
        f = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if f is not None:
            self.frame = f

    def id_cb(self, msg):
        self.aid, self.id_at = msg.data, self.now()

    def dist_cb(self, msg):
        self.adist = msg.data

    def pose_cb(self, msg):
        self.pose, self.pose_at = msg.pose, self.now()

    def draw(self):
        view = self.frame.copy()
        h, w = view.shape[:2]
        half = w // 2 if w >= 2 * h else w          # 오버레이는 좌 RGB · 우 depth 가 가로로 붙어 있다
        now = self.now()
        fresh_id = now - self.id_at < self.stale_s
        fresh_pose = self.pose is not None and now - self.pose_at < self.stale_s
        if fresh_pose and self.K is not None and self.aid != -1:
            rvec = quat_to_rvec(self.pose.orientation)
            tvec = np.array([[self.pose.position.x], [self.pose.position.y], [self.pose.position.z]])
            pts, _ = cv2.projectPoints(object_points(self.size), rvec, tvec, self.K, self.D)
            pts = pts.reshape(-1, 2)
            if np.all(np.isfinite(pts)) and np.all(np.abs(pts) < 1e4):
                poly = pts.astype(np.int32)
                cv2.polylines(view, [poly], True, GREEN, 3)
                cx, cy = poly.mean(axis=0).astype(int)
                cv2.putText(view, f'ID {self.aid}', (cx - 30, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)
        if fresh_id and self.aid != -1:
            txt, col = f'aruco/id {self.aid}  {self.adist:.2f} m', GREEN
        elif fresh_id:
            txt, col = 'aruco/id -1  (마커 없음)', RED
        else:
            txt, col = 'aruco_detector 발행 없음 — T5 에 real_aruco:=true ?', YELLOW
        cv2.rectangle(view, (0, 0), (half, 34), (0, 0, 0), -1)
        cv2.putText(view, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        if self.K is None:
            cv2.putText(view, 'camera_info 대기', (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)
        return view


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ns', default='/robot3')
    ap.add_argument('--size', type=float, default=0.10, help='마커 한 변 [m]')
    ap.add_argument('--stale', type=float, default=1.0, help='이 시간 지난 aruco 값은 안 그린다 [s]')
    ap.add_argument('--scale', type=float, default=yaml_fx_scale(), help='camera_fx_scale (기본: perception.yaml)')
    a = ap.parse_args()
    rclpy.init()
    n = DetectionView(a.ns.rstrip('/'), a.size, a.stale, a.scale)
    win = f'MRO Detection {a.ns}'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.03)
            if n.frame is not None:
                cv2.imshow(win, n.draw())
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
