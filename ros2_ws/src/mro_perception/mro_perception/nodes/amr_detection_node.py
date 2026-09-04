#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/amr_detection_node.py → ros2_ws/src/mro_perception/mro_perception/nodes/amr_detection_node.py
변경: sys.path 해킹 제거 → 패키지 import, 모델 경로 → share 디렉토리.
TODO([인지]): DETECT_CLASS 를 worker_a/worker_b 로, 웹캠1 서술자와 대조하는 재식별 신뢰도 게이트 추가 → target/paired, target/worker_id 발행 (BR-T16).
TODO: 모듈 상수(DETECT_CONF, MAX_CORE_GAP_MM 등)를 ROS2 파라미터로 (config/perception.yaml).
"""
"""
amr_detection_node.py — [AMR Detection]
  OAK-D 동기화 RGB/depth → YOLO car → bbox 안쪽 ROI 보정 거리 → 역투영 → TF → map
  → /robot3/target/map_point (PointStamped, frame=map)  품질 통과한 검출마다
  → /robot3/target/visible   (Bool)  프레임마다: car bbox 가 보이는지 (품질 무관)
  → /robot3/target/image     (Image bgr8)  오버레이 (rqt_image_view 로 확인)

  /robot3/goto_goal/arrived (Bool true) 를 받으면 검출 시작. AUTO_START=True 면 처음부터.

실행: python3 amr_detection_node.py --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static
"""
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from mro_perception.core.depth_distance import (DEPTH_TRANSPORT, colorize_depth, decode_compressed_depth,  # noqa: E402
                            distance_from_bbox, format_result)
from mro_perception.core.target_utils import back_project, to_map  # noqa: E402

ROBOT_NS = '/robot3'
MODEL_PATH = os.path.join(get_package_share_directory('mro_perception'), 'models', 'my_best.pt')  # TODO: worker.pt 로 교체
START_TOPIC = 'goto_goal/arrived'
POINT_TOPIC = 'target/map_point'
IMAGE_TOPIC = 'target/image'
VISIBLE_TOPIC = 'target/visible'
AUTO_START = False           # True: arrived 신호 없이 바로 검출 (단독 테스트용)
DETECT_CLASS = 'car'
DETECT_CONF = 0.5
ONLY_OUR_CAR = True          # 흰 차만 (2_4_g 색 필터)
CAR_S_MAX, CAR_V_MIN = 100, 95
BBOX_INNER = 0.5
# 품질 게이트: ROI 핵심부(p20~median) 가 이 이상 벌어지면 거리값 불신.
# spread(max-min) 는 bbox 에 뒤쪽 바닥/벽이 들어오면 항상 1 m 넘게 나와 게이트로 부적합 (실측 로그 참고)
MAX_CORE_GAP_MM = 150
SYNC_SLOP_S = 0.1
SHOW_WINDOW = True


def is_our_car(img, x1, y1, x2, y2):
    crop = img[max(0, y1):y2, max(0, x1):x2]
    if crop.size == 0:
        return False
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    return hsv[..., 1].mean() < CAR_S_MAX and hsv[..., 2].mean() > CAR_V_MIN


def best_box(results, names, cls_name, conf_min, img, color_filter):
    best = None
    for r in results:
        for b in r.boxes:
            if names.get(int(b.cls[0])) != cls_name or float(b.conf[0]) < conf_min:
                continue
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            if color_filter and not is_our_car(img, x1, y1, x2, y2):
                continue
            c = float(b.conf[0])
            if best is None or c > best[4]:
                best = (x1, y1, x2, y2, c)
    return best


class AmrDetection(Node):
    def __init__(self):
        super().__init__('amr_detection')
        ns = self.get_namespace()
        self.ns = ROBOT_NS if ns == '/' else ns
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.names = self.model.names
        self.K = None
        self.pair = None
        self.enabled = AUTO_START

        rgb_topic = f'{self.ns}/oakd/rgb/image_raw/compressed'
        if DEPTH_TRANSPORT == 'compressedDepth':
            depth_sub = Subscriber(self, CompressedImage, f'{self.ns}/oakd/stereo/image_raw/compressedDepth')
        else:
            depth_sub = Subscriber(self, Image, f'{self.ns}/oakd/stereo/image_raw')
        rgb_sub = Subscriber(self, CompressedImage, rgb_topic)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=SYNC_SLOP_S)
        self.sync.registerCallback(self.synced_cb)
        self.create_subscription(CameraInfo, f'{self.ns}/oakd/stereo/camera_info', self.info_cb, 1)
        self.create_subscription(Bool, START_TOPIC, self.start_cb, 1)
        self.pub_point = self.create_publisher(PointStamped, POINT_TOPIC, 1)
        self.pub_image = self.create_publisher(Image, IMAGE_TOPIC, 1)
        self.pub_visible = self.create_publisher(Bool, VISIBLE_TOPIC, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if SHOW_WINDOW:
            cv2.namedWindow('AMR Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AMR Detection', 1408, 704)
        self.get_logger().info(f"YOLO {MODEL_PATH} | 시작 신호: {self.ns}/{START_TOPIC} "
                               f"(AUTO_START={AUTO_START}) | 출력: {self.ns}/{POINT_TOPIC}")

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def start_cb(self, msg):
        if msg.data and not self.enabled:
            self.enabled = True
            self.get_logger().info('검출 시작')

    def synced_cb(self, rgb_msg, depth_msg):
        depth = (decode_compressed_depth(depth_msg) if isinstance(depth_msg, CompressedImage)
                 else self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))
        rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if depth is not None and rgb is not None:
            self.pair = (rgb, depth, depth_msg.header.stamp, depth_msg.header.frame_id)

    def process(self):
        """메인 루프에서 호출. 최신 쌍 하나를 처리하고 소비."""
        if self.pair is None:
            return
        rgb, depth, stamp, frame_id = self.pair
        self.pair = None
        vis = rgb.copy()
        label = 'waiting for start signal' if not self.enabled else 'detecting'

        if self.enabled and self.K is not None:
            box = best_box(self.model(rgb, verbose=False), self.names, DETECT_CLASS, DETECT_CONF,
                           rgb, ONLY_OUR_CAR)
            self.pub_visible.publish(Bool(data=box is not None))
            if box:
                x1, y1, x2, y2, c = box
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                r = distance_from_bbox(depth, x1, y1, x2, y2, inner_ratio=BBOX_INNER)
                self.get_logger().info(f'bbox conf={c:.2f} {format_result(r)}')
                if r is not None and (r['med'] - r['q']) <= MAX_CORE_GAP_MM:
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    pm = to_map(self.tf_buffer, back_project(self.K, u, v, r['z_corr'], frame_id, stamp),
                                self.get_logger())
                    if pm is not None:
                        self.pub_point.publish(pm)
                        label = f"car {r['z_corr']:.2f} m -> map ({pm.point.x:.2f}, {pm.point.y:.2f})"
                elif r is not None:
                    label = f"core gap {r['med'] - r['q']:.0f} mm > {MAX_CORE_GAP_MM} - rejected"
                cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                label = 'detecting - no car'

        frame = np.hstack((vis, colorize_depth(depth)))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(frame, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        if SHOW_WINDOW:
            cv2.imshow('AMR Detection', frame)


def main():
    rclpy.init()
    node = AmrDetection()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process()
            if SHOW_WINDOW and (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()      # Ctrl+C 시 이미 내려간 경우 중복 호출 방지
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
