#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/amr_detection_node.py → ros2_ws/src/mro_perception/mro_perception/nodes/amr_detection_node.py

amr_detection_node.py — [AMR Detection]
  OAK-D 동기화 RGB/depth → YOLO worker_a/worker_b → bbox 안쪽 ROI 보정 거리 → 역투영 → TF → map
  → target/map_point (PointStamped, frame=map)  품질 통과한 검출마다
  → target/visible   (Bool)  프레임마다: 작업자 bbox 가 보이는지 (품질 무관)
  → target/image     (Image bgr8)  오버레이 (rqt_image_view 로 확인)

  시작 신호는 없다 — 켜지면 곧바로 검출·발행한다. 결과를 언제 쓸지는 follow_target 이 shuttle/arrived 로 정한다
  (계약 v1.10. 옛 goto_goal/arrived 게이트는 interfaces.md '쓰지 않는 옛 토픽' 에 따라 제거).
  토픽은 전부 상대 이름이고 launch 가 namespace:=/robotN 을 붙인다 (robot.launch.py).

실행: ros2 launch mro_bringup robot.launch.py
단독: ros2 run mro_perception amr_detection_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static

TODO([인지]): 웹캠1 서술자와 대조하는 재식별 신뢰도 게이트 추가 → target/paired, target/worker_id 발행 (BR-T16).
TODO: 모듈 상수(DETECT_CONF, MAX_CORE_GAP_MM 등)를 ROS2 파라미터로 (config/perception.yaml).
"""
import os

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

from mro_perception.core.depth_distance import (DEPTH_TRANSPORT, colorize_depth, decode_compressed_depth,
                            distance_from_bbox, format_result)
from mro_perception.core.target_utils import back_project, to_map

MODEL_PATH = os.path.join(get_package_share_directory('mro_perception'), 'models', 'worker.pt')
# 출력 토픽 3개 (계약 interfaces.md 6절). 받는 쪽은 follow_target(추종)과 kpi_logger(K-04 Lost 집계)
POINT_TOPIC = 'target/map_point'     # 작업자 위치, map 좌표 — 거리 품질 게이트를 통과한 검출만
IMAGE_TOPIC = 'target/image'         # 모니터링용 오버레이 영상 (좌: RGB+bbox, 우: depth 컬러맵)
VISIBLE_TOPIC = 'target/visible'     # 프레임마다 True/False — 작업자가 화면에 잡히는지 (거리 품질과 무관)
# 클래스는 반드시 이름으로 거른다 — worker.pt 의 인덱스는 데이터셋 알파벳순이라 amr=0, worker_b=1, worker_a=2 다
# (models/README.md). 인덱스로 매핑하면 검출은 정상으로 보이고 페어링만 전부 틀리는 형태로 나타난다
DETECT_CLASSES = ('worker_a', 'worker_b')   # amr(상대 로봇)은 네거티브 클래스라 추종 후보에서 뺀다
DETECT_CONF = 0.5            # config/perception.yaml detect_conf 가 단일 출처 (파라미터 전환은 9/8)
BBOX_INNER = 0.5             # 거리 측정에 쓰는 bbox 안쪽 비율. 가장자리는 배경(바닥·벽) depth 가 섞여서 중심부 50 % 만 본다
# 품질 게이트: ROI 핵심부(p20~median) 가 이 이상 벌어지면 거리값 불신.
# spread(max-min) 는 bbox 에 뒤쪽 바닥/벽이 들어오면 항상 1 m 넘게 나와 게이트로 부적합 (실측 로그 참고)
MAX_CORE_GAP_MM = 150
SYNC_SLOP_S = 0.1            # RGB 와 depth 프레임의 타임스탬프 차이 허용치(초). 이 안이면 같은 순간으로 본다
SHOW_WINDOW = True           # 로컬 미리보기 창. 로봇 PC 에서 GUI 없이 돌릴 때는 False (파라미터 전환은 9/8)


def best_box(results, names, cls_names, conf_min):
    """가장 confident 한 검출 하나를 고른다.

    클래스 이름을 함께 반환한다 — 재식별 게이트(BR-T16)가 outfit_class 로 쓸 값이고
    오버레이 라벨에도 필요하다.
    """
    best = None
    for r in results:
        for b in r.boxes:
            name = names.get(int(b.cls[0]))                 # 번호 → 이름 (amr / worker_b / worker_a)
            if name not in cls_names or float(b.conf[0]) < conf_min:   # 작업자가 아니거나 확신이 낮으면 버린다
                continue
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            c = float(b.conf[0])
            if best is None or c > best[4]:
                best = (x1, y1, x2, y2, c, name)
    return best


class AmrDetection(Node):
    def __init__(self):
        super().__init__('amr_detection')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.names = self.model.names        # {0: 'amr', 1: 'worker_b', 2: 'worker_a'} — 번호→이름 표
        self.K = None                        # 카메라 내부 파라미터 3×3. 픽셀 → 3D 역투영에 필요. camera_info 에서 한 번 받는다
        self.pair = None                     # 아직 처리하지 않은 최신 (RGB, depth) 한 쌍. 콜백이 채우고 process() 가 비운다

        # 카메라 토픽도 상대 이름 — launch 의 namespace 가 붙는다. '/robot3' 폴백을 두면 robot4 가
        # 네임스페이스 없이 떴을 때 robot3 카메라에 몰래 붙는다
        if DEPTH_TRANSPORT == 'compressedDepth':
            depth_sub = Subscriber(self, CompressedImage, 'oakd/stereo/image_raw/compressedDepth')
        else:
            depth_sub = Subscriber(self, Image, 'oakd/stereo/image_raw')
        rgb_sub = Subscriber(self, CompressedImage, 'oakd/rgb/image_raw/compressed')
        # RGB 와 depth 는 따로 오므로 타임스탬프가 SYNC_SLOP_S 안으로 맞는 쌍만 synced_cb 로 넘긴다
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=SYNC_SLOP_S)
        self.sync.registerCallback(self.synced_cb)
        self.create_subscription(CameraInfo, 'oakd/stereo/camera_info', self.info_cb, 1)
        self.pub_point = self.create_publisher(PointStamped, POINT_TOPIC, 1)
        self.pub_image = self.create_publisher(Image, IMAGE_TOPIC, 1)
        self.pub_visible = self.create_publisher(Bool, VISIBLE_TOPIC, 1)

        self.tf_buffer = Buffer()                            # 카메라 좌표 → map 좌표 변환용 TF
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if SHOW_WINDOW:
            cv2.namedWindow('AMR Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AMR Detection', 1408, 704)
        self.get_logger().info(f'YOLO {MODEL_PATH} | ns={self.get_namespace()} | 검출 대상 {DETECT_CLASSES} '
                               f'| 출력: {POINT_TOPIC}, {VISIBLE_TOPIC}, {IMAGE_TOPIC}')

    def info_cb(self, msg):
        if self.K is None:                                   # 카메라 파라미터는 바뀌지 않으니 첫 번째만 저장
            self.K = np.array(msg.k).reshape(3, 3)

    def synced_cb(self, rgb_msg, depth_msg):
        # 콜백에서는 디코드해서 보관만 한다. YOLO 를 여기서 돌리면 콜백이 느려져 프레임이 큐에 쌓이고
        # 몇 초 전 장면을 처리하게 된다 — 무거운 일은 process() 가 최신 한 쌍만 골라서 한다
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
        self.pair = None                                     # 소비 표시 — 같은 프레임을 두 번 처리하지 않는다
        vis = rgb.copy()                                     # 오버레이는 복사본에 그린다
        label = 'detecting' if self.K is not None else 'waiting for camera_info'

        if self.K is not None:
            # ① YOLO — 작업자(worker_a/worker_b) 중 가장 확신 높은 bbox 하나
            box = best_box(self.model(rgb, verbose=False), self.names, DETECT_CLASSES, DETECT_CONF)
            # ② 보이는지 여부는 거리 품질과 상관없이 매 프레임 발행 — kpi_logger 가 이걸로 Lost(1→0) 를 센다 (K-04)
            self.pub_visible.publish(Bool(data=box is not None))
            if box:
                x1, y1, x2, y2, c, cls_name = box
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # ③ bbox 안쪽 depth 로 거리 산출 (p20·median·보정값). 바깥쪽은 배경이 섞여서 안 쓴다
                r = distance_from_bbox(depth, x1, y1, x2, y2, inner_ratio=BBOX_INNER)
                self.get_logger().info(f'{cls_name} conf={c:.2f} {format_result(r)}')
                # ④ 품질 게이트 — 핵심부 depth 가 흩어져 있으면(가림·경계 걸침) 이 프레임의 거리는 믿지 않는다
                if r is not None and (r['med'] - r['q']) <= MAX_CORE_GAP_MM:
                    # ⑤ bbox 중심 픽셀 + 거리 → 카메라 3D 점 → TF 로 map 좌표
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    pm = to_map(self.tf_buffer, back_project(self.K, u, v, r['z_corr'], frame_id, stamp),
                                self.get_logger())
                    if pm is not None:                       # TF 가 아직 없으면(기동 직후) 이 프레임은 건너뛴다
                        self.pub_point.publish(pm)           # ⑥ follow_target 이 받아 목표점으로 쓴다
                        label = f"{cls_name} {r['z_corr']:.2f} m -> map ({pm.point.x:.2f}, {pm.point.y:.2f})"
                elif r is not None:
                    label = f"core gap {r['med'] - r['q']:.0f} mm > {MAX_CORE_GAP_MM} - rejected"
                cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                label = 'detecting - no worker'

        # 모니터링 프레임: 좌 RGB+bbox, 우 depth 컬러맵. 위쪽 검은 띠에 상태 라벨
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
        # spin() 대신 수동 루프 — 콜백 처리(spin_once)와 무거운 검출(process)을 번갈아 돌리고,
        # cv2.imshow 는 메인 스레드에서만 안전하게 동작한다
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
