#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/amr_detection_node.py → ros2_ws/src/mro_perception/mro_perception/nodes/amr_detection_node.py

amr_detection_node.py — [AMR Detection] 로봇 카메라로 작업자를 찾아 "어디에 있는지·내 담당이 맞는지" 를 알려주는 노드

■ 한 줄 요약
  OAK-D 카메라의 컬러(RGB)·거리(depth) 영상을 받아 → YOLO 로 인형(worker_a / worker_b)을 찾고
  → 웹캠1 이 등록한 "내 담당" 서술자와 대조해 짝을 확인하고(재식별 게이트, BR-T16)
  → 그 인형까지의 거리를 재서 → 로봇이 쓰는 지도(map) 좌표로 바꿔 → follow_target 에게 넘긴다.

■ 데이터 흐름
  oakd/rgb/image_raw/compressed  ─┐
  oakd/stereo/image_raw/compressedDepth ─┤→ 시각 맞춰 한 쌍으로 → synced_cb() 가 보관 → process() 가 처리
  oakd/stereo/camera_info ────────┘  (카메라 내부 파라미터 K — 픽셀을 3D 로 되돌릴 때 필요)
  mission/assigned (관제)  ──────────→ "내 담당은 W1"          ─┐
  /webcam1/worker_descriptor (웹캠1) → "W1 은 worker_a, 색 지문은 …" ─┴→ PairingGate.register()

  process():  ① YOLO 검출 (작업자 후보 전부) → ② 후보마다 색 지문 → ③ 게이트: 내 담당 고르기·paired 판정
              → ④ target/visible 발행 → ⑤ bbox 안쪽 depth 로 거리 → ⑥ 거리 품질 게이트
              → ⑦ 픽셀+거리 → 카메라 3D 점 → 직선거리 = target/distance 발행 → TF 로 map 좌표
              → ⑧ target/map_point 발행 → 오버레이 영상

■ 내보내는 토픽 (계약 interfaces.md 6절 — 전부 상대 이름, launch 가 /robotN 을 앞에 붙인다)
  target/map_point  PointStamped(frame=map)  대상 위치. ⑥ 품질 게이트 통과분만            → follow_target 목표점
  target/visible    Bool                     매 프레임 "대상이 화면에 잡혔나". 품질 무관  → kpi_logger Lost(K-04)
  target/distance   Float32 (m)                대상까지 **직선거리**. ⑥ 품질 게이트 통과분만  → kpi_logger AC-05 추종거리
                                             카메라 3D 점의 노름 — 화면 어디 있든 줄자와 같다. TF 없이 나간다 (TS-U05)
  target/paired     Bool                     재식별 게이트 통과 여부 (바뀔 때 + 1 Hz)     → follow_target 추종 허가,
  target/worker_id  String                   짝이 된 작업자 ID (W1/W2, 미확정이면 '')        fleet_manager → PAIRED
  target/image/compressed  CompressedImage(jpeg)  모니터링. 좌 RGB+bbox / 우 depth 컬러맵 → rqt_image_view
                                             raw 는 내지 않는다 — 2.97 MB×9 Hz=214 Mbit/s 라 Wi-Fi 를 막는다 (I-016 교훈)

■ "대상" 이 누구인가
  서술자·미션이 아직 없으면(S1~S3 전)  → 보이는 작업자 중 확신도 최고. paired 는 False (단독 시험용 동작)
  서술자·미션이 들어오면(S5 이후)      → 내 담당 복장의 후보 중 색 지문이 가장 닮은 것. 남의 담당은 대상이 아니다

■ 용어
  bbox      YOLO 가 찾은 물체를 둘러싼 네모 (x1,y1,x2,y2 픽셀)
  conf      YOLO 의 확신도 0~1
  색 지문   bbox 안 색 분포표(H16×S8=128). core/reid.hsv_hist() — 웹캠1 과 반드시 같은 함수
  depth     픽셀마다 "카메라에서 몇 mm" 가 든 영상. 0 은 측정 실패
  K         카메라 내부 파라미터 3×3. 픽셀 ↔ 3D 변환의 자
  TF        좌표계 변환 관계. 카메라 → base_link → odom → map 을 이어준다
  map 좌표  SLAM 지도 원점 기준 (x, y) m. Nav2 목표·구역 폴리곤·웹캠 힌트가 모두 이 좌표

■ 시작 신호는 없다
  켜지면 곧바로 검출·발행한다. 결과를 "언제부터 쓸지" 는 follow_target 이 shuttle/arrived·target/paired 로 정한다
  (계약 v1.10). 옛 goto_goal/arrived 게이트는 interfaces.md '쓰지 않는 옛 토픽' 에 따라 제거했다.

사용법·시험 절차(TS-U03·U04·U05·Q-13)·오버레이 읽는 법: 패키지 루트 README.md (ros2_ws/src/mro_perception/README.md)

실행: ros2 launch mro_bringup robot.launch.py            (namespace·tf 리맵·robotN.yaml 을 launch 가 붙인다)
단독: ros2 run mro_perception amr_detection_node --ros-args -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static

파라미터 (robotN.yaml / config/perception.yaml 이 덮어쓴다. 아래 모듈 상수는 기본값):
  detect_conf · reid_threshold · descriptor_frames · pair_on_s · pair_off_s · publish_hz
  · max_core_gap_mm · bbox_inner · sync_slop_s · show_window
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from mro_interfaces.msg import Mission, WorkerDescriptor

# core/ 는 ROS 에 의존하지 않는 순수 계산 함수 모음 (단위 테스트 대상). 이 노드는 메시지 변환과 호출 순서만 맡는다
from mro_perception.core.depth_distance import (DEPTH_TRANSPORT, colorize_depth, decode_compressed_depth,
                            distance_from_bbox, format_result)
from mro_perception.core.reid import PairingGate, hsv_hist
from mro_perception.core.target_utils import back_project, to_map

# ── 모델 ─────────────────────────────────────────────────────────────────────────────────────────────
# worker.pt 는 mro_perception/models/ 가 단일 출처 — 웹캠1·2 launch 도 같은 파일을 가리킨다 (models/README.md)
MODEL_PATH = os.path.join(get_package_share_directory('mro_perception'), 'models', 'worker.pt')

# ── 토픽 (계약 interfaces.md 6절) ─────────────────────────────────────────────────────────────────────
POINT_TOPIC = 'target/map_point'     # 대상 위치(map). 거리 품질 게이트를 통과한 검출만. → follow_target
IMAGE_TOPIC = 'target/image/compressed'   # 모니터링 오버레이 JPEG. raw 로 내면 다른 PC 의 rqt 가 Wi-Fi 를 막는다
VISIBLE_TOPIC = 'target/visible'     # 매 프레임 True/False. 거리 품질과 무관 → kpi_logger 가 Lost(1→0) 를 센다
DISTANCE_TOPIC = 'target/distance'   # 대상까지 직선거리(m). AC-05 의 기준값(FR-29) → kpi_logger. 품질 통과분만
PAIRED_TOPIC = 'target/paired'       # 재식별 게이트 통과 여부 → follow_target(추종 허가), fleet_manager(PAIRED 전이)
WORKER_ID_TOPIC = 'target/worker_id'  # 짝이 된 작업자 ID (W1/W2). 미확정이면 ''
MISSION_TOPIC = 'mission/assigned'   # 관제 → 이 로봇. worker_id 로 "내 담당" 을 안다 (상대 이름)
DESCRIPTOR_TOPIC = '/webcam1/worker_descriptor'   # 웹캠1 → 전역. 절대 이름 — 네임스페이스를 붙이면 안 된다

# ── 검출 ────────────────────────────────────────────────────────────────────────────────────────────
# 클래스는 반드시 "이름" 으로 거른다. worker.pt 의 번호표가 알파벳순이라 amr=0, worker_b=1, worker_a=2 다
# (models/README.md). 번호로 매핑하면 검출은 멀쩡해 보이는데 a/b 가 뒤바뀌어 페어링만 전부 틀린다
DETECT_CLASSES = ('worker_a', 'worker_b')   # 추종 후보 = 작업자 둘. amr(상대 로봇)은 후보에서 뺀다 (회피 용도는 미결)
DETECT_CONF = 0.5            # 이 확신도 아래는 버린다. 오탐 = 오추종(AC-04 0건)이라 F1 최적(0.4)보다 높게

# ── 재식별 게이트 (파라미터 기본값 — robotN.yaml 이 덮어쓴다) ──────────────────────────────────────────
REID_THRESHOLD = 0.6         # 색 지문 유사도 임계. Q-13 [팀 확정] — AMR 시점(TS-U04 프레임)으로 분포를 재서 정한다
DESCRIPTOR_FRAMES = 10       # 지문 평균 창. 웹캠1 의 DESCRIPTOR_FRAMES 와 같아야 같은 척도다
PAIR_ON_S = 1.0              # 이 시간 연속 통과해야 paired=True (웹캠1 register_s 와 같은 발상)
PAIR_OFF_S = 3.0             # 이 시간 연속 실패·미검출이면 paired=False. 짧은 가림에 짝이 풀리지 않게 [팀 확정]
PUBLISH_HZ = 1.0             # paired·worker_id 재발행 주기. 늦게 켜진 노드도 받게 한다
JPEG_QUALITY = 80            # 모니터 영상 압축 품질. 웹캠 노드와 같은 값 (webcam.yaml jpeg_quality)
DIAG_LOG_S = 1.0             # 계측 로그 주기(초). 0 이면 끔. 렉 원인 분리용 — 처리 프레임 나이·루프/콜백/처리 속도

# ── 거리 측정 (파라미터 기본값) ──────────────────────────────────────────────────────────────────────
BBOX_INNER = 0.5             # bbox 중심부 50 % 만 depth 를 읽는다. 가장자리는 인형 뒤 바닥·벽의 depth 가 섞인다
# distance_from_bbox() 는 ROI 안 depth 의 p20(q)·중앙값(med)·최댓값 등을 돌려준다.
# 인형 하나만 깨끗하게 들어오면 p20 과 med 가 비슷하고, 가림·경계에 걸치면 둘이 벌어진다 → 그 프레임의 거리는 안 믿는다.
# (max-min 인 spread 는 bbox 에 뒷배경이 조금만 들어와도 1 m 를 넘어 게이트로 못 쓴다 — 실측 로그)
MAX_CORE_GAP_MM = 150

# ── 입력·표시 (파라미터 기본값) ──────────────────────────────────────────────────────────────────────
SYNC_SLOP_S = 0.1            # RGB 와 depth 의 타임스탬프 차이가 이 안(초)이면 "같은 순간" 으로 묶는다
SHOW_WINDOW = True           # 로컬 미리보기 창. 모니터 없는 로봇 PC 에서는 yaml 로 false — 창을 못 만들면 노드가 죽는다


def worker_boxes(results, names, cls_names, conf_min):
    """YOLO 결과에서 작업자 후보 전부를 확신도 내림차순으로 돌려준다.

    results 구조: results[i].boxes 가 그 프레임의 검출 목록.
      b.cls[0]  클래스 번호 → names 표로 이름을 얻는다
      b.conf[0] 확신도 0~1
      b.xyxy[0] bbox 픽셀 (x1, y1, x2, y2)
    반환: [(x1, y1, x2, y2, conf, 클래스이름), ...]. 후보를 하나로 줄이는 건 게이트의 일이다 —
    여기서 확신도 1등만 남기면 두 인형이 다 보일 때 내 담당을 못 볼 수 있다.
    """
    out = []
    for r in results:
        for b in r.boxes:
            name = names.get(int(b.cls[0]))                 # 번호 → 이름
            if name not in cls_names or float(b.conf[0]) < conf_min:   # 작업자가 아니거나 확신이 낮으면 버린다
                continue
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            out.append((x1, y1, x2, y2, float(b.conf[0]), name))
    return sorted(out, key=lambda t: t[4], reverse=True)


class AmrDetection(Node):
    """카메라 입력은 콜백이 '보관' 만 하고, 무거운 YOLO·거리 계산은 process() 가 최신 한 쌍만 골라서 한다.

    콜백 안에서 YOLO 를 돌리면 콜백이 느려져 프레임이 큐에 쌓이고 몇 초 전 장면을 처리하게 된다.
    그래서 구조가 [콜백 = 저장] / [process = 계산] 으로 나뉘어 있고, main() 이 둘을 번갈아 돌린다.
    재식별 판정은 core.reid.PairingGate 가 맡고, 이 노드는 서술자·미션을 넣어주고 결과를 토픽으로 낼 뿐이다.
    """

    def __init__(self):
        super().__init__('amr_detection')
        self.bridge = CvBridge()                             # ROS Image ↔ OpenCV 배열 변환기

        # 파라미터 — robotN.yaml(/robotN/** 와일드카드) 이 덮어쓴다. 없으면 위 모듈 기본값
        self.declare_parameter('reid_threshold', REID_THRESHOLD)
        self.declare_parameter('detect_conf', DETECT_CONF)
        self.declare_parameter('descriptor_frames', DESCRIPTOR_FRAMES)
        self.declare_parameter('pair_on_s', PAIR_ON_S)
        self.declare_parameter('pair_off_s', PAIR_OFF_S)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('max_core_gap_mm', MAX_CORE_GAP_MM)
        self.declare_parameter('bbox_inner', BBOX_INNER)
        self.declare_parameter('sync_slop_s', SYNC_SLOP_S)
        self.declare_parameter('show_window', SHOW_WINDOW)
        self.declare_parameter('diag_log_s', DIAG_LOG_S)
        self.declare_parameter('jpeg_quality', JPEG_QUALITY)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.detect_conf = float(p('detect_conf'))
        self.max_core_gap_mm = float(p('max_core_gap_mm'))
        self.bbox_inner = float(p('bbox_inner'))
        self.show_window = bool(p('show_window'))
        # 계측 — 렉이 '입력이 늦게 오는 것'인지 '노드가 못 따라가는 것'인지 가른다 (README 7절)
        self.diag_log_s = float(p('diag_log_s'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.diag = dict(t0=time.monotonic(), loops=0, rgb_cb=0, depth_cb=0, synced=0, processed=0,
                         age_sum=0.0, age_max=0.0, proc_ms_sum=0.0, proc_ms_max=0.0, tf_cb=0)

        # 모델. names 는 {0: 'amr', 1: 'worker_b', 2: 'worker_a'} — 번호를 이름으로 바꾸는 표
        self.model = YOLO(MODEL_PATH)
        self.names = self.model.names

        # 상태 변수
        self.K = None        # 카메라 내부 파라미터 3×3. camera_info 에서 한 번 받으면 끝. 없으면 3D 변환 불가
        self.pair = None     # 아직 처리 안 한 최신 (RGB, depth, 시각, 카메라 frame_id). 콜백이 채우고 process() 가 비운다

        # 재식별 — "내 담당" 은 두 메시지를 합쳐야 안다: mission/assigned 가 worker_id 를, 웹캠1 서술자가 그 사람의
        # 복장·색 지문을 준다. 어느 쪽이 먼저 와도 되게 둘 다 보관하고 _bind() 에서 맞춘다
        self.gate = PairingGate(float(p('reid_threshold')), int(p('descriptor_frames')),
                                float(p('pair_on_s')), float(p('pair_off_s')))
        self.my_worker_id = ''           # mission/assigned 에서
        self.descriptors = {}            # worker_id -> (outfit_class, color_hist)  웹캠1 이 1 Hz 로 재발행
        self.paired = False              # 마지막으로 발행한 값 (바뀔 때 즉시 발행)
        self.last_reason = ''

        # 카메라 입력. 토픽은 상대 이름 — launch 의 namespace 가 앞에 붙는다.
        # ('/robot3' 폴백을 두면 robot4 가 네임스페이스 없이 떴을 때 robot3 카메라에 몰래 붙는다 → 두지 않는다)
        if DEPTH_TRANSPORT == 'compressedDepth':             # Wi-Fi 대역폭 때문에 depth 는 압축본을 받는다 (기본)
            depth_sub = Subscriber(self, CompressedImage, 'oakd/stereo/image_raw/compressedDepth')
        else:
            depth_sub = Subscriber(self, Image, 'oakd/stereo/image_raw')
        rgb_sub = Subscriber(self, CompressedImage, 'oakd/rgb/image_raw/compressed')
        rgb_sub.registerCallback(lambda m: self.diag.__setitem__('rgb_cb', self.diag['rgb_cb'] + 1))
        depth_sub.registerCallback(lambda m: self.diag.__setitem__('depth_cb', self.diag['depth_cb'] + 1))
        # RGB 와 depth 는 별도 토픽으로 따로 도착한다. 타임스탬프가 SYNC_SLOP_S 안으로 맞는 쌍만 synced_cb 로 넘겨,
        # "이 컬러 프레임의 이 픽셀이 저 depth 프레임의 이 거리" 라고 말할 수 있게 한다
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=float(p('sync_slop_s')))
        self.sync.registerCallback(self.synced_cb)
        self.create_subscription(CameraInfo, 'oakd/stereo/camera_info', self.info_cb, 1)

        # 재식별 입력
        self.create_subscription(Mission, MISSION_TOPIC, self.mission_cb, 10)
        self.create_subscription(WorkerDescriptor, DESCRIPTOR_TOPIC, self.descriptor_cb, 10)

        # 출력
        self.pub_point = self.create_publisher(PointStamped, POINT_TOPIC, 1)
        self.pub_image = self.create_publisher(CompressedImage, IMAGE_TOPIC, 1)
        self.pub_visible = self.create_publisher(Bool, VISIBLE_TOPIC, 1)
        self.pub_distance = self.create_publisher(Float32, DISTANCE_TOPIC, 10)   # p95 표본이라 큐를 넉넉히
        self.pub_paired = self.create_publisher(Bool, PAIRED_TOPIC, 10)
        self.pub_worker_id = self.create_publisher(String, WORKER_ID_TOPIC, 10)
        self.create_timer(1.0 / float(p('publish_hz')), self.publish_pairing)   # 재발행 — 늦게 켜진 노드용

        # TF — 카메라 좌표의 점을 map 좌표로 바꿀 때 쓴다. Listener 가 /tf 를 듣고 Buffer 에 모아둔다
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # /tf 콜백 횟수 — 같은 실행기에서 카메라 콜백과 경쟁하는 양. 구독 객체의 callback 은 실행 시점에 읽히므로
        # (rclpy Subscription.callback 은 평범한 속성) 여기서 감싸면 센다. 리스너의 메서드를 바꾸는 건 소용없다
        _tf_cb = self.tf_listener.tf_sub.callback

        def _tf_counted(m):
            self.diag['tf_cb'] += 1
            return _tf_cb(m)
        self.tf_listener.tf_sub.callback = _tf_counted

        if self.show_window:
            cv2.namedWindow('AMR Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AMR Detection', 1408, 704)     # 704×704 두 장을 가로로 붙인 크기
        self.get_logger().info(f'YOLO {MODEL_PATH} | ns={self.get_namespace()} | 검출 대상 {DETECT_CLASSES} '
                               f'conf≥{self.detect_conf} | reid_threshold={self.gate.threshold} '
                               f'| 출력: {POINT_TOPIC}, {VISIBLE_TOPIC}, {DISTANCE_TOPIC}, '
                               f'{PAIRED_TOPIC}, {WORKER_ID_TOPIC}')

    # ── 콜백: 보관만 한다 ──────────────────────────────────────────────────────────────────────────────
    def info_cb(self, msg):
        """camera_info → K. 카메라 파라미터는 바뀌지 않으니 첫 메시지만 저장한다."""
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)           # msg.k 는 9개짜리 1차원 → 3×3

    def synced_cb(self, rgb_msg, depth_msg):
        """시각이 맞는 (RGB, depth) 한 쌍 → 디코드해서 self.pair 에 보관만 한다. 계산은 process() 가."""
        depth = (decode_compressed_depth(depth_msg) if isinstance(depth_msg, CompressedImage)
                 else self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))   # mm 단위 2차원 배열
        rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)          # JPEG → BGR 배열
        if depth is not None and rgb is not None:
            self.diag['synced'] += 1
            # 시각·frame_id 는 depth 쪽 것을 쓴다 — 거리(3D 점)의 출처가 depth 카메라이기 때문
            self.pair = (rgb, depth, depth_msg.header.stamp, depth_msg.header.frame_id)

    def mission_cb(self, msg):
        """mission/assigned → 내 담당 worker_id. 관제가 로봇마다 따로 보내지만 robot_ns 가 있으면 한 번 더 확인한다."""
        if msg.robot_ns and msg.robot_ns != self.get_namespace():
            return
        if msg.worker_id and msg.worker_id != self.my_worker_id:
            self.my_worker_id = msg.worker_id
            self.get_logger().info(f'미션 배정: 내 담당 = {self.my_worker_id} (mission {msg.mission_id})')
            self._bind()

    def descriptor_cb(self, msg):
        """/webcam1/worker_descriptor → worker_id 별 (복장, 색 지문). 웹캠1 이 1 Hz 로 재발행하므로 늦게 켜져도 받는다."""
        if not msg.worker_id or not msg.color_hist:
            return
        self.descriptors[msg.worker_id] = (msg.outfit_class, np.asarray(msg.color_hist, np.float32))
        if msg.worker_id == self.my_worker_id:
            self._bind()

    def _bind(self):
        """worker_id 와 서술자가 둘 다 있으면 게이트에 등록한다. 서술자가 갱신되면 지문도 따라 갱신된다."""
        d = self.descriptors.get(self.my_worker_id)
        if d is None:
            return
        first = not self.gate.ready()
        self.gate.register(d[0], d[1])
        if first:
            self.get_logger().info(f'재식별 게이트 준비: {self.my_worker_id} = {d[0]}')

    # ── 출력 ─────────────────────────────────────────────────────────────────────────────────────────
    def publish_pairing(self, force=False):
        """paired·worker_id 발행. 타이머(1 Hz)와 값이 바뀐 순간 둘 다에서 부른다."""
        self.pub_paired.publish(Bool(data=self.paired))
        self.pub_worker_id.publish(String(data=self.my_worker_id if self.paired else ''))

    def _set_paired(self, paired, reason):
        if paired != self.paired:
            self.paired = paired
            self.get_logger().info(f'paired={paired} ({self.my_worker_id}) — {reason}')
            self.publish_pairing()                           # 바뀐 순간 즉시. 타이머는 재발행용
        self.last_reason = reason

    # ── 메인 처리 ─────────────────────────────────────────────────────────────────────────────────────
    def process(self):
        """메인 루프가 매번 호출. 보관된 최신 한 쌍을 ①~⑧ 순서로 처리하고 소비한다."""
        self.diag['loops'] += 1
        self._diag_log()
        if self.pair is None:                                # 새 프레임이 없으면 할 일 없음
            return
        rgb, depth, stamp, frame_id = self.pair
        t_proc = time.perf_counter()
        age = (self.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9   # 카메라 찍힘 → 지금
        self.diag['age_sum'] += age
        self.diag['age_max'] = max(self.diag['age_max'], age)
        self.pair = None                                     # 소비 표시 — 같은 프레임을 두 번 처리하지 않는다
        vis = rgb.copy()                                     # 오버레이는 복사본에 그린다 (원본은 YOLO 입력)
        label = 'detecting' if self.K is not None else 'waiting for camera_info'

        if self.K is not None:                               # K 가 없으면 3D 변환을 못 하니 검출도 건너뛴다
            # ① YOLO — 작업자 후보 전부 (확신도 내림차순)
            boxes = worker_boxes(self.model(rgb, verbose=False), self.names, DETECT_CLASSES, self.detect_conf)

            # ②③ 게이트 — 서술자·미션이 있으면 "내 담당" 을 고른다. 없으면(단독 시험·S1 전) 확신도 1등을 대상으로
            if self.gate.ready():
                cands = []
                for bx in boxes:
                    x1, y1, x2, y2 = bx[:4]
                    crop = rgb[max(0, y1):y2, max(0, x1):x2]     # 웹캠1 과 같은 규칙 — bbox 전체 (계약 v1.3)
                    if crop.size:
                        cands.append((bx[5], hsv_hist(crop), bx))
                t = self.get_clock().now().nanoseconds / 1e9
                paired, target, sim, reason = self.gate.update(t, cands)
                self._set_paired(paired, reason)
                tag = f'sim={sim:.2f} {"PAIRED" if paired else "unpaired"}'
            else:
                target, tag = (boxes[0] if boxes else None), 'no descriptor (unpaired)'
                self._set_paired(False, 'no descriptor')

            # ④ "보이나" 는 거리 품질과 무관하게 매 프레임 발행. kpi_logger 가 1→0 (1 s 이상) 을 Lost 로 센다 (K-04)
            self.pub_visible.publish(Bool(data=target is not None))

            for bx in boxes:                                 # 후보 전부 얇게 그려서 무엇이 보였는지 남긴다
                cv2.rectangle(vis, bx[:2], bx[2:4], (120, 120, 120), 1)
            if target:
                x1, y1, x2, y2, c, cls_name = target
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # ⑤ 거리 — bbox 중심부(BBOX_INNER) depth 의 p20(q)·중앙값(med)·보정 거리(z_corr, m)
                r = distance_from_bbox(depth, x1, y1, x2, y2, inner_ratio=self.bbox_inner)
                self.get_logger().info(f'{cls_name} conf={c:.2f} {tag} | {format_result(r)}')

                # ⑥ 품질 게이트 — p20 과 med 가 MAX_CORE_GAP_MM 이상 벌어지면 가림·경계 걸침이다. 이 프레임 거리는 버린다
                if r is not None and (r['med'] - r['q']) <= self.max_core_gap_mm:
                    # ⑦ bbox 중심 픽셀 (u,v) + 정면 거리 z → 카메라 광학 좌표의 3D 점 (X 우, Y 하, Z 전방)
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    pt = back_project(self.K, u, v, r['z_corr'], frame_id, stamp)
                    # 거리는 이 3D 점의 노름 = 직선거리. depth 의 z 는 "정면 방향" 거리라 화면 가장자리에서는
                    # 직선거리보다 최대 20 % 짧다 — 줄자(TS-U05)·추종거리(AC-05)는 직선거리다.
                    # TF 와 무관하게 여기서 낸다 (map 없이도 줄자 대조가 되게). 품질 게이트 통과분만 — p95 오염 방지
                    dist = math.hypot(pt.point.x, pt.point.y, pt.point.z)
                    self.pub_distance.publish(Float32(data=dist))
                    # 같은 점을 map 좌표로. timeout=0 — 기다리면 실행기가 멈춰 /tf 가 못 들어오므로(단일 스레드)
                    # 즉시 판정. 없으면 이 프레임만 건너뛴다. 1 s 대기 두 번이 탐지 프레임마다 2 s 렉을 만들었다 (9/8)
                    pm = to_map(self.tf_buffer, pt, self.get_logger(), timeout=Duration())
                    if pm is not None:                       # TF 가 아직 없으면(기동 직후·시계 어긋남) 건너뛴다
                        # ⑧ follow_target 이 받아 "이 앞 0.5 m" 를 목표로 삼는다 (paired 일 때만 쓴다 — 주행 TODO ④)
                        self.pub_point.publish(pm)
                        label = f"{cls_name} {dist:.2f} m -> map ({pm.point.x:.2f}, {pm.point.y:.2f}) {tag}"
                    else:
                        label = f"{cls_name} {dist:.2f} m (no TF) {tag}"
                elif r is not None:
                    label = f"core gap {r['med'] - r['q']:.0f} mm > {self.max_core_gap_mm:.0f} - rejected"
                cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                label = f'detecting - no target ({self.last_reason})'

        # 모니터링 프레임 — 좌 RGB+bbox, 우 depth 컬러맵, 위쪽 검은 띠에 상태 라벨. 검출이 없어도 매 프레임 내보낸다
        frame = np.hstack((vis, colorize_depth(depth)))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(frame, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        # JPEG 로 압축해 발행 (16 ms, 0.7 MB). raw Image 3 MB 는 같은 PC 안에서만 감당된다
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = stamp                          # 원본 프레임 시각 — ros2 topic delay 로 지연을 잴 수 있게
            msg.header.frame_id = frame_id
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub_image.publish(msg)
        if self.show_window:
            cv2.imshow('AMR Detection', frame)
        ms = (time.perf_counter() - t_proc) * 1000
        self.diag['processed'] += 1
        self.diag['proc_ms_sum'] += ms
        self.diag['proc_ms_max'] = max(self.diag['proc_ms_max'], ms)

    def _diag_log(self):
        """diag_log_s 마다 한 줄. 읽는 법은 README 7절."""
        if self.diag_log_s <= 0:
            return
        d = self.diag
        dt = time.monotonic() - d['t0']
        if dt < self.diag_log_s:
            return
        n = max(d['processed'], 1)
        self.get_logger().info(
            f"[diag {dt:.1f}s] loop {d['loops'] / dt:.0f}/s | cb rgb {d['rgb_cb'] / dt:.1f} "
            f"depth {d['depth_cb'] / dt:.1f} tf {d['tf_cb'] / dt:.0f}/s | synced {d['synced'] / dt:.1f}/s "
            f"| processed {d['processed'] / dt:.1f}/s "
            f"| age avg {d['age_sum'] / n * 1000:.0f} max {d['age_max'] * 1000:.0f} ms "
            f"| proc avg {d['proc_ms_sum'] / n:.0f} max {d['proc_ms_max']:.0f} ms")
        d.update(t0=time.monotonic(), loops=0, rgb_cb=0, depth_cb=0, synced=0, processed=0,
                 age_sum=0.0, age_max=0.0, proc_ms_sum=0.0, proc_ms_max=0.0, tf_cb=0)


def main():
    rclpy.init()
    node = AmrDetection()
    try:
        # spin() 대신 수동 루프. spin_once 가 콜백(프레임 수신·TF·camera_info·미션·서술자·타이머)을 처리하고,
        # process 가 계산한다. cv2.imshow 는 메인 스레드에서만 안전해서 여기서 waitKey 를 함께 돈다 (q 로 종료)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process()
            if node.show_window and (cv2.waitKey(1) & 0xFF) == ord('q'):
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
