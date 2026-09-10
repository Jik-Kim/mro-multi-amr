#!/usr/bin/env python3
"""로봇 카메라로 작업자를 찾아 "어디에 있는지·내 담당이 맞는지" 를 알려주는 노드.

[이관] Kn1_mini/nav/amr_detection_node.py → mro_perception/nodes/amr_detection_node.py

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
              → ③' 폴백 상태: 보이는데 30 s 못 넘으면 target/pair_state=GATE_FAIL (관제 버튼)
              → ④ target/visible 발행 → ⑤ bbox 안쪽 depth 로 거리 → ⑥ 거리 품질 게이트
              → ⑦ 픽셀+거리 → 카메라 3D 점 → 직선거리 = target/distance 발행 → TF 로 map 좌표
              → ⑧ target/map_point 발행 → 오버레이 영상

■ 내보내는 토픽 (계약 interfaces.md 6절 — 전부 상대 이름, launch 가 /robotN 을 앞에 붙인다)
  target/map_point  PointStamped(frame=map)  대상 위치. ⑥ 품질 게이트 통과분만            → follow_target 목표점
  target/visible    Bool                     매 프레임 "대상이 화면에 잡혔나". 품질 무관  → kpi_logger Lost(K-04)
  target/distance   Float32 (m)              대상까지 **직선거리**. ⑥ 품질 통과분만  → kpi_logger AC-05
                                             카메라 3D 점의 노름 — 화면 어디 있든 줄자와 같다. TF 없이 나간다 (TS-U05)
  target/paired     Bool                     재식별 게이트 통과 여부 (바뀔 때 + 1 Hz)     → follow_target 추종 허가,
  target/worker_id  String                   짝이 된 작업자 ID (W1/W2, 미확정 '')  fleet_manager → PAIRED
  target/pair_state String                   PAIRED / GATE_FAIL / NO_TARGET   → control_ui 폴백 버튼
  target/image/compressed  CompressedImage(jpeg)  모니터링. 좌 RGB+bbox / 우 depth 컬러맵 → rqt_image_view
                                             raw 는 내지 않는다 — 2.97 MB×9 Hz 라 Wi-Fi 를 막는다 (I-016)

■ "대상" 이 누구인가
  서술자·미션이 아직 없으면(S1~S3 전)  → 보이는 작업자 중 확신도 최고. paired 는 False (단독 시험용 동작)
  서술자·미션이 들어오면(S5 이후)      → 내 담당 복장의 후보 중 색 지문이 가장 닮은 것. 남의 담당은 대상이 아니다

■ 명시 페어링 폴백 (FR-13, 계약 10절) — 게이트를 못 넘어 로봇이 영영 서 있지 않게 운영자가 짝을 정해 주는 길
  손 들기   작업자가 보이는데 게이트 미달이 30 s(gate_fail_s) 이어지면 pair_state=GATE_FAIL → 관제 UI 에 버튼.
            사람이 안 보이는 동안(NO_TARGET)은 세지 않는다 — 그건 못 알아보는 게 아니라 없는 것이다 (10.1)
  창구      서비스 manual_pair (fleet_manager 가 /robotN/manual_pair 로 부른다). 요청 (robot_ns, worker_id) 을
            core/reid.manual_pair_decision 이 검사 — 남의 로봇·이미 짝·안 보임·1 m 밖·그새 스스로 통과 → 거부 + 사유
  확정      수락하면 여기서 paired=True·worker_id 를 낸다 (발행자는 이 노드 하나뿐, 10.4). 자동 게이트가 뒤집지 못하게
            manual_worker 로 잠그고, mission/finish 가 오면 잠금·게이트를 함께 푼다 (10.6, FR-30)

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
단독: ros2 run mro_perception amr_detection_node --ros-args -r __ns:=/robot3
        -r /tf:=tf -r /tf_static:=tf_static

파라미터 (robotN.yaml / config/perception.yaml 이 덮어쓴다. 아래 모듈 상수는 기본값):
  detect_conf · reid_threshold · descriptor_frames · pair_on_s · pair_off_s · publish_hz
  · max_core_gap_mm · bbox_inner · sync_slop_s · show_window · camera_fx_scale(I-031, aruco 와 공용)
  · gate_fail_s · manual_pair_max_m · target_stale_s (FR-13)
"""
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from mro_interfaces.msg import Mission, WorkerDescriptor
from mro_interfaces.srv import ManualPair
# core/ 는 ROS 에 의존하지 않는 순수 계산 함수 모음 (단위 테스트 대상).
# 이 노드는 메시지 변환과 호출 순서만 맡는다
from mro_perception.core.camera import corrected_K, read_fx_scale
from mro_perception.core.depth_distance import (colorize_depth, decode_compressed_depth,
                                                DEPTH_TRANSPORT, distance_from_bbox, format_result)
from mro_perception.core.reid import (hsv_hist, manual_pair_decision, norm_ns, PairingGate,
                                      PairStateTracker)
from mro_perception.core.target_utils import back_project, to_map
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

# ── 모델 ───────────────────────────────────────────────────────────────────────────────────────────
# worker.pt 는 mro_perception/models/ 가 단일 출처 — 웹캠1·2 launch 도 같은 파일을 가리킨다 (models/README.md)
MODEL_PATH = os.path.join(get_package_share_directory('mro_perception'), 'models', 'worker.pt')

# ── 토픽 (계약 interfaces.md 6절) ─────────────────────────────────────────────────────────────────────
POINT_TOPIC = 'target/map_point'     # 대상 위치(map). 거리 품질 게이트를 통과한 검출만. → follow_target
IMAGE_TOPIC = 'target/image/compressed'   # 모니터링 오버레이 JPEG. raw 로 내면 다른 PC 의 rqt 가 Wi-Fi 를 막는다
VISIBLE_TOPIC = 'target/visible'     # 매 프레임 True/False. 거리 품질과 무관 → kpi_logger 가 Lost(1→0) 를 센다
DISTANCE_TOPIC = 'target/distance'   # 대상까지 직선거리(m). AC-05 의 기준값(FR-29) → kpi_logger. 품질 통과분만
# 재식별 게이트 통과 여부 → follow_target(추종 허가), fleet_manager(PAIRED 전이)
PAIRED_TOPIC = 'target/paired'
WORKER_ID_TOPIC = 'target/worker_id'  # 짝이 된 작업자 ID (W1/W2). 미확정이면 ''
# PAIRED / GATE_FAIL / NO_TARGET → control_ui(폴백 버튼)·fleet_manager (계약 10.2)
PAIR_STATE_TOPIC = 'target/pair_state'
# srv/ManualPair 서버. fleet_manager 가 /robotN/manual_pair 로 부른다 (계약 10.3)
MANUAL_PAIR_SERVICE = 'manual_pair'
FINISH_TOPIC = 'mission/finish'          # 관제 → 이 로봇. True 면 페어링(자동·수동 모두) 즉시 해제 (FR-30, 계약 10.6)
MISSION_TOPIC = 'mission/assigned'   # 관제 → 이 로봇. worker_id 로 "내 담당" 을 안다 (상대 이름)
DESCRIPTOR_TOPIC = '/webcam1/worker_descriptor'   # 웹캠1 → 전역. 절대 이름 — 네임스페이스를 붙이면 안 된다

# ── 검출 ───────────────────────────────────────────────────────────────────────────────────────────
# 클래스는 반드시 "이름" 으로 거른다. worker.pt 의 번호표가 알파벳순이라 amr=0, worker_b=1, worker_a=2 다
# (models/README.md). 번호로 매핑하면 검출은 멀쩡해 보이는데 a/b 가 뒤바뀌어 페어링만 전부 틀린다
DETECT_CLASSES = ('worker_a', 'worker_b')   # 추종 후보 = 작업자 둘. amr(상대 로봇)은 후보에서 뺀다 (회피 용도는 미결)
DETECT_CONF = 0.5            # 이 확신도 아래는 버린다. 오탐 = 오추종(AC-04 0건)이라 F1 최적(0.4)보다 높게

# ── 재식별 게이트 (파라미터 기본값 — robotN.yaml 이 덮어쓴다) ──────────────────────────────────────────
REID_THRESHOLD = 0.6         # 색 지문 유사도 임계. Q-13 [팀 확정] — AMR 시점(TS-U04 프레임)으로 분포를 재서 정한다
DESCRIPTOR_FRAMES = 10       # 지문 평균 창. 웹캠1 의 DESCRIPTOR_FRAMES 와 같아야 같은 척도다
PAIR_ON_S = 1.0              # 이 시간 연속 통과해야 paired=True (웹캠1 register_s 와 같은 발상)
PAIR_OFF_S = 3.0             # 이 시간 연속 실패·미검출이면 paired=False. 짧은 가림에 짝이 풀리지 않게 [팀 확정]
PUBLISH_HZ = 1.0             # paired·worker_id·pair_state 재발행 주기. 늦게 켜진 노드도 받게 한다
# 작업자가 보이는데 게이트를 이 시간 넘게 못 넘으면 pair_state=GATE_FAIL → 관제에 폴백 버튼 (FR-13)
GATE_FAIL_S = 30.0
MANUAL_PAIR_MAX_M = 1.0      # 명시 페어링도 이 거리 안에서만 받는다 (FR-14). 멀면 좌표 오차가 커서 수동이든 자동이든 결과가 같다
# 마지막으로 본 후보·거리를 "지금" 으로 치는 시간. 정상(10 fps)이면 열 배 여유지만, 프레임이 이보다 늦어지면
# 폴백 경로가 통째로 닫힌다 — 버튼도 안 뜨고(1 Hz 타이머가 NO_TARGET 으로 내림) 요청도 NO_TARGET 으로 거절된다.
# 렉이 나는 현장에서 ros2 param 으로 늘릴 수 있어야 하므로 파라미터다 (I-045)
TARGET_STALE_S = 1.0
JPEG_QUALITY = 80            # 모니터 영상 압축 품질. 웹캠 노드와 같은 값 (webcam.yaml jpeg_quality)
DIAG_LOG_S = 1.0             # 계측 로그 주기(초). 0 이면 끔. 렉 원인 분리용 — 처리 프레임 나이·루프/콜백/처리 속도
# to_map 이 TF 를 기다리는 시간. 0 = 즉시 판정(정상).
# 1.0 이면 9/8 렉(프레임당 2 s)이 재현된다 — 시연·검증용
TF_WAIT_S = 0.0

# ── 거리 측정 (파라미터 기본값) ──────────────────────────────────────────────────────────────────────
BBOX_INNER = 0.5             # bbox 중심부 50 % 만 depth 를 읽는다. 가장자리는 인형 뒤 바닥·벽의 depth 가 섞인다
# distance_from_bbox() 는 ROI 안 depth 의 p20(q)·중앙값(med)·최댓값 등을 돌려준다.
# 인형 하나만 깨끗하게 들어오면 p20 과 med 가 비슷하고, 가림·경계에 걸치면 둘이 벌어진다 → 그 프레임의 거리는 안 믿는다.
# (max-min 인 spread 는 bbox 에 뒷배경이 조금만 들어와도 1 m 를 넘어 게이트로 못 쓴다 — 실측 로그)
MAX_CORE_GAP_MM = 150

# ── 입력·표시 (파라미터 기본값) ──────────────────────────────────────────────────────────────────────
SYNC_SLOP_S = 0.1            # RGB 와 depth 의 타임스탬프 차이가 이 안(초)이면 "같은 순간" 으로 묶는다
# 동기화기가 짝을 기다리며 들고 있는 프레임 수 = rgb·depth 도착 시차 허용치 (20 ≈ 2 s). 작으면(3 으로 뒀다가 9/8 synced 0)
# depth 가 rgb 보다 늦게 오는 순간 짝이 영영 안 맞는다. 묵은 프레임 재생 방지는 CAMERA_QOS depth 1 이 맡는다 — 이 큐는
# 이미 받은 것만 들고 있다
SYNC_QUEUE = 20
# 카메라 구독 QoS — "최신 한 장만". 처리가 잠깐 밀려도 전송 계층이 묵은 프레임을 버려 화면이 '몇 초 전' 이 되지 않는다.
# RELIABLE·큐 10 이던 이관 설정은 밀린 10장을 순서대로 재생했다 (9/8 렉 분석). BEST_EFFORT 구독은 RELIABLE 발행자도 받는다
CAMERA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
# 실행 중에 못 바꾸는 파라미터 — 값이 구조를 만들기 때문이다. 동기화기·창·타이머·지문 버퍼·보정 K 는
# 기동 때 한 번 만들어지고, 뒤에 값만 바꿔도 이미 만들어진 것에는 닿지 않는다. 바꾸려면 노드를 다시 띄운다
RESTART_ONLY = ('sync_slop_s', 'show_window', 'publish_hz', 'descriptor_frames', 'camera_fx_scale')

# 로컬 미리보기 창. 기본은 끔 (I-043) — 화면 없는 PC 에서 창을 못 만들면 노드가 죽는다. 창이 안 뜨는 손해는
# 플래그 하나 붙여 다시 띄우면 끝이지만, 죽는 손해는 그 회차를 통째로 날린다. 보려면 show_window:=true
SHOW_WINDOW = False


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
        self.declare_parameter('tf_wait_s', TF_WAIT_S)
        self.declare_parameter('gate_fail_s', GATE_FAIL_S)
        self.declare_parameter('manual_pair_max_m', MANUAL_PAIR_MAX_M)
        self.declare_parameter('target_stale_s', TARGET_STALE_S)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.detect_conf = float(p('detect_conf'))
        self.max_core_gap_mm = float(p('max_core_gap_mm'))
        self.bbox_inner = float(p('bbox_inner'))
        self.show_window = bool(p('show_window'))
        # 계측 — 렉이 '입력이 늦게 오는 것'인지 '노드가 못 따라가는 것'인지 가른다 (README 7절)
        self.diag_log_s = float(p('diag_log_s'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.manual_pair_max_m = float(p('manual_pair_max_m'))
        self.target_stale_s = float(p('target_stale_s'))
        if self.target_stale_s <= 0:
            # 0 이하면 _fresh 가 항상 거짓이라 '아무도 안 보인다' 가 되어 폴백이 통째로 죽는다 — 조용히 먹으면 안 된다.
            # 기동 때는 launch 가 넘긴 yaml 이 틀린 경우라 거절할 상대가 없다. 로그를 남기고 기본값으로 뜬다
            self.get_logger().error(f'target_stale_s={self.target_stale_s} 는 0 보다 커야 한다 — '
                                    f'기본값 {TARGET_STALE_S} 로 진행')
            self.target_stale_s = TARGET_STALE_S
        self.tf_wait = Duration(seconds=float(p('tf_wait_s')))   # 0 이 아니면 실행기가 그만큼 멈춘다 (재현용)
        # camera_info 의 초점거리가 704×704 crop 영상에 안 맞아(I-031) fx·fy 에 곱하는 배율. 기본 16/9.
        # 선언·범위 검증은 core/camera.read_fx_scale 이 한다 — aruco_detector 와 같은 이름·같은 기본값을 쓰기 위해
        self.fx_scale = read_fx_scale(self)
        # 실행 중 파라미터 변경을 실제로 반영한다. 이게 없으면 ros2 param set 이 성공했다고 답하면서 노드는 옛 값을
        # 계속 쓴다 — 파라미터 서버와 노드가 다른 값을 갖는 상태가 되고, 현장에서 '바꿨는데 안 변한다' 가 된다.
        # 선언과 초기 읽기가 다 끝난 뒤에 단다 — 앞에 달면 declare_parameter 도 이 콜백을 타는데 그때는 속성이 없다
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.diag = {'t0': time.monotonic(), 'loops': 0, 'rgb_cb': 0, 'depth_cb': 0, 'synced': 0,
                     'processed': 0, 'age_sum': 0.0, 'age_max': 0.0, 'proc_ms_sum': 0.0,
                     'proc_ms_max': 0.0, 'tf_cb': 0}

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
        # worker_id -> (outfit_class, color_hist). 웹캠1 이 1 Hz 로 재발행
        self.descriptors = {}
        self.paired = False              # 마지막으로 발행한 값 (바뀔 때 즉시 발행)
        self.last_reason = ''

        # 명시 페어링 폴백 (FR-13, 계약 10절) — 게이트 미달이 gate_fail_s 넘게 이어지면 pair_state 로 손을 들고,
        # 운영자가 고른 작업자를 manual_pair 서비스로 받아 확정한다. 수동 확정은 자동 게이트가 뒤집지 못한다 (manual_worker)
        self.tracker = PairStateTracker(float(p('gate_fail_s')))
        self.pair_state = 'NO_TARGET'    # 마지막으로 발행한 값
        self.manual_worker = ''          # 명시 페어링으로 확정된 worker_id. '' 이면 자동 게이트가 정한다
        self.manual_outfit = ''          # 그때 운영자가 가리킨 사람의 복장 클래스 — 이후 이 복장의 후보만 대상으로 삼는다
        self.last_seen_t = None          # 작업자 후보를 마지막으로 본 시각(초) — 서비스가 "지금 보이나" 를 판단
        self.last_seen_cls = ''          # 그 후보(대상 또는 확신도 1등)의 복장 클래스
        self.last_dist = None            # 품질 게이트를 통과한 마지막 거리(m) 와 그 시각 — "1 m 안인가" 를 판단
        self.last_dist_t = None

        # 카메라 입력. 토픽은 상대 이름 — launch 의 namespace 가 앞에 붙는다.
        # ('/robot3' 폴백을 두면 다른 로봇(robot8)이 네임스페이스 없이 떴을 때 robot3 카메라에 몰래 붙는다 → 두지 않는다)
        if DEPTH_TRANSPORT == 'compressedDepth':             # Wi-Fi 대역폭 때문에 depth 는 압축본을 받는다 (기본)
            depth_sub = Subscriber(self, CompressedImage, 'oakd/stereo/image_raw/compressedDepth',
                                   qos_profile=CAMERA_QOS)
        else:
            depth_sub = Subscriber(self, Image, 'oakd/stereo/image_raw', qos_profile=CAMERA_QOS)
        rgb_sub = Subscriber(self, CompressedImage, 'oakd/rgb/image_raw/compressed',
                             qos_profile=CAMERA_QOS)
        rgb_sub.registerCallback(
            lambda m: self.diag.__setitem__('rgb_cb', self.diag['rgb_cb'] + 1))
        depth_sub.registerCallback(
            lambda m: self.diag.__setitem__('depth_cb', self.diag['depth_cb'] + 1))
        # RGB 와 depth 는 별도 토픽으로 따로 도착한다. 타임스탬프가 SYNC_SLOP_S 안으로 맞는 쌍만 synced_cb 로 넘겨,
        # "이 컬러 프레임의 이 픽셀이 저 depth 프레임의 이 거리" 라고 말할 수 있게 한다
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=SYNC_QUEUE,
                                                slop=float(p('sync_slop_s')))
        self.sync.registerCallback(self.synced_cb)
        self.create_subscription(CameraInfo, 'oakd/stereo/camera_info', self.info_cb, 1)

        # 재식별 입력
        self.create_subscription(Mission, MISSION_TOPIC, self.mission_cb, 10)
        self.create_subscription(WorkerDescriptor, DESCRIPTOR_TOPIC, self.descriptor_cb, 10)
        self.create_subscription(Bool, FINISH_TOPIC, self.finish_cb, 10)
        # 명시 페어링 창구. 단일 스레드 실행기라 콜백은 저장된 최근 상태만 보고 즉시 답한다 (fleet_manager 는 2 s 만 기다린다)
        self.create_service(ManualPair, MANUAL_PAIR_SERVICE, self.manual_pair_cb)

        # 출력
        self.pub_point = self.create_publisher(PointStamped, POINT_TOPIC, 1)
        self.pub_image = self.create_publisher(CompressedImage, IMAGE_TOPIC, 1)
        self.pub_visible = self.create_publisher(Bool, VISIBLE_TOPIC, 1)
        self.pub_distance = self.create_publisher(Float32, DISTANCE_TOPIC, 10)   # p95 표본이라 큐를 넉넉히
        self.pub_paired = self.create_publisher(Bool, PAIRED_TOPIC, 10)
        self.pub_worker_id = self.create_publisher(String, WORKER_ID_TOPIC, 10)
        self.pub_pair_state = self.create_publisher(String, PAIR_STATE_TOPIC, 10)
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
        self.get_logger().info(f'YOLO {MODEL_PATH} | ns={self.get_namespace()} | '
                               f'검출 대상 {DETECT_CLASSES} '
                               f'conf≥{self.detect_conf} | reid_threshold={self.gate.threshold} '
                               f'| 출력: {POINT_TOPIC}, {VISIBLE_TOPIC}, {DISTANCE_TOPIC}, '
                               f'{PAIRED_TOPIC}, {WORKER_ID_TOPIC}, {PAIR_STATE_TOPIC} | '
                               f'서비스: {MANUAL_PAIR_SERVICE} '
                               f'(gate_fail_s={self.tracker.fail_s:.0f}, '
                               f'manual_pair_max_m={self.manual_pair_max_m}, '
                               f'target_stale_s={self.target_stale_s})')

    # ── 실행 중 파라미터 변경 ───────────────────────────────────────────────────────────────────────
    def _live_params(self):
        """실행 중 바꿀 수 있는 값 — 이름 → (범위 설명, 검사, 적용). 여기 없는 선언 값은 RESTART_ONLY 다.

        적용이 setattr 로 끝나는 이유는 뜨거운 경로가 전부 속성을 그때그때 읽기 때문이다 — 값만 갈아두면
        다음 프레임부터 새 값으로 돈다. 게이트·트래커도 자기 속성을 매번 읽으므로 같다.
        """
        return {
            'detect_conf': ('0 초과 1 이하', lambda v: 0 < v <= 1,
                            lambda v: setattr(self, 'detect_conf', float(v))),
            'reid_threshold': ('0 이상 1 이하', lambda v: 0 <= v <= 1,
                               lambda v: setattr(self.gate, 'threshold', float(v))),
            'pair_on_s': ('0 초과', lambda v: v > 0, lambda v: setattr(self.gate, 'on_s', float(v))),
            'pair_off_s': ('0 초과', lambda v: v > 0,
                           lambda v: setattr(self.gate, 'off_s', float(v))),
            'gate_fail_s': ('0 초과', lambda v: v > 0,
                            lambda v: setattr(self.tracker, 'fail_s', float(v))),
            'target_stale_s': ('0 초과', lambda v: v > 0,
                               lambda v: setattr(self, 'target_stale_s', float(v))),
            'manual_pair_max_m': ('0 초과', lambda v: v > 0,
                                  lambda v: setattr(self, 'manual_pair_max_m', float(v))),
            'max_core_gap_mm': ('0 초과', lambda v: v > 0,
                                lambda v: setattr(self, 'max_core_gap_mm', float(v))),
            'bbox_inner': ('0 초과 1 이하', lambda v: 0 < v <= 1,
                           lambda v: setattr(self, 'bbox_inner', float(v))),
            'jpeg_quality': ('1 이상 100 이하', lambda v: 1 <= v <= 100,
                             lambda v: setattr(self, 'jpeg_quality', int(v))),
            'diag_log_s': ('0 이상 (0 이면 끔)', lambda v: v >= 0,
                           lambda v: setattr(self, 'diag_log_s', float(v))),
            'tf_wait_s': ('0 이상 (렉 재현용)', lambda v: v >= 0,
                          lambda v: setattr(self, 'tf_wait', Duration(seconds=float(v)))),
        }

    def _on_set_parameters(self, params):
        """ros2 param set 을 받는 자리. 범위 밖이면 **거절**하고, 통과하면 노드가 쓰는 값까지 갈아준다.

        거절이 핵심이다 — 로그만 남기고 넘어가면 파라미터 서버에는 나쁜 값이 들어앉고 노드는 옛 값을 쓴다.
        나중에 ros2 param get 으로 확인한 사람이 노드도 그 값으로 돈다고 믿게 된다.
        """
        live = self._live_params()
        for prm in params:
            if prm.name in RESTART_ONLY:
                return SetParametersResult(successful=False,
                                           reason=f'{prm.name} 은 실행 중에 못 바꾼다 — 노드를 다시 띄운다')
            spec = live.get(prm.name)
            if spec is None:                 # 우리가 선언하지 않은 값(use_sim_time 등)은 건드리지 않는다
                continue
            rng, ok, _ = spec
            if not ok(prm.value):
                return SetParametersResult(successful=False,
                                           reason=f'{prm.name}={prm.value} 는 범위 밖 — {rng} 여야 한다')
        for prm in params:                   # 전부 통과한 뒤에 적용 — 하나라도 틀리면 아무것도 안 바꾼다
            spec = live.get(prm.name)
            if spec is not None:
                spec[2](prm.value)
                self.get_logger().info(f'파라미터 변경: {prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    def _now_s(self):
        """ROS 시각(초). 게이트·pair_state 시계와 서비스의 신선도 판정이 같은 시계를 쓰도록 한 곳에서."""
        return self.get_clock().now().nanoseconds / 1e9

    # ── 콜백: 보관만 한다 ──────────────────────────────────────────────────────────────────────────────
    def info_cb(self, msg):
        """camera_info → K. 카메라 파라미터는 바뀌지 않으니 첫 메시지만 저장한다."""
        if self.K is None:
            # 보정 없이 쓰면 back_project 의 좌우(x) 성분이 1.76 배 크게 나와 추종 목표가 옆으로 밀린다 (I-031)
            self.K = corrected_K(msg, self.fx_scale)
            self.get_logger().info(f'camera_info 수신 — fx {msg.k[0]:.1f} × '
                                   f'{self.fx_scale:.4f} 보정 (I-031)')

    def synced_cb(self, rgb_msg, depth_msg):
        """시각이 맞는 (RGB, depth) 한 쌍 → 디코드해서 self.pair 에 보관만 한다. 계산은 process() 가."""
        depth = (decode_compressed_depth(depth_msg) if isinstance(depth_msg, CompressedImage)
                 else self.bridge.imgmsg_to_cv2(depth_msg,
                                                desired_encoding='passthrough'))   # mm 2차원 배열
        rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)   # JPEG → BGR
        if depth is not None and rgb is not None:
            self.diag['synced'] += 1
            # 시각·frame_id 는 depth 쪽 것을 쓴다 — 거리(3D 점)의 출처가 depth 카메라이기 때문
            self.pair = (rgb, depth, depth_msg.header.stamp, depth_msg.header.frame_id)

    def mission_cb(self, msg):
        """mission/assigned → 내 담당 worker_id. 관제가 로봇마다 따로 보내지만 robot_ns 가 있으면 한 번 더 확인한다."""
        # manual_pair 와 같은 이름 비교 규칙
        if msg.robot_ns and norm_ns(msg.robot_ns) != norm_ns(self.get_namespace()):
            return
        if msg.worker_id and msg.worker_id != self.my_worker_id:
            if self.manual_worker and msg.worker_id != self.manual_worker:
                # 계약 10.6 은 mission/finish 로 푸는 것이 정상 경로다. finish 없이 다른 작업자가 배정되면(유실·순서 뒤바뀜)
                # 옛 잠금을 들고 새 미션을 도는 것이 더 나쁘다 — 잠금을 풀고 새 미션의 자동 게이트부터 다시
                self.get_logger().warn(f'새 미션 {msg.worker_id} — 명시 페어링 '
                                       f'{self.manual_worker} 해제 (mission/finish 없이 재배정)')
                self.manual_worker = self.manual_outfit = ''
                self.tracker.reset()
                self._set_pair_state('NO_TARGET', publish=False)
                self._set_paired(False, 'reassigned')
            self.my_worker_id = msg.worker_id
            self.get_logger().info(f'미션 배정: 내 담당 = {self.my_worker_id} (mission {msg.mission_id})')
            self._bind()

    def descriptor_cb(self, msg):
        """/webcam1/worker_descriptor → worker_id 별 (복장, 색 지문). 웹캠1 이 1 Hz 로 재발행하므로 늦게 켜져도 받는다."""
        if not msg.worker_id or not msg.color_hist:
            return
        self.descriptors[msg.worker_id] = (msg.outfit_class,
                                           np.asarray(msg.color_hist, np.float32))
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

    def finish_cb(self, msg):
        """mission/finish → 페어링 해제. 수동 확정도 여기서만 풀린다 (계약 10.6, FR-30). 다음 미션은 자동 게이트부터."""
        if not msg.data:
            return
        self.get_logger().info(f'미션 종료 — 페어링 해제 (manual={self.manual_worker!r}, '
                               f'worker={self.my_worker_id!r})')
        self.manual_worker = self.manual_outfit = ''
        self.my_worker_id = ''
        self.gate.reset(forget=True)     # 서술자까지 잊는다 — 다음 mission/assigned 전에는 짝을 만들지 않는다 (캐시는 남음)
        self.tracker.reset()
        self._set_pair_state('NO_TARGET', publish=False)
        self.paired, self.last_reason = False, 'mission finished'
        self.publish_pairing()           # worker_id·paired·pair_state 를 한 번에 — 서로 어긋난 순간이 없게

    # ── 출력 ───────────────────────────────────────────────────────────────────────────────────────
    def publish_pairing(self, force=False):
        """worker_id·paired·pair_state 발행. 타이머(1 Hz)와 값이 바뀐 순간 둘 다에서 부른다."""
        # 프레임이 끊기면 process() 가 안 돌아 pair_state 가 굳는다 — 후보를 못 본 지 target_stale_s 가 지났으면 여기서
        # 시계를 돌려 NO_TARGET 으로 내린다 (관제 버튼은 떠 있는데 요청은 NO_TARGET 으로 거절되는 모순 방지)
        if not self.paired and not self._fresh(self.last_seen_t, self._now_s()):
            self._set_pair_state(
                self.tracker.update(self._now_s(), False, False, self.gate.ready()), publish=False)
        wid = self.manual_worker or self.my_worker_id
        # worker_id 를 먼저 — fleet_manager 는 paired=True 를 받는 순간 마지막 worker_id 를 읽는다. 토픽이 달라 DDS 가
        # 순서를 보장하진 않지만(그쪽은 DB 의 미션 worker_id 로 보완), 같은 순간의 세 값이 한 묶음으로 나가는 것이 요점
        self.pub_worker_id.publish(String(data=wid if self.paired else ''))
        self.pub_paired.publish(Bool(data=self.paired))
        self.pub_pair_state.publish(String(data=self.pair_state))

    def _set_paired(self, paired, reason):
        """짝 여부(paired)를 갱신한다. 바뀌면 세 값을 함께 발행하고 True 를 돌려준다 — pair_state 는 이 전에 맞춰 두어야 한다."""
        changed = paired != self.paired
        if changed:
            self.paired = paired
            self.get_logger().info(
                f'paired={paired} ({self.manual_worker or self.my_worker_id}) — {reason}')
            self.publish_pairing()                           # 바뀐 순간 즉시. 타이머는 재발행용
        self.last_reason = reason
        return changed

    def _set_pair_state(self, state, publish=True):
        """pair_state 갱신. 바뀌면 True. publish=False 면 발행은 뒤따르는 publish_pairing 에 맡긴다 (세 값 한 묶음)."""
        if state == self.pair_state:
            return False
        self.pair_state = state
        if publish:
            self.pub_pair_state.publish(String(data=state))
        if state == 'GATE_FAIL':
            self.get_logger().warn(f'★ 게이트 미달 {self.tracker.fail_s:.0f} s — pair_state=GATE_FAIL. '
                                   f'관제 UI 에 명시 페어링 버튼이 뜬다 (FR-13)')
        else:
            self.get_logger().info(f'pair_state={state}')
        return True

    def _fresh(self, t_ref, t):
        """t_ref 가 target_stale_s 안의 최근 값인가 — 서비스가 '지금' 을 판단하는 기준."""
        return t_ref is not None and t - t_ref <= self.target_stale_s

    # ── 명시 페어링 폴백 (FR-13, 계약 10절) ─────────────────────────────────────────────────────────
    def manual_pair_cb(self, req, res):
        """운영자가 고른 (robot_ns, worker_id) 를 받는다 — /robotN/manual_pair.

        받을지는 core.reid.manual_pair_decision 이 정한다.

        수락하면 그 자리에서 paired=True·worker_id 를 발행한다 — 발행자는 이 노드 하나뿐이다 (계약 10.4).
        판정 재료는 process() 가 남긴 최근 상태(마지막으로 본 시각·거리·게이트 결과)라 기다리는 일이 없다.
        """
        t = self._now_s()
        visible = self._fresh(self.last_seen_t, t)
        dist = self.last_dist if self._fresh(self.last_dist_t, t) else None
        ok, reason = manual_pair_decision(req.robot_ns, self.get_namespace(), req.worker_id,
                                          self.my_worker_id, self.paired, visible, dist,
                                          self.manual_pair_max_m, self.gate.last_ok)
        res.accepted, res.reason = ok, reason
        if ok:
            # 운영자가 가리킨 "지금 보이는 사람" 의 복장을 기억한다 — 이후 그 복장의 후보만 대상 (다른 복장에 뺏기지 않게)
            self.manual_worker, self.manual_outfit = req.worker_id, self.last_seen_cls
            self.tracker.update(t, True, True)
            self._set_pair_state('PAIRED', publish=False)
            # worker_id·paired·pair_state 를 한 묶음으로
            self._set_paired(True, f'manual pair {req.worker_id}')
            self.get_logger().warn(f'★ 명시 페어링 수락: {req.worker_id} (거리 {dist:.2f} m) — 운영자 결정. '
                                   f'mission/finish 까지 유지')
        else:
            self.get_logger().warn(f'명시 페어링 거부: {reason} '
                                   f'(robot_ns={req.robot_ns!r} worker_id={req.worker_id!r} '
                                   f'paired={self.paired} visible={visible} dist={dist})')
        return res

    def _bbox_distance(self, depth, bx, frame_id, stamp, t_now):
        """검출 영역 하나로 (r, pt, dist) 를 낸다. ⑤ 거리 → ⑥ 품질 게이트 → ⑦ 3D 점·직선거리.

        게이트 탈락이면 pt·dist 는 None.

        target/distance 와 명시 페어링의 1 m 판정(FR-14)이 **같은 자**를 쓰도록 계산은 여기 한 곳에만 있다.
        통과한 거리는 last_dist 에 남긴다 — manual_pair 서비스가 "1 m 안인가" 를 볼 재료.
        """
        x1, y1, x2, y2 = bx[:4]
        # ⑤ 거리 — bbox 중심부(BBOX_INNER) depth 의 p20(q)·중앙값(med)·보정 거리(z_corr, m)
        r = distance_from_bbox(depth, x1, y1, x2, y2, inner_ratio=self.bbox_inner)
        # ⑥ 품질 게이트 — p20 과 med 가 MAX_CORE_GAP_MM 이상 벌어지면 가림·경계 걸침이다. 이 프레임 거리는 버린다
        if r is None or (r['med'] - r['q']) > self.max_core_gap_mm:
            return r, None, None
        # ⑦ bbox 중심 픽셀 (u,v) + 정면 거리 z → 카메라 광학 좌표의 3D 점 (X 우, Y 하, Z 전방)
        pt = back_project(self.K, (x1 + x2) // 2, (y1 + y2) // 2, r['z_corr'], frame_id, stamp)
        # 거리는 이 3D 점의 노름 = 직선거리. depth 의 z 는 "정면 방향" 거리라 화면 가장자리에서는
        # 직선거리보다 최대 20 % 짧다 — 줄자(TS-U05)·추종거리(AC-05)는 직선거리다
        dist = math.hypot(pt.point.x, pt.point.y, pt.point.z)
        self.last_dist, self.last_dist_t = dist, t_now
        return r, pt, dist

    # ── 메인 처리 ────────────────────────────────────────────────────────────────────────────────────
    def process(self):
        """메인 루프가 매번 호출. 보관된 최신 한 쌍을 ①~⑧ 순서로 처리하고 소비한다."""
        self.diag['loops'] += 1
        self._diag_log()
        if self.pair is None:                                # 새 프레임이 없으면 할 일 없음
            return
        rgb, depth, stamp, frame_id = self.pair
        t_proc = time.perf_counter()
        now_ns = self.get_clock().now().nanoseconds
        t_now = now_ns / 1e9                                 # 게이트·pair_state·신선도 판정이 같은 시각을 쓴다
        age = (now_ns - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9   # 카메라 찍힘 → 지금
        self.diag['age_sum'] += age
        self.diag['age_max'] = max(self.diag['age_max'], age)
        self.pair = None                                     # 소비 표시 — 같은 프레임을 두 번 처리하지 않는다
        vis = rgb.copy()                                     # 오버레이는 복사본에 그린다 (원본은 YOLO 입력)
        label = 'detecting' if self.K is not None else 'waiting for camera_info'

        if self.K is not None:                               # K 가 없으면 3D 변환을 못 하니 검출도 건너뛴다
            # ① YOLO — 작업자 후보 전부 (확신도 내림차순)
            boxes = worker_boxes(self.model(rgb, verbose=False), self.names,
                                 DETECT_CLASSES, self.detect_conf)

            # ②③ 게이트 — 서술자·미션이 있으면 "내 담당" 을 고른다. 없으면(단독 시험·S1 전) 확신도 1등을 대상으로
            top = boxes[0] if boxes else None            # 게이트가 고르지 않았을 때의 대상 = 확신도 1등
            if self.gate.ready():
                cands = []
                for bx in boxes:
                    x1, y1, x2, y2 = bx[:4]
                    crop = rgb[max(0, y1):y2, max(0, x1):x2]     # 웹캠1 과 같은 규칙 — bbox 전체 (계약 v1.3)
                    if crop.size:
                        cands.append((bx[5], hsv_hist(crop), bx))
                paired, target, sim, reason = self.gate.update(t_now, cands)
                tag = f'sim={sim:.2f} {"PAIRED" if paired else "unpaired"}'
            else:
                paired, target, reason = False, top, 'no descriptor'
                tag = 'no descriptor (unpaired)'
            if self.manual_worker:                       # 운영자가 확정한 짝 — 자동 게이트가 뒤집지 못한다 (계약 10.5)
                # 대상은 운영자가 가리킨 복장의 후보만. 게이트가 고른 것이 그 복장이면 그대로, 아니면 그 복장의 확신도 1등.
                # 복장이 다른 사람은 확신도가 높아도 따르지 않는다 — 아니면 다른 사람이 들어올 때마다 대상이 튄다
                if target is None or target[5] != self.manual_outfit:
                    pool = [b for b in boxes if b[5] == self.manual_outfit]
                    target = pool[0] if pool else None
                paired, reason, tag = True, f'manual pair {self.manual_worker}', 'MANUAL PAIRED'
            seen = target or top
            if seen is not None:                         # 명시 페어링 서비스가 "지금 보이나·누구인가" 를 판단하는 재료
                self.last_seen_t, self.last_seen_cls = t_now, seen[5]
            # ③' 폴백 상태 (FR-13) — 후보가 보이는데 못 넘는 시간을 재서 gate_fail_s 를 넘기면 관제에 손을 든다.
            # pair_state 를 먼저 맞추고 _set_paired 가 세 값을 한 묶음으로 낸다 — paired 와 pair_state 가 어긋난 순간이 없게
            state_changed = self._set_pair_state(
                self.tracker.update(t_now, bool(boxes), paired, self.gate.ready()), publish=False)
            if not self._set_paired(paired, reason) and state_changed:
                self.pub_pair_state.publish(String(data=self.pair_state))
            if target is None and top is not None:
                # 게이트가 후보를 안 골랐다(복장 불일치) — 폴백이 필요한 바로 그 상황. 운영자가 가리킬 사람은 지금 보이는
                # 확신도 1등이니 그 거리를 재 두어야 manual_pair 의 1 m 판정(FR-14)이 된다. 발행은 하지 않는다
                self._bbox_distance(depth, top, frame_id, stamp, t_now)

            # ④ "보이나" 는 거리 품질과 무관하게 매 프레임 발행. kpi_logger 가 1→0 (1 s 이상) 을 Lost 로 센다 (K-04)
            self.pub_visible.publish(Bool(data=target is not None))

            for bx in boxes:                                 # 후보 전부 얇게 그려서 무엇이 보였는지 남긴다
                cv2.rectangle(vis, bx[:2], bx[2:4], (120, 120, 120), 1)
            if target:
                x1, y1, x2, y2, c, cls_name = target
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # ⑤⑥⑦ 거리 → 품질 게이트 → 3D 점·직선거리 (_bbox_distance 한 곳에서. 폴백 판정도 같은 자)
                r, pt, dist = self._bbox_distance(depth, target, frame_id, stamp, t_now)
                self.get_logger().info(f'{cls_name} conf={c:.2f} {tag} | {format_result(r)}')
                if pt is not None:
                    # TF 와 무관하게 여기서 낸다 (map 없이도 줄자 대조가 되게). 품질 게이트 통과분만 — p95 오염 방지
                    self.pub_distance.publish(Float32(data=dist))
                    # 같은 점을 map 좌표로. timeout=0 — 기다리면 실행기가 멈춰 /tf 가 못 들어오므로(단일 스레드)
                    # 즉시 판정. 없으면 이 프레임만 건너뛴다. 1 s 대기 두 번이 탐지 프레임마다 2 s 렉을 만들었다 (9/8)
                    pm = to_map(self.tf_buffer, pt, self.get_logger(), timeout=self.tf_wait)
                    if pm is not None:                       # TF 가 아직 없으면(기동 직후·시계 어긋남) 건너뛴다
                        # ⑧ follow_target 이 받아 "이 앞 0.5 m" 를 목표로 삼는다 (paired 일 때만 쓴다 — 주행 TODO ④)
                        self.pub_point.publish(pm)
                        label = (f'{cls_name} {dist:.2f} m -> map '
                                 f'({pm.point.x:.2f}, {pm.point.y:.2f}) {tag}')
                    else:
                        label = f'{cls_name} {dist:.2f} m (no TF) {tag}'
                elif r is not None:
                    label = (f"core gap {r['med'] - r['q']:.0f} mm > "
                             f'{self.max_core_gap_mm:.0f} - rejected')
                cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
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
            # 원본 프레임 시각 — ros2 topic delay 로 지연을 잴 수 있게
            msg.header.stamp = stamp
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
            f"depth {d['depth_cb'] / dt:.1f} tf {d['tf_cb'] / dt:.0f}/s | "
            f"synced {d['synced'] / dt:.1f}/s "
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
