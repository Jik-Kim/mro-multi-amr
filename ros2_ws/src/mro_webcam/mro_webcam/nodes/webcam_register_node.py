#!/usr/bin/env python3
"""웹캠1 작업자 등록 노드 (S1) — PC-W1.

출입구 시야에서 작업자를 추적하고, 외형 서술자를 붙여 /webcam1/worker_descriptor 로 발행한다.
fleet_manager 가 이를 받아 작업자-로봇을 배정하고 언독(S2)으로 이어간다.

  카메라 → YOLO.track(ByteTrack) → 트랙 ID → bbox crop
        → reid.hsv_hist (H16×S8=128) → W1/W2 배정 → WorkerDescriptor 발행(1 Hz)

계약 (docs/interfaces.md)
  - color_hist 는 mro_perception.core.reid.hsv_hist() 를 **그대로 호출**해야 AMR 재식별과 비교가 성립한다
  - 시간 상수는 프레임 수가 아니라 초로 둔다 (DETECT_S / CLEAR_S / REGISTER_S)
  - 카메라 토픽은 BEST_EFFORT(sensor_data), 등록 결과는 RELIABLE
  - 웹캠은 안전 판단에 관여하지 않는다

주의
  - 캡처 fps > 처리 Hz 라 큐에 오래된 프레임이 남는다 → CAP_PROP_BUFFERSIZE=1 (I-016)
  - rematch_threshold 는 잠정값이다. 인형 2체로 유사도 분포를 재서 확정한다 (계약 v1.3)
  - 자동 노출이 길어지면 캡처가 처리 주기 밑으로 떨어져 루프까지 느려진다
  - 셔틀 트리거는 mission/assigned 가 계약이다 (FR-07). 옛 webcam/detected 발행은 제거했다
  - 웹캠1 은 출입구만 본다 (9/7 결정, 계약 v1.10). 관제탑 구역은 시야 밖이고 map 좌표도 내지 않는다.
    등록은 작업자가 출입구를 지나는 순간에 일어나므로 통과 시간이 register_s 를 채우는지 확인할 것 (Q-26)

등록 구역 게이트는 두지 않는다 — 시야 전체가 곧 등록 구역이다 (Q-26)

실행
  ros2 launch mro_bringup webcam1_pc.launch.py
  ros2 run mro_webcam webcam_register_node --ros-args -p model_path:=<worker.pt 경로>
확인
  ros2 topic echo /webcam1/worker_descriptor
"""
import os

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from mro_interfaces.msg import WorkerDescriptor
from mro_perception.core.reid import hsv_hist, similarity   # 계약: AMR(amr_detection) 과 반드시 같은 함수

# 모델은 mro_perception 이 단일 출처다 (models/README.md, PR #19). 사본을 두면 한쪽만 갱신된다
PERCEPTION_SHARE = get_package_share_directory('mro_perception')
# ############################################################################
MODEL_PATH = os.path.join(PERCEPTION_SHARE, 'models', 'worker.pt')
WEBCAM_INDEX = -1            # -1: 자동 탐색(높은 번호부터). 파라미터라 None 을 못 써서 -1 을 쓴다.
                             # 내장 카메라가 있는 PC 는 자동 탐색이 그걸 고를 수 있다 — 그때 번호를 준다
CONF = 0.5                   # 인지팀 perception.yaml detect_conf 와 같은 값.
                             # F1 최적은 0.4 지만 오탐 = 오추종이고 AC-04 는 0건이 절대 기준이다
PERIOD_S = 0.1               # 처리 주기 (10 Hz). 히스테리시스를 프레임 수로 환산하는 기준
DETECT_S = 0.3               # 이 시간 동안 연속으로 보이면 True   (10 Hz 기준 3프레임)
CLEAR_S = 0.5                # 이 시간 동안 연속으로 안 보이면 False (10 Hz 기준 5프레임)
REGISTER_S = 1.0             # 트랙이 이 시간 연속 유지되면 등록 확정 (TS-U01: 등록 <= 10 s)
PUBLISH_HZ = 1.0             # 등록된 작업자 재발행 주기. 관제가 늦게 켜져도 받게 한다
MAX_WORKERS = 2              # SOT 확정: 작업자 2명
TARGET_CLASSES = ['worker_a', 'worker_b']   # 이 클래스만 등록 대상. 빈 리스트면 전체 허용
                                            # (테스트: -p target_classes:="['person']")
REMATCH_THRESHOLD = 0.45     # 끊긴 트랙이 기존 등록자와 같은 사람인지 판정 (등록 슬롯 재사용).
                             # robotN.yaml 의 reid_threshold(로봇의 페어링 허용 판정)와는 다른 결정이라
                             # 이름을 분리한다. 같게 두면 나중에 값을 통일해 버려 한쪽이 깨진다.
                             # 9/7 실측으로 확정 (tools/reid_threshold.py, 계약 v1.3 '눈대중 금지').
                             #   같은 인형이 시야에 다시 들어왔을 때 sim 0.52~0.86 (n=6, 최솟값 0.52)
                             #   종전 0.70 은 재등록 기회 5회 중 1회만 성공했다 — 게다가 실패해도
                             #   REG 2/2 가 그대로 떠 있고 발행도 계속돼 겉으로 드러나지 않는다
                             # 낮춰서 생기는 위험(다른 인형을 같은 사람으로 봄)은 두 겹으로 막혀 있다:
                             #   ① 같은 복장 클래스끼리만 비교한다 (아래 outfit != cls_name)
                             #   ② 복장 상호 오분류 0/410 = 0.00 % (인지팀 실측 9/7, conf 0.5)
SHOW_WINDOW = True
# ############################################################################


def open_webcam(index):
    # 자동 탐색은 높은 번호부터: 노트북 내장 카메라가 0~3 을 차지하고 나중에 꽂은 USB 웹캠이 그 뒤 번호를 받음
    for i in ([index] if index is not None else range(9, -1, -1)):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # 캡처 25 fps > 처리 10 Hz 라 큐에 오래된 프레임이 남는다. 1 로 제한 (실측 0.16 s → 0.04 s)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap, i
            cap.release()
    return None, None


class WebcamDetection(Node):
    def __init__(self):
        super().__init__('webcam_register')   # launch·문서·파라미터 파일과 이름을 맞춘다
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('target_classes', TARGET_CLASSES)
        self.declare_parameter('rematch_threshold', REMATCH_THRESHOLD)
        self.declare_parameter('webcam_index', WEBCAM_INDEX)
        self.declare_parameter('conf', CONF)
        model_path = self.get_parameter('model_path').value or MODEL_PATH   # launch 가 빈 값을 넘길 수 있다
        self.target_classes = list(self.get_parameter('target_classes').value)
        self.rematch_threshold = float(self.get_parameter('rematch_threshold').value)
        self.conf = float(self.get_parameter('conf').value)
        index = int(self.get_parameter('webcam_index').value)
        self.model = YOLO(model_path)
        self.names = self.model.names
        self.bridge = CvBridge()
        # 모니터링용. 계약상 카메라 토픽은 sensor_data(BEST_EFFORT) — 밀리면 버리고 발행자를 막지 않는다
        # 모니터링용. raw 900 KB x 10 Hz = 9 MB/s 는 Wi-Fi 를 막는다 → JPEG 약 50 KB (I-016, NR-03)
        self.pub_img = self.create_publisher(CompressedImage, 'webcam/image/compressed',
                                             qos_profile_sensor_data)
        # S1 등록 결과. 계약상 명령·상태·미션은 RELIABLE (기본값) — 버리면 안 된다
        self.pub_desc = self.create_publisher(WorkerDescriptor, '/webcam1/worker_descriptor', 10)

        self.cap, idx = open_webcam(None if index < 0 else index)
        if self.cap is None:
            raise RuntimeError('웹캠을 못 열었음. v4l2-ctl --list-devices 로 번호 확인 후 '
                               '-p webcam_index:=<번호> 로 지정한다')
        self.get_logger().info(f'웹캠 /dev/video{idx} | 모델 {model_path} classes={list(self.names.values())} '
                               f'| conf={self.conf} | 대상 클래스 {self.target_classes} '
                               f'| {DETECT_S}s 연속 → True, {CLEAR_S}s 미검출 → False')

        self.detected = False
        self.hit = self.miss = 0
        self.tracks = []          # (x1,y1,x2,y2,conf,cls_name,track_id) — 다음 단계(서술자·등록)에서 쓴다
        self.hists = {}           # track_id -> color_hist (float32[128]). interfaces.md: H16 x S8
        self.track_seen = {}      # track_id -> 연속 관측 프레임 수
        self.workers = {}         # track_id -> (worker_id, outfit_class)
        self.last_logged = None
        self.create_timer(PERIOD_S, self.process_frame)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_workers)

    def process_frame(self):
        ok, img = self.cap.read()
        if not ok:
            self.get_logger().warn('웹캠 프레임 실패', throttle_duration_sec=5.0)
            return

        # ByteTrack 으로 프레임 간 동일 대상에 같은 track_id 를 유지한다 (worker_id 부여의 전제)
        tracks = []
        best = None
        r = self.model.track(img, persist=True, conf=self.conf, verbose=False)[0]
        if r.boxes is not None and r.boxes.id is not None:
            for box, tid in zip(r.boxes, r.boxes.id):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                c = float(box.conf[0])
                cls_name = self.names.get(int(box.cls[0]), '?')
                if self.target_classes and cls_name not in self.target_classes:
                    continue          # 대상 아닌 클래스가 등록 자리를 차지하지 않게 한다
                tracks.append((x1, y1, x2, y2, c, cls_name, int(tid)))
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, f'ID{int(tid)} {cls_name} {c:.2f}', (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                if best is None or c > best[4]:
                    best = (x1, y1, x2, y2, c)
        self.tracks = tracks

        # 트랙별 색 서술자. bbox crop -> HSV 2D 히스토그램(길이 128)
        # crop 은 bbox 전체를 쓴다 — 계약 v1.3(9/6) 이 '인형 머리부터 바지 밑단까지' 로 확정했다.
        # 상의만 자르면 웹캠1 거리에서 픽셀이 부족해 히스토그램이 뭉개진다 (I-015)
        for x1, y1, x2, y2, _c, _cls, tid in tracks:
            crop = img[max(0, y1):y2, max(0, x1):x2]
            if crop.size == 0:
                continue
            h = hsv_hist(crop)
            if tid not in self.hists:
                self.get_logger().info(f'서술자 생성 ID{tid} len={len(h)}')
            self.hists[tid] = h

        # 등록 판정: 트랙이 REGISTER_S 동안 끊기지 않고 보이면 worker_id 를 배정한다
        alive = {t[6] for t in tracks}
        for tid in alive:
            self.track_seen[tid] = self.track_seen.get(tid, 0) + 1
        for tid in list(self.track_seen):
            if tid not in alive:
                del self.track_seen[tid]          # 끊기면 처음부터 다시 센다
        need = round(REGISTER_S / PERIOD_S)
        for _x1, _y1, _x2, _y2, _c, cls_name, tid in tracks:
            if tid in self.workers or self.track_seen.get(tid, 0) < need:
                continue
            h = self.hists.get(tid)
            # 추적이 끊기면 같은 사람도 새 track_id 를 받는다. 서술자로 기존 등록자인지 먼저 확인해
            # 한 사람이 W1·W2 를 모두 차지하는 것을 막는다 (TS-U01: 2명 별개 ID)
            rebound = None
            if h is not None:
                for old_tid, (wid, outfit) in self.workers.items():
                    old = self.hists.get(old_tid)
                    if old is None or outfit != cls_name:
                        continue
                    sim = similarity(old, h)
                    if sim >= self.rematch_threshold:
                        rebound = (old_tid, wid, sim)
                        break
            if rebound:
                old_tid, wid, sim = rebound
                self.workers[tid] = self.workers.pop(old_tid)      # 트랙만 갈아끼운다
                self.get_logger().info(f'재등록 {wid}: track {old_tid} → {tid} (sim {sim:.2f})')
                continue
            if len(self.workers) >= MAX_WORKERS:
                continue
            wid = f'W{len(self.workers) + 1}'
            self.workers[tid] = (wid, cls_name)
            self.get_logger().info(f'등록 확정 {wid} (track {tid}, {cls_name})')

        # 히스테리시스: 초 단위 상수를 처리 주기로 나눠 프레임 수로 환산.
        # webcam/detected 발행은 제거했고(FR-07 은 mission/assigned 가 계약), 지금은 로그 전이 판정에 쓴다
        if best:
            self.hit += 1
            self.miss = 0
        else:
            self.miss += 1
            self.hit = 0
        if not self.detected and self.hit >= round(DETECT_S / PERIOD_S):
            self.detected = True
        elif self.detected and self.miss >= round(CLEAR_S / PERIOD_S):
            self.detected = False

        # 현재 등록 상태. cv2.putText 는 한글 글리프가 없어 화면용은 ASCII 로 따로 만든다
        joined = ', '.join(f'{w}({o})' for w, o in self.workers.values())
        label = f'등록 {len(self.workers)}/{MAX_WORKERS}: {joined}' if self.workers else '등록 없음'
        overlay = f'REG {len(self.workers)}/{MAX_WORKERS}: {joined}' if self.workers else 'REG 0/%d' % MAX_WORKERS

        if self.detected != self.last_logged:
            self.get_logger().info(('🟢 ' if self.detected else '⚪ ') + label)
            self.last_logged = self.detected

        cv2.putText(img, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(img))
        if SHOW_WINDOW:
            cv2.imshow('WebCam Detection', img)
            cv2.waitKey(1)

    def publish_workers(self):
        """등록된 작업자를 PUBLISH_HZ 로 재발행. 관제(fleet_manager)가 늦게 켜져도 받게 한다."""
        for tid, (wid, outfit) in self.workers.items():
            h = self.hists.get(tid)
            if h is None:
                continue
            m = WorkerDescriptor()
            m.header.stamp = self.get_clock().now().to_msg()   # 계약: 모든 로그·메시지는 ROS 시각
            m.worker_id = wid
            m.outfit_class = outfit
            m.color_hist = [float(v) for v in h]
            m.robot_ns = ''                                     # fleet_manager 가 채운다
            self.pub_desc.publish(m)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
