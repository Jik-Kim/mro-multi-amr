#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/webcam_detection_node.py → ros2_ws/src/mro_webcam/mro_webcam/nodes/webcam_register_node.py
역할 변경: 'car 검출 → 출발 신호' 에서 '작업자 등록(S1)' 으로.
TODO([웹캠]): ① YOLO track(ByteTrack) 으로 트랙 ID 부여 ② 트랙별 복장 클래스 + mro_perception.core.reid.hsv_hist 서술자 생성
③ mro_interfaces/WorkerDescriptor 를 /webcam1/worker_descriptor 로 발행 (등록 ≤ 10 s, 2명 별개 ID)
④ 기존 webcam/detected · webcam/car_map_point 발행은 호환용으로 남기되 fleet_manager 가 미션 생성 트리거로 쓴다.
"""
"""
webcam_detection_node.py — [WebCam Detection]  (팀원 yolo_publisher_wc_det.py 기반, 우리 계약에 맞춘 버전)

원본: yolo/webcam/yolo_publisher_wc_det.py (그대로 보관). 여기서 달라진 곳은 전부 [추가]/[수정] 표시.
  [수정] 모델: yolo/webcam/best.pt (클래스 car 하나뿐) — MODEL_PATH 고정
  [수정] 웹캠 번호 WEBCAM_INDEX (None 이면 0~4 자동)
  [수정] CONF 0.7 (원본 주석의 오탐 실측 근거: 0.25 → 프레임당 ~4개, 0.8 → 0.3개)
  [수정] 토픽을 계약 이름으로: webcam/detected (Bool)   ← 원본 object_detected
  [추가] webcam/car_map_point (PointStamped, map) — nav/webcam_homography.json 이 있으면 bbox 아래중앙 → map
  [삭제] output/ 에 jpg·csv 저장 (검출 프레임마다 디스크에 쓰던 것)

실행:  cd ~/Kn1_mini/nav && python3 webcam_detection_node.py --ros-args -r __ns:=/robot3
확인:  ros2 topic echo /robot3/webcam/detected   /   ros2 topic echo /robot3/webcam/car_map_point
"""
import os

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from mro_webcam.core.homography import DEFAULT_JSON, bbox_to_map, load_homography

SHARE = get_package_share_directory('mro_webcam')
# ############################################################################
MODEL_PATH = os.path.join(SHARE, 'models', 'best.pt')  # TODO: worker.pt 로 교체
WEBCAM_INDEX = None          # None: 0~4 중 프레임 나오는 첫 장치. 외장 웹캠은 보통 1 또는 2
CONF = 0.7                   # 단일 클래스 모델이라 오탐 많음 → 높게
DETECT_FRAMES = 3            # 연속 N프레임 보이면 True
CLEAR_FRAMES = 5             # 연속 N프레임 안 보이면 False
SHOW_WINDOW = True
# ############################################################################


def open_webcam(index):
    # 자동 탐색은 높은 번호부터: 노트북 내장 카메라가 0~3 을 차지하고 나중에 꽂은 USB 웹캠이 그 뒤 번호를 받음
    for i in ([index] if index is not None else range(9, -1, -1)):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap, i
            cap.release()
    return None, None


class WebcamDetection(Node):
    def __init__(self):
        super().__init__('webcam_detection')
        self.model = YOLO(MODEL_PATH)
        self.names = self.model.names
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, 'webcam/image', 1)
        self.pub_det = self.create_publisher(Bool, 'webcam/detected', 10)
        self.pub_pt = self.create_publisher(PointStamped, 'webcam/car_map_point', 10)

        # [추가] 호모그래피 (없으면 좌표 발행 생략, 트리거만)
        self.H = None
        if os.path.exists(DEFAULT_JSON):
            self.H = load_homography(DEFAULT_JSON)
            self.get_logger().info(f'호모그래피 로드: {DEFAULT_JSON}')
        else:
            self.get_logger().warn(f'호모그래피 없음 ({DEFAULT_JSON}) — car_map_point 발행 안 함. '
                                   f'webcam_homography.py calib 로 생성')

        self.cap, idx = open_webcam(WEBCAM_INDEX)
        if self.cap is None:
            raise RuntimeError('웹캠을 못 열었음. v4l2-ctl --list-devices 로 번호 확인 후 WEBCAM_INDEX 설정')
        self.get_logger().info(f'웹캠 /dev/video{idx} | 모델 {MODEL_PATH} classes={list(self.names.values())} '
                               f'| conf={CONF} | {DETECT_FRAMES}프레임 연속 → True')

        self.detected = False
        self.hit = self.miss = 0
        self.last_logged = None
        self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ok, img = self.cap.read()
        if not ok:
            self.get_logger().warn('웹캠 프레임 실패', throttle_duration_sec=5.0)
            return

        best = None
        for r in self.model(img, stream=True, conf=CONF, verbose=False):
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                c = float(box.conf[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, f'{self.names.get(int(box.cls[0]), "?")} {c:.2f}', (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                if best is None or c > best[4]:
                    best = (x1, y1, x2, y2, c)

        # N프레임 연속 조건 (원본 로직 그대로)
        if best:
            self.hit += 1
            self.miss = 0
        else:
            self.miss += 1
            self.hit = 0
        if not self.detected and self.hit >= DETECT_FRAMES:
            self.detected = True
        elif self.detected and self.miss >= CLEAR_FRAMES:
            self.detected = False
        self.pub_det.publish(Bool(data=self.detected))

        # [추가] 차 위치 → map (연속 조건을 통과해 detected 일 때만 발행·표시)
        label = 'car' if self.detected else 'no car'
        if best and self.H is not None and self.detected:
            x1, y1, x2, y2, _ = best
            mx, my = bbox_to_map(self.H, x1, y1, x2, y2)
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = 'map'
            pt.point.x, pt.point.y = mx, my
            self.pub_pt.publish(pt)
            cv2.circle(img, ((x1 + x2) // 2, y2), 5, (0, 255, 0), -1)
            label += f' -> map ({mx:.2f}, {my:.2f})'

        if self.detected != self.last_logged:
            self.get_logger().info(('🟢 ' if self.detected else '⚪ ') + label)
            self.last_logged = self.detected

        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        if SHOW_WINDOW:
            cv2.imshow('WebCam Detection', img)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            self.pub_det.publish(Bool(data=False))   # 종료 시 탐지 상태로 남지 않게 (원본 로직)
        except Exception:
            pass
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
