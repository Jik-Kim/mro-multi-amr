"""웹캠2 작업자 추적 및 map 좌표 힌트 노드.

사용처
  - 입력: USB 웹캠2, /webcam1/worker_descriptor, homography_cam2.json
  - 처리: YOLO 추적 → bbox 하단 중앙 → SLAM map 좌표 변환
  - 출력: /webcam2/target_hint (kpi_logger, follow_target)

웹캠2 정보는 재탐지 힌트와 KPI 계측에만 사용하고 안전 판단에는 쓰지 않는다.
"""
import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from mro_interfaces.msg import TargetHint, WorkerDescriptor
from mro_webcam.core.homography import bbox_to_map, load_homography


SHARE = get_package_share_directory('mro_webcam')
DEFAULT_MODEL_PATH = os.path.join(SHARE, 'models', 'best.pt')
DEFAULT_CLASSES = ['worker_a', 'worker_b']


@dataclass(frozen=True)
class CameraConfig:
    """카메라 장치 설정."""

    index: int
    width: int
    height: int
    fps: float
    buffer_size: int


@dataclass(frozen=True)
class NodeConfig:
    """웹캠 힌트 노드의 실행 설정."""

    camera: CameraConfig
    process_hz: float
    image_publish_hz: float
    compressed_topic: str
    image_frame_id: str
    jpeg_quality: int
    model_path: str
    homography_path: str
    confidence: float
    target_classes: list
    detection_confirm_s: float
    detection_hold_s: float
    lost_confirm_s: float
    hint_valid_s: float
    show_window: bool


class LatestFrameCamera:
    """V4L2 큐를 계속 비우며 최신 프레임 하나만 보관한다."""

    # 카메라 장치와 최신 프레임 공유 자원을 준비한다.
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.capture = cv2.VideoCapture(config.index, cv2.CAP_V4L2)
        self._configure()
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(
                f'/dev/video{config.index}를 열 수 없습니다. '
                'v4l2-ctl --list-devices로 번호를 확인하세요.'
            )

        self.lock = threading.Lock()
        self.frame = None
        self.frame_number = 0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._capture_loop,
            name='webcam_hint_capture',
            daemon=True,
        )

    # 드라이버에 MJPG, 버퍼, 해상도와 캡처 FPS를 요청한다.
    def _configure(self):
        settings = (
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
            (cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size),
            (cv2.CAP_PROP_FRAME_WIDTH, self.config.width),
            (cv2.CAP_PROP_FRAME_HEIGHT, self.config.height),
            (cv2.CAP_PROP_FPS, self.config.fps),
        )
        for prop, value in settings:
            self.capture.set(prop, value)

    # 백그라운드 캡처 스레드를 시작한다.
    def start(self):
        self.thread.start()

    # 이전 번호보다 새로운 프레임이 있을 때만 복사본을 반환한다.
    def latest(self, last_number):
        with self.lock:
            if self.frame is None or self.frame_number == last_number:
                return None, last_number
            return self.frame.copy(), self.frame_number

    # 드라이버에 실제 적용된 카메라 설정을 로그용 문자열로 만든다.
    def description(self):
        fourcc = int(self.capture.get(cv2.CAP_PROP_FOURCC))
        fourcc_text = ''.join(
            chr((fourcc >> (8 * index)) & 0xFF) for index in range(4)
        )
        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        buffer_size = int(self.capture.get(cv2.CAP_PROP_BUFFERSIZE))
        return (
            f'/dev/video{self.config.index} {width}x{height}, '
            f'{fps:.1f} FPS, {fourcc_text}, buffer={buffer_size} '
            f'(requested={self.config.buffer_size})'
        )

    # 카메라 큐를 계속 비우며 공유 영역에 최신 프레임만 덮어쓴다.
    def _capture_loop(self):
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                if not self.stop_event.is_set():
                    self.logger.warning(
                        '웹캠2 프레임을 읽지 못했습니다.',
                        throttle_duration_sec=5.0,
                    )
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = frame
                self.frame_number += 1

    # 캡처를 중지하고 카메라와 스레드 자원을 정리한다.
    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            self.logger.warning('webcam_hint 캡처 스레드가 종료되지 않아 카메라 read를 해제합니다.')
            self.capture.release()
            self.thread.join(timeout=1.0)
        else:
            self.capture.release()







class WebcamHint(Node):
    """작업자를 추적하고 SLAM map 좌표를 TargetHint로 발행한다."""

    # ROS I/O, YOLO, 호모그래피, 카메라와 처리 타이머를 구성한다.
    def __init__(self):
        super().__init__('webcam_hint')
        config = self._load_parameters()
        self.image_frame_id = config.image_frame_id
        self.jpeg_quality = config.jpeg_quality
        self.confidence = config.confidence
        self.target_classes = config.target_classes
        self.detection_confirm_count = math.ceil(
            config.detection_confirm_s * config.process_hz
        )
        self.detection_hold_s = config.detection_hold_s
        self.lost_confirm_s = config.lost_confirm_s
        self.hint_valid_s = config.hint_valid_s
        self.show_window = config.show_window
        self._validate(config.camera, config.process_hz, config.image_publish_hz, config.homography_path)

        # 알고리즘: YOLO 검출 좌표를 SLAM map 좌표로 변환한다.
        self.homography = load_homography(config.homography_path)
        self.model = YOLO(config.model_path)
        self.class_names = self.model.names
        self.workers = {}  # worker_id -> outfit_class
        self.track_states = {}  # track_id -> first_seen, last_seen, confirmed

        # ROS I/O: 등록 작업자를 받고 작업자 map 좌표를 보낸다.
        self.create_subscription(WorkerDescriptor, '/webcam1/worker_descriptor', self._on_worker_descriptor, 10)
        self.hint_publisher = self.create_publisher(TargetHint, '/webcam2/target_hint', qos_profile_sensor_data)
        self.image_publisher = self.create_publisher(CompressedImage, config.compressed_topic, qos_profile_sensor_data)

        # 캡처, YOLO 처리, 압축 영상 발행을 서로 다른 실행 경로로 분리한다.
        self.camera = LatestFrameCamera(config.camera, self.get_logger())
        self.last_frame_number = 0
        self.yolo_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / config.process_hz, self._process_latest_frame, callback_group=self.yolo_callback_group)
        self.create_timer(1.0 / config.image_publish_hz, self._publish_compressed_image, callback_group=self.image_callback_group)
        self.camera.start()
        self.get_logger().info(
            f'webcam_hint: {self.camera.description()} | '
            f'process={config.process_hz:.1f} Hz | '
            f'image_publish={config.image_publish_hz:.1f} Hz | '
            f'model={config.model_path}'
        )

    # 파라미터를 선언하면서 실행 시 override된 최종값을 읽는다.
    def _load_parameters(self):
        camera = CameraConfig(
            index=int(self.declare_parameter('camera_index', 0).value),
            width=int(self.declare_parameter('width', 640).value),
            height=int(self.declare_parameter('height', 480).value),
            fps=float(self.declare_parameter('fps', 30.0).value),
            buffer_size=int(self.declare_parameter('buffer_size', 5).value),
        )
        return NodeConfig(
            camera=camera,
            process_hz=float(self.declare_parameter('process_hz', 10.0).value),
            image_publish_hz=float(self.declare_parameter('image_publish_hz', 10.0).value),
            compressed_topic=str(self.declare_parameter('compressed_topic', '/webcam2/image_raw/compressed').value),
            image_frame_id=str(self.declare_parameter('image_frame_id', 'webcam2_optical_frame').value),
            jpeg_quality=int(self.declare_parameter('jpeg_quality', 80).value),
            model_path=str(self.declare_parameter('model_path', DEFAULT_MODEL_PATH).value),
            homography_path=str(self.declare_parameter('homography_json', '').value),
            confidence=float(self.declare_parameter('confidence', 0.5).value),
            target_classes=list(self.declare_parameter('target_classes', DEFAULT_CLASSES).value),
            detection_confirm_s=float(self.declare_parameter('detection_confirm_s', 0.3).value),
            detection_hold_s=float(self.declare_parameter('detection_hold_s', 0.5).value),
            lost_confirm_s=float(self.declare_parameter('lost_confirm_s', 1.0).value),
            hint_valid_s=float(self.declare_parameter('hint_valid_s', 2.0).value),
            show_window=bool(self.declare_parameter('show_window', False).value),
        )

    # 실행 불가능한 FPS, 버퍼, 신뢰도와 경로 값을 초기에 차단한다.
    def _validate(self, camera, process_hz, image_publish_hz, homography_path):
        if camera.fps <= 0.0 or process_hz <= 0.0 or image_publish_hz <= 0.0:
            raise ValueError('fps, process_hz, image_publish_hz는 0보다 커야 합니다.')
        if camera.buffer_size <= 0:
            raise ValueError('buffer_size는 1 이상이어야 합니다.')
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError('confidence는 0~1 범위여야 합니다.')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality는 1~100 범위여야 합니다.')
        if self.detection_confirm_count <= 0 or self.detection_hold_s < 0.0 or self.lost_confirm_s <= 0.0 or self.hint_valid_s <= 0.0:
            raise ValueError('탐지 확정 횟수는 1 이상이고 추적 시간 파라미터는 음수일 수 없습니다.')
        if not homography_path:
            raise ValueError('homography_json 경로가 필요합니다.')

    # ------------------------------------------------------------------ ROS I/O
    # 웹캠1이 등록한 작업자 ID와 복장 클래스 대응을 갱신한다.
    def _on_worker_descriptor(self, message):
        self.workers[message.worker_id] = message.outfit_class

    # 변환된 작업자 map 좌표와 검출 신뢰도를 TargetHint로 발행한다.
    def _publish_hint(self, worker_id, map_x, map_y, confidence):
        stamp = self.get_clock().now().to_msg()
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = 'map'
        point.point.x = float(map_x)
        point.point.y = float(map_y)

        hint = TargetHint()
        hint.header = point.header
        hint.worker_id = worker_id
        hint.point = point
        hint.confidence = float(confidence)
        self.hint_publisher.publish(hint)

    # 캡처 스레드의 최신 프레임을 JPEG로 압축해 독립 주기로 발행한다.
    def _publish_compressed_image(self):
        frame, _ = self.camera.latest(-1)
        if frame is None:
            return

        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warning(
                '웹캠2 JPEG 인코딩에 실패했습니다.',
                throttle_duration_sec=5.0,
            )
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.image_frame_id
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.image_publisher.publish(message)

    # ------------------------------------------------------- YOLO + homography
    # 처리하지 않은 최신 프레임을 YOLO에 넣고 각 검출 결과를 처리한다.
    def _process_latest_frame(self):
        frame, frame_number = self.camera.latest(self.last_frame_number)
        if frame is None:
            return
        self.last_frame_number = frame_number

        result = self.model.track(
            frame,
            persist=True,
            conf=self.confidence,
            verbose=False,
        )[0]
        now = self.get_clock().now().nanoseconds / 1e9
        if result.boxes is not None:
            for box in result.boxes:
                self._process_detection(frame, box, now)
        self._remove_lost_tracks(now)

        if self.show_window:
            cv2.imshow('webcam2 hint', frame)
            cv2.waitKey(1)

    # 대상 클래스를 작업자와 연결하고 bbox를 SLAM map 좌표로 변환한다.
    def _process_detection(self, frame, box, now):
        outfit_class = self._class_name(int(box.cls[0]))
        if self.target_classes and outfit_class not in self.target_classes:
            return

        track_id = int(box.id[0]) if box.id is not None else None
        if track_id is None or not self._confirm_track(track_id, now):
            return

        worker_id = self._worker_id_for_class(outfit_class)
        if worker_id is None:
            return

        x1, y1, x2, y2 = map(float, box.xyxy[0])
        confidence = float(box.conf[0])
        map_x, map_y = bbox_to_map(self.homography, x1, y1, x2, y2)
        self._publish_hint(worker_id, map_x, map_y, confidence)

        if self.show_window:
            self._draw_detection(
                frame, (x1, y1, x2, y2), worker_id, confidence
            )

    # 새 트랙은 최소 검출 횟수를 채우고, 확정된 트랙은 즉시 통과시킨다.
    def _confirm_track(self, track_id, now):
        state = self.track_states.get(track_id)
        if state is None or now - state['last_seen'] > self.detection_hold_s:
            state = {'last_seen': now, 'detection_count': 1, 'confirmed': False}
            self.track_states[track_id] = state
        else:
            state['last_seen'] = now
            if not state['confirmed']:
                state['detection_count'] += 1
        if not state['confirmed'] and state['detection_count'] >= self.detection_confirm_count:
            state['confirmed'] = True
            self.get_logger().info(f'track_id={track_id} 확정 ({state["detection_count"]}회 검출)')
        return state['confirmed']

    # Lost 확정 시간을 넘겼으면 트랙 상태를 제거한다.
    def _remove_lost_tracks(self, now):
        lost_ids = [track_id for track_id, state in self.track_states.items() if now - state['last_seen'] >= self.lost_confirm_s]
        for track_id in lost_ids:
            del self.track_states[track_id]
            self.get_logger().info(f'track_id={track_id} Lost 확정')

    # YOLO 클래스 ID를 모델에 정의된 클래스 이름으로 변환한다.
    def _class_name(self, class_id):
        if isinstance(self.class_names, dict):
            return self.class_names.get(class_id, str(class_id))
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return str(class_id)

    # 복장 클래스가 유일하게 일치하는 등록 작업자 ID를 찾는다.
    def _worker_id_for_class(self, outfit_class):
        candidates = [
            worker_id
            for worker_id, registered_class in self.workers.items()
            if registered_class == outfit_class
        ]
        return candidates[0] if len(candidates) == 1 else None

    # 미리보기 프레임에 bbox와 작업자 ID·신뢰도를 표시한다.
    @staticmethod
    def _draw_detection(frame, bbox, worker_id, confidence):
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f'{worker_id} {confidence:.2f}',
            (x1, max(15, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    # 카메라 스레드와 OpenCV 창을 정리한 뒤 ROS 노드를 종료한다.
    def destroy_node(self):
        if hasattr(self, 'camera'):
            self.camera.close()
        cv2.destroyAllWindows()
        super().destroy_node()


# ROS 노드를 시작하고 Ctrl+C 종료 시 자원을 안전하게 정리한다.
def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = WebcamHint()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
