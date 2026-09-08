"""웹캠2 압축 영상의 통신 상태와 화면을 확인하는 테스트 노드."""

import os
import time

import cv2

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


DEFAULT_TOPIC = '/webcam2/image_raw/compressed'
WINDOW_NAME = 'Webcam2 compressed image monitor'
EXPECTED_TYPE = 'sensor_msgs/msg/CompressedImage'
DIAGNOSTIC_INTERVAL_S = 5.0
RECEPTION_TIMEOUT_S = 2.0


class Webcam2ImageViewer(Node):
    """압축 영상을 표시하고 DDS 발견·수신 상태를 진단한다."""

    def __init__(self):
        """파라미터, 영상 구독자와 통신 진단 타이머를 생성한다."""
        super().__init__('webcam2_image_viewer')
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.declare_parameter('show_window', True)
        self.declare_parameter('log_interval_s', 1.0)

        self.topic = str(self.get_parameter('topic').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.log_interval_s = float(
            self.get_parameter('log_interval_s').value
        )
        if self.log_interval_s <= 0.0:
            raise ValueError('log_interval_s는 0보다 커야 합니다')

        now = time.monotonic()
        self.started_at = now
        self.last_frame_at = None
        self.last_log_at = now
        self.frame_count = 0
        self.last_log_count = 0
        self.interval_bytes = 0
        self.interval_ages_ms = []
        self.first_frame_logged = False
        self.gui_error_logged = False

        self.subscription = self.create_subscription(
            CompressedImage,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.check_reception)
        self._log_startup()

    def _log_startup(self):
        """토픽과 ROS 통신 환경을 시작 로그로 출력한다."""
        domain_id = os.environ.get('ROS_DOMAIN_ID', '(미설정)')
        discovery_server = os.environ.get(
            'ROS_DISCOVERY_SERVER', '(미설정: 멀티캐스트)'
        )
        super_client = os.environ.get('ROS_SUPER_CLIENT', '(미설정)')
        self.get_logger().info(
            f'구독 시작: {self.topic} [{EXPECTED_TYPE}, BEST_EFFORT]'
        )
        self.get_logger().info(
            f'통신 환경: ROS_DOMAIN_ID={domain_id}, '
            f'ROS_DISCOVERY_SERVER={discovery_server}, '
            f'ROS_SUPER_CLIENT={super_client}'
        )
        if self.show_window:
            self.get_logger().info(
                '영상 창에서 q 또는 ESC를 누르면 종료합니다'
            )
        else:
            self.get_logger().info(
                'show_window=false: 화면 없이 수신 로그만 확인합니다'
            )

    def image_callback(self, message):
        """CompressedImage를 복원해 표시하고 수신 통계를 갱신한다."""
        now = time.monotonic()
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error(
                f'메시지는 받았지만 JPEG 디코딩 실패: '
                f'format={message.format!r}, bytes={len(message.data)}',
                throttle_duration_sec=DIAGNOSTIC_INTERVAL_S,
            )
            return

        self.frame_count += 1
        self.interval_bytes += len(message.data)
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        receive_ns = self.get_clock().now().nanoseconds
        if stamp_ns > 0:
            self.interval_ages_ms.append((receive_ns - stamp_ns) / 1e6)
        self.last_frame_at = now
        height, width = frame.shape[:2]
        if not self.first_frame_logged:
            self.get_logger().info(
                f'첫 영상 수신 성공: {width}x{height}, '
                f'{len(message.data) / 1024:.1f} KiB, '
                f'format={message.format!r}'
            )
            self.first_frame_logged = True

        self._show_frame(frame)
        self._log_rate(now, width, height, len(message.data))

    def _show_frame(self, frame):
        """GUI 사용 시 영상을 표시하고 종료 키를 처리한다."""
        if not self.show_window:
            return
        try:
            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.show_window = False
            if not self.gui_error_logged:
                self.get_logger().error(
                    f'영상은 수신했지만 창을 열 수 없습니다: {error}. '
                    'DISPLAY 설정을 확인하거나 '
                    '--ros-args -p show_window:=false를 사용하세요'
                )
                self.gui_error_logged = True
            return

        if key in (ord('q'), 27):
            self.get_logger().info('사용자 입력으로 모니터를 종료합니다')
            rclpy.shutdown()

    def _log_rate(self, now, width, height, compressed_bytes):
        """설정된 주기마다 수신 FPS와 영상 정보를 출력한다."""
        elapsed = now - self.last_log_at
        if elapsed < self.log_interval_s:
            return
        received = self.frame_count - self.last_log_count
        bandwidth_mbps = self.interval_bytes * 8.0 / elapsed / 1e6
        if self.interval_ages_ms:
            age_mean_ms = float(np.mean(self.interval_ages_ms))
            age_p95_ms = float(np.percentile(self.interval_ages_ms, 95))
            age_max_ms = float(np.max(self.interval_ages_ms))
            age_text = (
                f'pub→sub age mean/p95/max='
                f'{age_mean_ms:.1f}/{age_p95_ms:.1f}/{age_max_ms:.1f} ms'
            )
        else:
            age_text = 'pub→sub age=측정 불가(header stamp=0)'
        self.get_logger().info(
            f'수신 정상: {width}x{height}, '
            f'{compressed_bytes / 1024:.1f} KiB, '
            f'{received / elapsed:.1f} FPS, '
            f'{bandwidth_mbps:.2f} Mbps, {age_text}'
        )
        self.last_log_at = now
        self.last_log_count = self.frame_count
        self.interval_bytes = 0
        self.interval_ages_ms.clear()

    def check_reception(self):
        """발행자 발견 여부와 영상 수신 중단을 구분해 출력한다."""
        publishers = self.get_publishers_info_by_topic(self.topic)
        if self.last_frame_at is not None:
            silent_s = time.monotonic() - self.last_frame_at
            if silent_s >= RECEPTION_TIMEOUT_S:
                self.get_logger().warning(
                    f'발행자는 보이지만 영상 수신이 {silent_s:.1f}초 중단됨: '
                    f'{self.topic}',
                    throttle_duration_sec=DIAGNOSTIC_INTERVAL_S,
                )
            return

        waited_s = time.monotonic() - self.started_at
        if waited_s < RECEPTION_TIMEOUT_S:
            return
        if not publishers:
            self.get_logger().warning(
                f'발행자 미발견: {self.topic}. '
                '발행 PC와 ROS_DOMAIN_ID·ROS_DISCOVERY_SERVER를 맞추세요',
                throttle_duration_sec=DIAGNOSTIC_INTERVAL_S,
            )
            return

        types = sorted({info.topic_type for info in publishers})
        nodes = sorted(
            {
                f'{info.node_namespace.rstrip("/")}/{info.node_name}'
                for info in publishers
            }
        )
        if EXPECTED_TYPE not in types:
            self.get_logger().error(
                f'발행자는 발견했지만 타입 불일치: '
                f'기대={EXPECTED_TYPE}, 실제={types}, 노드={nodes}',
                throttle_duration_sec=DIAGNOSTIC_INTERVAL_S,
            )
            return

        reliabilities = sorted(
            {str(info.qos_profile.reliability) for info in publishers}
        )
        self.get_logger().warning(
            f'발행자 {len(publishers)}개를 발견했지만 메시지가 오지 않음: '
            f'노드={nodes}, reliability={reliabilities}. '
            '발행 측 카메라·발행 주기와 DDS QoS를 확인하세요',
            throttle_duration_sec=DIAGNOSTIC_INTERVAL_S,
        )


def main(args=None):
    """테스트 노드를 실행하고 종료 시 OpenCV 창을 정리한다."""
    rclpy.init(args=args)
    node = Webcam2ImageViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
