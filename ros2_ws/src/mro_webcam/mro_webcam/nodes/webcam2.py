#!/usr/bin/env python3
"""USB webcam2 영상 발행 및 클릭 기반 Nav2 주행 테스트 노드."""

import math
import os
import queue
import threading

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from mro_webcam.core.homography import load_homography, pixel_to_map


class Webcam2Node(Node):
    """캡처와 ROS 발행을 분리해 매번 최신 JPEG 프레임만 발행한다."""

    def __init__(self):
        super().__init__('webcam2')

        # 파라미터 선언
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('compressed_topic', '/webcam2/image_raw/compressed')
        self.declare_parameter('frame_id', 'webcam2_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)                             # 이미지 캡처 주기
        self.declare_parameter('publish_fps', 10.0)                     # 이미지 토픽 발행주기
        self.declare_parameter('buffer_size', 5)                        # 이미지 보관 큐 사이즈
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('show_window', False)
        self.declare_parameter('click_coordinate_enabled', True)
        self.declare_parameter('click_navigation_enabled', False)
        self.declare_parameter('manual_pixel_input_enabled', False)
        # 클릭 주행(수동 시험)에 쓸 액션. 특정 로봇을 기본값으로 박아 두면 그 로봇이 빠졌을 때
        #   조용히 죽은 액션으로 목표를 보낸다 — 켜는 쪽이 명시하게 한다 (9/8, robot4 이탈로 드러남)
        self.declare_parameter('navigation_action', '')
        self.declare_parameter('goal_yaw', 0.0)
        default_homography = os.path.join(get_package_share_directory('mro_webcam'), 'config', 'WebCam2', 'homography_cam2.json')
        self.declare_parameter('homography_json', default_homography)




        # 파라미터 반영
        camera_index = int(self.get_parameter('camera_index').value)
        compressed_topic = str(self.get_parameter('compressed_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        capture_fps = float(self.get_parameter('fps').value)
        publish_fps = float(self.get_parameter('publish_fps').value)
        buffer_size = int(self.get_parameter('buffer_size').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.click_coordinate_enabled = bool(self.get_parameter('click_coordinate_enabled').value)
        self.click_navigation_enabled = bool(self.get_parameter('click_navigation_enabled').value)
        self.manual_pixel_input_enabled = bool(self.get_parameter('manual_pixel_input_enabled').value)
        navigation_action = str(self.get_parameter('navigation_action').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        homography_json = str(self.get_parameter('homography_json').value)

        if (self.click_coordinate_enabled or self.click_navigation_enabled) and not self.show_window:
            raise ValueError('클릭 좌표 확인은 show_window=true가 필요합니다.')
        coordinate_input_enabled = (
            self.click_coordinate_enabled
            or self.click_navigation_enabled
            or self.manual_pixel_input_enabled
        )
        self.homography = load_homography(homography_json) if coordinate_input_enabled else None
        if self.click_navigation_enabled and not navigation_action:
            raise RuntimeError(
                'click_navigation_enabled=true 이면 navigation_action 을 줘야 한다 — '
                '예: -p navigation_action:=/robot8/navigate_to_pose')
        self.navigate_client = ActionClient(self, NavigateToPose, navigation_action) if self.click_navigation_enabled else None
        self.last_click = None
        self.window_closed = False
        self.frame_width = width
        self.frame_height = height
        self.manual_pixel_queue = queue.Queue()


        # 파라미터 검증
        if capture_fps <= 0.0 or publish_fps <= 0.0:
            raise ValueError('fps와 publish_fps는 0보다 커야 합니다.')
        if buffer_size <= 0:
            raise ValueError('buffer_size는 1 이상이어야 합니다.')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality는 1~100 범위여야 합니다.')

        self.capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        # USB 전송은 MJPG 30 FPS로 요청하고 V4L2 프레임 큐는 5장으로 제한한다.
        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG'),
        )

        # 캡처 파라미터값 설정 후 open 체크
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, capture_fps)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(
                f'/dev/video{camera_index}를 열 수 없습니다. '
                'v4l2-ctl --list-devices로 카메라 번호를 확인하세요.'
            )




        # 이미지 토픽 발행자
        self.publisher = self.create_publisher(
            CompressedImage,
            compressed_topic,
            qos_profile_sensor_data,
        )
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.capture_stop = threading.Event()
        self.capture_thread = threading.Thread(
            target=self.capture_frames,
            name='webcam2_capture',
            daemon=True,
        )
        self.timer = self.create_timer(1.0 / publish_fps, self.publish_frame)

        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        actual_buffer_size = int(
            self.capture.get(cv2.CAP_PROP_BUFFERSIZE)
        )
        fourcc = int(self.capture.get(cv2.CAP_PROP_FOURCC))
        fourcc_text = ''.join(
            chr((fourcc >> (8 * index)) & 0xFF)
            for index in range(4)
        )
        self.get_logger().info(
            f'webcam2: /dev/video{camera_index} -> {compressed_topic} '
            f'({actual_width}x{actual_height}, capture={actual_fps:.1f} FPS, '
            f'publish={publish_fps:.1f} Hz, format={fourcc_text}, '
            f'buffer={actual_buffer_size}[requested={buffer_size}], '
            f'jpeg_quality={self.jpeg_quality})'
        )
        if self.show_window:
            cv2.namedWindow('webcam2', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('webcam2', self.on_mouse)
        if self.click_navigation_enabled:
            self.get_logger().warning(f'클릭 주행 테스트 활성화: 좌클릭 시 {navigation_action}으로 map 목표를 전송합니다.')
        self.capture_thread.start()
        if self.manual_pixel_input_enabled:
            self.manual_input_thread = threading.Thread(
                target=self.read_manual_pixels,
                name='webcam2_manual_pixel_input',
                daemon=True,
            )
            self.manual_input_thread.start()
            self.get_logger().info(
                '수동 픽셀 입력 활성화: 터미널에 "u v"를 입력하세요. '
                '종료는 "q"입니다.'
            )

    def read_manual_pixels(self):
        """터미널에서 픽셀 좌표를 입력받아 ROS 타이머가 처리하도록 넘긴다."""
        while not self.capture_stop.is_set():
            try:
                value = input('pixel u v> ').strip()
            except EOFError:
                return

            if value.lower() in {'q', 'quit', 'exit'}:
                self.get_logger().info('수동 픽셀 입력을 종료합니다.')
                return

            parts = value.replace(',', ' ').split()
            if len(parts) != 2:
                print('입력 형식: u v  (예: 320 240)', flush=True)
                continue

            try:
                u, v = (int(part) for part in parts)
            except ValueError:
                print('u와 v는 정수로 입력하세요.', flush=True)
                continue

            if not (0 <= u < self.frame_width and 0 <= v < self.frame_height):
                print(
                    f'픽셀 범위: u=0~{self.frame_width - 1}, '
                    f'v=0~{self.frame_height - 1}',
                    flush=True,
                )
                continue

            self.manual_pixel_queue.put((u, v))

    def on_mouse(self, event, u, v, flags, userdata):
        """왼쪽 마우스 클릭을 map 좌표로 변환하고, 활성화 시 Nav2에 보낸다."""
        if event != cv2.EVENT_LBUTTONDOWN or self.homography is None:
            return
        self.process_pixel_coordinate(u, v, '이미지')

    def process_pixel_coordinate(self, u, v, source):
        """클릭 또는 수동 입력 픽셀을 map 좌표로 변환한다."""
        map_x, map_y = pixel_to_map(self.homography, u, v)
        self.last_click = (u, v, map_x, map_y)
        coordinate_text = f'{source} 픽셀=({u}, {v}) -> SLAM map=({map_x:.6f}, {map_y:.6f}) m'
        print(coordinate_text, flush=True)
        self.get_logger().info(coordinate_text)
        if self.click_navigation_enabled:
            self.send_navigation_goal(map_x, map_y)

    def send_navigation_goal(self, map_x, map_y):
        """SLAM map 좌표를 NavigateToPose action goal로 전송한다."""
        if not self.navigate_client.server_is_ready():
            self.get_logger().error('주행 action 서버가 준비되지 않아 목표를 보내지 않습니다.')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(map_x)
        goal.pose.pose.position.y = float(map_y)
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        """Nav2의 goal 수락 여부를 기록한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('클릭 주행 목표가 거절됐습니다.')
            return
        self.get_logger().info('클릭 주행 목표가 수락됐습니다.')
        goal_handle.get_result_async().add_done_callback(self.on_navigation_result)

    def on_navigation_result(self, future):
        """Nav2 action 종료 상태를 기록한다."""
        self.get_logger().info(f'클릭 주행 action 종료: status={future.result().status}')

    def capture_frames(self):
        """카메라 큐를 계속 비우고 가장 최근 프레임만 보관한다."""
        while not self.capture_stop.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                if not self.capture_stop.is_set():
                    self.get_logger().warning(
                        'webcam2 프레임을 읽지 못했습니다.',
                        throttle_duration_sec=5.0,
                    )
                continue
            with self.frame_lock:
                self.latest_frame = frame

    def publish_frame(self):
        """최신 프레임을 JPEG로 인코딩해 발행한다."""
        while True:
            try:
                u, v = self.manual_pixel_queue.get_nowait()
            except queue.Empty:
                break
            self.process_pixel_coordinate(u, v, '수동 입력')

        with self.frame_lock:
            frame = (
                None
                if self.latest_frame is None
                else self.latest_frame.copy()
            )

        if frame is None:
            self.get_logger().warning(
                'webcam2 최신 프레임을 기다리는 중입니다.',
                throttle_duration_sec=5.0,
            )
            return

        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warning(
                'webcam2 JPEG 인코딩에 실패했습니다.',
                throttle_duration_sec=5.0,
            )
            return


        # 메시지 패킹
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.publisher.publish(message)

        if self.show_window:
            display = frame.copy()
            if self.last_click is not None:
                u, v, map_x, map_y = self.last_click
                cv2.circle(display, (u, v), 6, (0, 0, 255), -1)
                cv2.putText(display, f'pixel ({u}, {v})', (u + 8, max(20, v - 28)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                cv2.putText(display, f'SLAM map ({map_x:.3f}, {map_y:.3f})', (u + 8, max(40, v - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.imshow('webcam2', display)
            cv2.waitKey(1)
            if cv2.getWindowProperty('webcam2', cv2.WND_PROP_VISIBLE) < 1:
                self.window_closed = True
                self.get_logger().info('카메라 창이 닫혀 webcam2 노드를 종료합니다.')
                if rclpy.ok():
                    rclpy.shutdown()

    def destroy_node(self):
        """캡처 스레드와 미리보기 자원을 정리한다."""
        self.capture_stop.set()
        if hasattr(self, 'capture'):
            self.capture.release()
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Webcam2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
