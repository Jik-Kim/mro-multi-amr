#!/usr/bin/env python3
"""USB webcam2 영상을 ROS 2 Image 토픽으로 발행한다."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Webcam2Node(Node):
    """USB로 연결된 webcam2의 프레임을 주기적으로 발행한다."""

    def __init__(self):
        super().__init__('webcam2')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_topic', '/webcam2/image_raw')
        self.declare_parameter('frame_id', 'webcam2_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('show_window', False)

        camera_index = int(self.get_parameter('camera_index').value)
        image_topic = str(self.get_parameter('image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        if fps <= 0.0:
            raise ValueError('fps는 0보다 커야 합니다.')

        self.capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(
                f'/dev/video{camera_index}를 열 수 없습니다. '
                'v4l2-ctl --list-devices로 카메라 번호를 확인하세요.'
            )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image,
            image_topic,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)

        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'webcam2: /dev/video{camera_index} -> {image_topic} '
            f'({actual_width}x{actual_height}, {actual_fps:.1f} FPS)'
        )

    def publish_frame(self):
        """카메라 프레임 하나를 읽어 발행한다."""
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning(
                'webcam2 프레임을 읽지 못했습니다.',
                throttle_duration_sec=5.0,
            )
            return

        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

        if self.show_window:
            cv2.imshow('webcam2', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """카메라와 미리보기 자원을 해제한다."""
        if hasattr(self, 'capture'):
            self.capture.release()
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
