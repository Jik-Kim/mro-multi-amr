#!/usr/bin/env python3
"""webcam2 ROS 2 Image 토픽을 화면에 표시하는 테스트용 구독 노드."""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Webcam2Subscriber(Node):
    """webcam2 영상을 구독해 OpenCV 창으로 확인한다."""

    def __init__(self):
        super().__init__('webcam2_subscriber')

        self.declare_parameter('image_topic', '/webcam2/image_raw')
        self.declare_parameter('window_name', 'webcam2 subscriber')
        image_topic = str(self.get_parameter('image_topic').value)
        self.window_name = str(self.get_parameter('window_name').value)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'webcam2 영상 구독 중: {image_topic}')

    def image_callback(self, message):
        """ROS Image를 OpenCV 영상으로 변환해 표시한다."""
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'영상 변환 실패: {error}')
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        """미리보기 창을 닫고 노드를 종료한다."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Webcam2Subscriber()
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
