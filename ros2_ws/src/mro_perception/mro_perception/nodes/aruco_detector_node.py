"""ArUco 마커 검출 — 베이(A/B/C) 도착 확정. BRD v3.0 S4 / BR-T06.

입력: oakd/rgb/image_raw/compressed
출력: aruco/id (Int32, 검출된 마커 ID, 없으면 -1), aruco/distance (Float32, m, 마커 한 변 길이 기준 추정)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Int32, Float32

MARKER_SIZE_M = 0.10          # 베이 마커 한 변 (BRD 3.2 파라미터). TODO: 파라미터화
DICT = cv2.aruco.DICT_4X4_50  # ID 체계: A=1, B=2, C=3. ID 10~ 는 KOZ 용 예약(Phase 2, 미사용)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.K = None
        self.dist = np.zeros(5)
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters())
        self.create_subscription(CameraInfo, 'oakd/rgb/camera_info', self.info_cb, 10)
        self.create_subscription(CompressedImage, 'oakd/rgb/image_raw/compressed', self.img_cb, 5)
        self.pub_id = self.create_publisher(Int32, 'aruco/id', 10)
        self.pub_dist = self.create_publisher(Float32, 'aruco/distance', 10)

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def img_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        corners, ids, _ = self.det.detectMarkers(frame)
        if ids is None:
            self.pub_id.publish(Int32(data=-1))
            return
        # 가장 큰(가까운) 마커 하나만 보고한다
        areas = [cv2.contourArea(c.reshape(-1, 2)) for c in corners]
        i = int(np.argmax(areas))
        mid = int(ids[i][0])
        dist = float('nan')
        if self.K is not None:
            # 마커 한 변의 픽셀 길이 → 핀홀 근사 거리. 정밀 자세가 필요하면 solvePnP 로 교체
            c = corners[i].reshape(-1, 2)
            side_px = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
            dist = MARKER_SIZE_M * self.K[0, 0] / max(side_px, 1e-6)
        self.pub_id.publish(Int32(data=mid))
        self.pub_dist.publish(Float32(data=dist))


def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
