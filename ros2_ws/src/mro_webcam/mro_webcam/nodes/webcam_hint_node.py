"""웹캠2 전역 추적 → 재탐지 방위 힌트. BRD v3.1 E-1 / BR-T27 / 4.1.

입력: USB 웹캠2 (작업공간 + P3 입구 시야 — 중앙 복도는 시야 밖, 9/5 확정), config/WebCam2/homography_cam2.json, /webcam1/worker_descriptor (복장 클래스로 트랙-작업자 대응)
출력: /webcam2/target_hint (mro_interfaces/TargetHint) — 작업자별 map 좌표. **안전 판단에는 절대 쓰지 않는다** (4장 원칙).
       /webcam2/positions.csv 는 kpi_logger 가 기록.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from mro_interfaces.msg import TargetHint, WorkerDescriptor

from mro_webcam.core.homography import bbox_to_map, load_homography

WEBCAM_INDEX = None      # None: 높은 번호부터 자동 탐색 (내장 카메라 회피)
CONF = 0.5


class WebcamHint(Node):
    def __init__(self):
        super().__init__('webcam_hint')
        self.declare_parameter('homography_json', '')
        self.declare_parameter('model_path', '')
        self.H = None
        path = self.get_parameter('homography_json').value
        if path:
            self.H = load_homography(path)
        self.workers = {}     # worker_id -> outfit_class
        self.create_subscription(WorkerDescriptor, '/webcam1/worker_descriptor', self.desc_cb, 10)
        self.pub = self.create_publisher(TargetHint, '/webcam2/target_hint', 10)
        # TODO([웹캠]): 웹캠 열기 + YOLO(worker.pt).track 으로 프레임 처리 타이머 (0.1 s). webcam_register_node 의 카메라 탐색 코드를 재사용.
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('webcam_hint: 골격. 카메라·YOLO 연결은 TODO')

    def desc_cb(self, msg):
        self.workers[msg.worker_id] = msg.outfit_class

    def tick(self):
        # TODO: 검출 결과 (x1,y1,x2,y2, outfit_class) 마다:
        #   mx, my = bbox_to_map(self.H, x1, y1, x2, y2)   # bbox 아래 중앙 = 바닥 접점
        #   worker_id = 복장 클래스가 일치하는 등록 작업자 (2명이 같은 클래스면 트랙 연속성으로 구분)
        #   self.publish(worker_id, mx, my, conf)
        pass

    def publish(self, worker_id, mx, my, conf):
        h = TargetHint()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = 'map'
        h.worker_id = worker_id
        p = PointStamped()
        p.header = h.header
        p.point.x, p.point.y = float(mx), float(my)
        h.point = p
        h.confidence = float(conf)
        self.pub.publish(h)


def main():
    rclpy.init()
    node = WebcamHint()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
