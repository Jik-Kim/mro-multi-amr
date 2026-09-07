"""ArUco 마커 검출 — 베이 도착 확정(S4)과 대기공간 측위 검증. BRD v3.5 / 계약 v1.10 6절·9.2.

입력: oakd/rgb/image_raw/compressed, oakd/rgb/camera_info
출력: aruco/id (Int32, 없으면 -1) · aruco/distance (Float32, m)

이 노드는 **값을 만들 뿐 판정하지 않는다** (계약 9.1). 같은 두 토픽을 두 곳이 다르게 쓴다.

  ID 1·2·3  베이 A/B/C  → shuttle_mission [주행] 이 `aruco/id == 요청 베이` 로 도착 확정 (FR-08, 오도착 0건)
  ID 4      측위 검증    → fleet_manager [관제] 가 `aruco/distance` 와 |amcl_pose − loc_check_marker.xy| 를
                          대조해 AMCL 발산을 잡는다 (FR-35, 계약 9.2 ①, 차이 ≤ 10 cm 면 정상)

**ID 4 를 거르면 관제의 측위 검증이 통째로 죽는다.** 예약된 10~19(KOZ, Phase 2)는 반대로 통과시키면
안 된다 — 베이로 오인된다. 그래서 허용 목록을 파라미터로 두고 그 밖은 무시한다.

거리는 solvePnP 로 낸다. 종전 핀홀 근사(마커 한 변의 픽셀 길이)는 마커를 비스듬히 보면 변이 짧아
보여 실제보다 멀다고 나왔다. 검출 조건이 정면 ±30°(계약 6절)라 기울어짐이 실제로 생기고, 관제의
판정 기준이 10 cm 라 그 오차를 감당할 수 없다. solvePnP 는 네 꼭짓점으로 평면 자세를 풀어 각도와
무관하게 거리를 낸다. 카메라 왜곡계수(CameraInfo.d)도 함께 쓴다.

정확도는 TS-U05 에서 줄자로 대조한다 — 값을 만드는 쪽이 자를 검증한다 (계약 9.1, FR-34).

실행: ros2 run mro_perception aruco_detector_node --ros-args -r __ns:=/robot3
확인: ros2 topic echo /robot3/aruco/id · /robot3/aruco/distance
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Int32, Float32

DICT = cv2.aruco.DICT_4X4_50   # 계약 6절. ID 1/2/3 = 베이 A/B/C, 4 = 측위 검증, 10~19 = Phase 2 예약


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # 값은 mro_perception/config/perception.yaml 이 단일 출처다 (계약 5절)
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('marker_ids', [1, 2, 3, 4])
        self.declare_parameter('publish_pose_log', True)
        self.size = float(self.get_parameter('marker_size_m').value)
        self.allowed = set(int(v) for v in self.get_parameter('marker_ids').value)
        self.pose_log = bool(self.get_parameter('publish_pose_log').value)

        self.K = None
        self.D = np.zeros((5, 1))
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT),
                                           cv2.aruco.DetectorParameters())
        # solvePnP 용 마커 평면 좌표 — ArUco 꼭짓점 순서(좌상·우상·우하·좌하)와 같은 순서여야 한다
        h = self.size / 2.0
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)

        self.create_subscription(CameraInfo, 'oakd/rgb/camera_info', self.info_cb, 10)
        # 계약 8절: 카메라 토픽은 BEST_EFFORT(sensor_data). 기본 QoS(RELIABLE)로 구독하면
        # 발행자가 BEST_EFFORT 일 때 연결 자체가 성립하지 않아 이미지가 한 장도 안 온다.
        # BEST_EFFORT 구독자는 RELIABLE 발행자로부터도 받으므로 이쪽이 안전하다
        self.create_subscription(CompressedImage, 'oakd/rgb/image_raw/compressed',
                                 self.img_cb, qos_profile_sensor_data)
        self.pub_id = self.create_publisher(Int32, 'aruco/id', 10)
        self.pub_dist = self.create_publisher(Float32, 'aruco/distance', 10)
        self.get_logger().info(f'마커 {self.size * 100:.1f} cm · 허용 ID {sorted(self.allowed)} · solvePnP')

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            d = np.array(msg.d, dtype=np.float64)
            self.D = d.reshape(-1, 1) if d.size else np.zeros((5, 1))
            self.get_logger().info(f'CameraInfo 수신 — fx {self.K[0, 0]:.1f} · 왜곡계수 {self.D.size}개')

    def solve(self, corner):
        """마커 하나의 (거리 m, 정면 대비 기운 각 deg). 실패하면 (nan, nan)."""
        img_pts = corner.reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(self.obj, img_pts, self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return float('nan'), float('nan')
        dist = float(np.linalg.norm(tvec))
        # 마커 법선(z축)과 카메라 시선의 각. 계약 6절 '정면 ±30°' 가 지켜지는지 기록용
        R, _ = cv2.Rodrigues(rvec)
        angle = float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), -1.0, 1.0))))
        return dist, angle

    def img_cb(self, msg):
        if not rclpy.ok():
            return    # 종료 중. 컨텍스트가 내려간 뒤 publish 하면 RCLError 로 크래시처럼 보인다
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        corners, ids, _ = self.det.detectMarkers(frame)

        # 허용 목록에 있는 것만 남긴다. 예약 ID(10~19)나 오검출은 여기서 버린다
        cand = ([(int(i[0]), c) for i, c in zip(ids, corners) if int(i[0]) in self.allowed]
                if ids is not None else [])
        if not cand or self.K is None:
            self.pub_id.publish(Int32(data=-1))
            return

        # 여럿 보이면 가장 큰(= 가장 가까운) 것 하나. 베이 마커와 ID 4 가 동시에 잡히는 구간은
        # 없지만(셔틀 중 vs 대기공간 접근 중), 그래도 결정론적으로 하나만 낸다
        mid, corner = max(cand, key=lambda p: cv2.contourArea(p[1].reshape(-1, 2)))
        dist, angle = self.solve(corner)

        self.pub_id.publish(Int32(data=mid))
        self.pub_dist.publish(Float32(data=dist))
        if self.pose_log and np.isfinite(dist):
            self.get_logger().info(f'ID {mid} · {dist:.3f} m · 기운각 {angle:.0f}°',
                                   throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # launch 가 내릴 때(SIGTERM) 트레이스백이 크래시처럼 보이지 않게
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
