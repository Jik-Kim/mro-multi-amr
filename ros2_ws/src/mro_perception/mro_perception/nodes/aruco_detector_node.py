"""ArUco 마커 검출 — 베이 도착 확정(S4)과 대기공간 측위 검증. BRD v3.5 / 계약 v1.14 6절·9.2.

입력: oakd/rgb/image_raw/compressed · oakd/rgb/camera_info
출력: aruco/id (Int32, 없으면 -1) · aruco/distance (Float32, m)
      aruco/pose (PoseStamped) — 마커 중심의 위치·자세, 카메라 광학 프레임(x 우·y 하·z 앞).
        주행이 베이 정렬에 쓰는 법: 좌우 치우침 = x, 앞 거리 = z, 마커 방위 = atan2(x, z),
        벽과의 각 = 회전의 yaw. base_link 로 옮기려면 TF(oakd_rgb_camera_optical_frame) 를 쓴다.
        회전은 정사각 마커의 두 해 문제로 흔들릴 수 있어 이전 프레임 자세로 동점을 해소한다 —
        그래도 정렬에는 x·z(방위)를 먼저 쓰고 yaw 는 보조로 쓰기를 권한다.

이 노드는 **값을 만들 뿐 판정하지 않는다** (계약 9.1). 같은 두 토픽을 두 곳이 다르게 쓴다.

  ID 1·2·3  베이 A/B/C  → shuttle_mission [주행] 이 `id == 요청 베이 AND 거리 ≤ 1.5 m` 로 도착 확정 (FR-08)
  ID 4      측위 검증    → fleet_manager [관제] 가 `aruco/distance` 와 |amcl_pose − loc_check_marker.xy| 를
                          대조해 AMCL 발산을 잡는다 (FR-35, 계약 9.2 ①, 차이 ≤ 10 cm 면 정상)

계산은 전부 core/aruco.py 에 있다. 여기는 토픽을 잇고 파라미터를 읽는 것만 한다.

거리는 **마커 네 꼭짓점만으로** 낸다(solvePnP). 원리는 한 줄이다:

    d = fx × marker_size_m ÷ (화면에서의 마커 픽셀 크기)

fx 와 마커 크기는 곱으로만 들어가므로 **fx 가 틀리면 거리가 그 비율 그대로 틀린다.** OAK-D 는
704×704 로 자른 영상을 내면서 자르기 전 16:9 폭 기준의 fx 를 실어 보내(I-031) 실제보다 작다.
9/8 까지는 이 배율을 정사각 crop 기하에서 16/9 = 1.7778 로 어림했고, 그 2~3 % 오차 때문에 pnp
거리가 자리마다 5~10 % 틀려 **스테레오 depth 를 기준자로 삼아야 했다** (짝짓기 큐 · 0.25 s 대기 ·
ROI 품질 게이트 · 역깊이 보정 · depth/pnp 비 누적).

9/9 체커보드로 fx 를 실측해 그 전제가 사라졌다 (camera_fx_scale, core/camera.read_fx_scale):

    robot3  camera_info fx 566.7 × 1.8316 = 1037.9        robot8  566.0 × 1.8355 = 1038.9

두 대의 실측 fx 가 1 px 안에서 같다. 줄자 대조는 전 구간 1.6 cm 안이다 — 0.58 m robot3 −1.4 /
robot8 +0.3 · 1.05 m −0.9 · 1.37 m −0.7(검증점) · 1.70 m +1.3 cm. 관제 FR-35 의 판정 기준이
10 cm 이고, 0.58 m 는 depth 가 +6.6 cm 로 틀리던 자리다. 그래서 depth 경로를 통째로 걷어냈다
(9/9, 304줄 → 157줄). depth 가 필요한 곳은 마커가 없는 인형 추종(amr_detection/target/distance)뿐이다.

거리 게이트(max_distance_m)는 '못 봤다'(id = -1)로 튼다. 자세를 못 푼 프레임도 같다 — 유효한
ID 와 NaN 거리를 함께 내보내면 관제의 비교가 조용히 깨진다. 주행이 자기 쪽에서 1.5 m 로
다시 자르므로(aruco_max_dist_m) 여기 값은 원거리 오검출을 거르는 느슨한 한계다.

카메라 구독은 sensor_data(BEST_EFFORT) QoS 다. 실기 드라이버는 RELIABLE 로 발행하지만
(9/7 robot3 확인) 계약 8절은 BEST_EFFORT 로 적어 놓았고, BEST_EFFORT 구독자는 둘 다 받는다.

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
from geometry_msgs.msg import PoseStamped

from mro_perception.core import aruco
from mro_perception.core.camera import corrected_K, read_fx_scale

CAMERA_FRAME = 'oakd_rgb_camera_optical_frame'   # 영상 헤더에 frame_id 가 비어 올 때의 기본값


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # 값은 mro_perception/config/perception.yaml 이 단일 출처다 (계약 5절). launch 가 넘긴다
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('marker_ids', [1, 2, 3, 4])
        self.declare_parameter('max_distance_m', 2.0)
        self.declare_parameter('publish_pose', True)          # aruco/pose (계약 4절 추가 — PR #40)
        self.declare_parameter('publish_pose_log', True)
        p = lambda k: self.get_parameter(k).value             # noqa: E731
        self.size = float(p('marker_size_m'))
        self.allowed = {int(v) for v in p('marker_ids')}
        self.max_dist = float(p('max_distance_m'))
        self.pose_log = bool(p('publish_pose_log'))
        # 선언·범위 검증은 core/camera.read_fx_scale 이 한다 — amr_detection 과 같은 이름·같은 기본값
        self.fx_scale = read_fx_scale(self)

        self.K = None
        self.D = np.zeros((5, 1))
        # (id, rvec, stamp) — 자세 두 해 사이 동점 해소용. 1 초 넘게 끊기면 버린다
        self.prev = None
        self.det = aruco.make_detector()

        self.create_subscription(CameraInfo, 'oakd/rgb/camera_info', self.info_cb, 10)
        self.create_subscription(CompressedImage, 'oakd/rgb/image_raw/compressed',
                                 self.img_cb, qos_profile_sensor_data)
        self.pub_id = self.create_publisher(Int32, 'aruco/id', 10)
        self.pub_dist = self.create_publisher(Float32, 'aruco/distance', 10)
        # 계약 4절에 없는 토픽이라 파라미터로 켤 때만 만든다. 켜기 전에 PM 에게 계약 추가를 요청한다
        self.pub_pose = self.create_publisher(PoseStamped, 'aruco/pose', 10) if p('publish_pose') else None

        pose_note = ' · aruco/pose 발행' if self.pub_pose else ''
        self.get_logger().info(f'마커 {self.size * 100:.1f} cm · 허용 ID {sorted(self.allowed)} · '
                               f'거리 solvePnP (fx ×{self.fx_scale:.4f}) · 한계 {self.max_dist} m{pose_note}')

    # ---------- 입력 ----------
    def info_cb(self, msg):
        """카메라 내부값은 바뀌지 않는다 — 첫 장만 받아 보정해 둔다."""
        if self.K is None:
            self.K = corrected_K(msg, self.fx_scale)
            d = np.array(msg.d, dtype=np.float64)
            self.D = d.reshape(-1, 1) if d.size else np.zeros((5, 1))
            self.get_logger().info(f'CameraInfo 수신 — fx {msg.k[0]:.1f} → {self.K[0, 0]:.1f} '
                                   f'(×{self.fx_scale:.4f} 보정) · 왜곡계수 {self.D.size}개')

    def none(self, why=None):
        """못 봤다 — id 만 -1 로 낸다. 침묵하면 받는 쪽이 옛 거리를 최신으로 본다 (계약 9.1)."""
        self.pub_id.publish(Int32(data=-1))
        if why:
            self.get_logger().warn(why, throttle_duration_sec=2.0)

    # ---------- 계산 · 출력 ----------
    def img_cb(self, msg):
        """컬러 한 장 → 마커 하나 → 거리·자세 → 발행. 콜백 하나가 곧 발행 하나다."""
        if not rclpy.ok():
            return    # 종료 중. 컨텍스트가 내려간 뒤 publish 하면 RCLError 로 크래시처럼 보인다
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        cand = aruco.detect(self.det, frame, self.allowed)
        if not cand or self.K is None:
            return self.none()

        # 여럿 보이면 가장 큰(= 가장 가까운) 것 하나. 베이 마커와 ID 4 가 동시에 잡히는 구간은
        # 없지만(셔틀 중 vs 대기공간 접근 중), 그래도 결정론적으로 하나만 낸다
        mid, corners = cand[0]
        t = stamp_s(msg.header)
        prev = self.prev[1] if (self.prev and self.prev[0] == mid and t - self.prev[2] < 1.0) else None
        pose = aruco.solve_pose(corners, self.K, self.D, self.size, prev_rvec=prev)
        if pose is None:
            return self.none(f'ID {mid} 자세 실패 — 프레임 버림')
        self.prev = (mid, pose['rvec'], t)

        dist = pose['dist']
        if not np.isfinite(dist) or dist > self.max_dist:
            return self.none(f'ID {mid} 버림 — 거리 {dist:.2f} m (한계 {self.max_dist} m)')
        self.pub_id.publish(Int32(data=mid))
        self.pub_dist.publish(Float32(data=float(dist)))
        if self.pub_pose is not None:
            self.pub_pose.publish(self.make_pose(msg.header, pose))
        if self.pose_log:
            self.get_logger().info(f"ID {mid} · {dist:.3f} m · 기운각 {pose['angle_deg']:.0f}°",
                                   throttle_duration_sec=1.0)

    @staticmethod
    def make_pose(header, pose):
        """solvePnP 의 tvec·rvec 을 그대로 싣는다 — aruco/distance 와 같은 자다."""
        m = PoseStamped()
        m.header.stamp = header.stamp
        m.header.frame_id = header.frame_id or CAMERA_FRAME
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pose['tvec'])
        qx, qy, qz, qw = aruco.rvec_to_quat(pose['rvec'])
        m.pose.orientation.x, m.pose.orientation.y = float(qx), float(qy)
        m.pose.orientation.z, m.pose.orientation.w = float(qz), float(qw)
        return m


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
