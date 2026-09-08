"""ArUco 마커 검출 — 베이 도착 확정(S4)과 대기공간 측위 검증. BRD v3.5 / 계약 v1.13 6절·9.2.

입력: oakd/rgb/image_raw/compressed · oakd/rgb/camera_info · oakd/stereo/image_raw/compressedDepth
출력: aruco/id (Int32, 없으면 -1) · aruco/distance (Float32, m)
      aruco/pose (PoseStamped) — 마커 중심의 위치·자세, 카메라 광학 프레임(x 우·y 하·z 앞).
        위치는 solvePnP 의 tvec 에 depth/pnp 비를 곱한 것이라 aruco/distance 와 같은 자다.
        주행이 베이 정렬에 쓰는 법: 좌우 치우침 = x, 앞 거리 = z, 마커 방위 = atan2(x, z),
        벽과의 각 = 회전의 yaw. base_link 로 옮기려면 TF(oakd_rgb_camera_optical_frame) 를 쓴다.
        회전은 정사각 마커의 두 해 문제로 흔들릴 수 있어 이전 프레임 자세로 동점을 해소한다 —
        그래도 정렬에는 x·z(방위)를 먼저 쓰고 yaw 는 보조로 쓰기를 권한다.

이 노드는 **값을 만들 뿐 판정하지 않는다** (계약 9.1). 같은 두 토픽을 두 곳이 다르게 쓴다.

  ID 1·2·3  베이 A/B/C  → shuttle_mission [주행] 이 `id == 요청 베이 AND 거리 ≤ 1.5 m` 로 도착 확정 (FR-08)
  ID 4      측위 검증    → fleet_manager [관제] 가 `aruco/distance` 와 |amcl_pose − loc_check_marker.xy| 를
                          대조해 AMCL 발산을 잡는다 (FR-35, 계약 9.2 ①, 차이 ≤ 10 cm 면 정상)

계산은 전부 core/aruco.py 에 있다. 여기는 토픽을 잇고 파라미터를 읽는 것만 한다.

거리는 **0.8 m 이상에서 depth 를 우선** 쓴다 (distance_source=auto). 스테레오 depth 는 미터를
직접 내므로 camera_info 의 초점거리 결함(I-031)과 무관하다. depth 원값에는 **로봇별 시차 편향
보정** z = 1/(1/z_raw + depth_k) 를 건다 (core/aruco.calibrated_depth, 값은 perception.yaml
/robotN/**). 스테레오 오차는 시차에 상수로 붙어 거리 오차가 z² 로 커지므로 배율(depth_scale)
하나로는 안 맞는다 — robot3 줄자 4점(0.85·0.98·1.31·1.55 m)에서 배율 0.972 는 1.55 m 를
+3.0 cm 로 냈고(9/8, PM 독립점 기준 2 cm 초과), k=0.024 는 전부 1.5 cm 안이다. core/depth_distance
의 Kn1_mini 보정식(correct_depth, A=1.023·B=0.024)은 같은 꼴이지만 A 가 이 로봇들에 안 맞아
1 m 이상을 4~7 cm 짧게 만들었다(9/7 줄자 5점).

depth_min_m(0.8) 아래는 depth 를 버리고 solvePnP 로 낸다 — 9/7 실측(당시 2호기 robot4) 0.57 m 에서 depth 원값이
+6.6 cm(11 %) 였고 474 프레임이 같은 비율이었다(카메라 근거리 한계, CALIB_RANGE_M 하한과 같다).
robot8 은 ID 4 앞 정차가 0.58 m 라(계약 6절) 이 구간이 관제 FR-35 에 그대로 걸린다. 같은
자리에서 pnp 는 +0.2 cm 였다.

같은 프레임에서 두 거리가 다 나오면 그 비(depth / pnp)를 최근 20 프레임(약 2 초) 중앙값으로
들고 있다가, depth 가 없는 프레임의 pnp 거리(와 aruco/pose 의 위치)에 곱한다. 창을 짧게 두는
이유 — pnp 의 배율은 자리마다 다르므로 마커·로봇이 움직이면 비도 바뀐다. 200 프레임 창으로는
옮긴 뒤 20~40 초 동안 옛 자리의 비가 곱해져 대체값이 16 cm 틀렸다 (9/7 robot3 1.31 m 실측). 9/7 robot3 실측에서
pnp 는 자리에 따라 5~10 % 길게 나왔고(실효 계수 1.63~1.82 로 흔들림, 원인 미상) 그대로 내면
대체 프레임마다 거리가 9 cm 튀어 관제의 10 cm 판정을 건드린다. 비를 곱하면 대체값이 depth 와
같은 자에 맞춰진다. 그 비는 fx_ratio_log_s 마다 로그로도 남긴다 (I-031 ②).

거리 게이트(max_distance_m)는 '못 봤다'(id = -1)로 튼다. 자세를 못 푼 프레임도 같다 — 유효한
ID 와 NaN 거리를 함께 내보내면 관제의 비교가 조용히 깨진다. 주행이 자기 쪽에서 1.5 m 로
다시 자르므로(aruco_max_dist_m) 여기 값은 원거리 오검출을 거르는 느슨한 한계다.

카메라 구독은 sensor_data(BEST_EFFORT) QoS 다. 실기 드라이버는 RELIABLE 로 발행하지만
(9/7 robot3 확인) 계약 8절은 BEST_EFFORT 로 적어 놓았고, BEST_EFFORT 구독자는 둘 다 받는다.

실행: ros2 run mro_perception aruco_detector_node --ros-args -r __ns:=/robot3
확인: ros2 topic echo /robot3/aruco/id · /robot3/aruco/distance
"""
import statistics
import time
from collections import deque

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
from mro_perception.core.depth_distance import decode_compressed_depth

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
        self.declare_parameter('distance_source', 'auto')     # auto | depth | pnp
        self.declare_parameter('depth_k', 0.0)                # 역깊이(시차) 편향 1/m, 로봇별 (yaml /robotN/**)
        self.declare_parameter('depth_scale', 1.0)            # 보정 뒤 배율. 보통 1.0 — core/aruco.calibrated_depth
        self.declare_parameter('depth_min_m', 0.8)            # 이 아래는 depth 대신 pnp
        self.declare_parameter('pair_tol_s', 0.02)            # 같은 프레임 쌍으로 볼 RGB·depth stamp 차
        self.declare_parameter('depth_wait_s', 0.25)          # RGB 뒤 짝 depth 를 이만큼 기다린다 (robot8 +117 ms, 최대 209)
        self.declare_parameter('publish_pose', True)          # aruco/pose (계약 4절 추가 — PR #40)
        self.declare_parameter('publish_pose_log', True)
        self.declare_parameter('fx_ratio_log_s', 10.0)        # 0 이면 끔
        p = lambda k: self.get_parameter(k).value             # noqa: E731
        self.size = float(p('marker_size_m'))
        self.allowed = {int(v) for v in p('marker_ids')}
        self.max_dist = float(p('max_distance_m'))
        self.source = str(p('distance_source'))
        self.depth_k = float(p('depth_k'))
        self.depth_scale = float(p('depth_scale'))
        self.depth_min = float(p('depth_min_m'))
        self.pair_tol = float(p('pair_tol_s'))
        self.depth_wait = float(p('depth_wait_s'))
        self.pose_log = bool(p('publish_pose_log'))
        self.fx_scale = read_fx_scale(self)
        if self.source not in ('auto', 'depth', 'pnp'):
            self.get_logger().error(f"distance_source='{self.source}' 는 auto|depth|pnp 중 하나 — auto 로 간다")
            self.source = 'auto'

        self.K = None
        self.D = np.zeros((5, 1))
        self.depths = deque(maxlen=8)   # 최근 depth (depth_mm, stamp_s) 들. RGB 와 stamp 로 짝짓는다
        self.pending = deque()          # 짝 depth 를 기다리는 RGB 프레임들 — 도착 순서대로 낸다
        self.depth_alive_t = None       # 마지막 depth 도착 시각 (스트림 생존 판정)
        self.paired_t = None            # 마지막으로 짝이 성립한 시각 (짝이 계속 안 맞으면 경고)
        self.prev = None             # (id, rvec, stamp) — 자세 두 해 사이 동점 해소용. 1 초 넘게 끊기면 버린다
        self.ratios = deque(maxlen=20)    # depth/pnp 비, 최근 2 초. pnp 대체값·pose 에 곱한다
        self.det = aruco.make_detector()

        self.create_subscription(CameraInfo, 'oakd/rgb/camera_info', self.info_cb, 10)
        self.create_subscription(CompressedImage, 'oakd/rgb/image_raw/compressed',
                                 self.img_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, 'oakd/stereo/image_raw/compressedDepth',
                                 self.depth_cb, qos_profile_sensor_data)
        self.pub_id = self.create_publisher(Int32, 'aruco/id', 10)
        self.pub_dist = self.create_publisher(Float32, 'aruco/distance', 10)
        # 계약 4절에 없는 토픽이라 파라미터로 켤 때만 만든다. 켜기 전에 PM 에게 계약 추가를 요청한다
        self.pub_pose = self.create_publisher(PoseStamped, 'aruco/pose', 10) if p('publish_pose') else None
        if float(p('fx_ratio_log_s')) > 0:
            self.create_timer(float(p('fx_ratio_log_s')), self.report_ratio)
        # RGB 와 depth 는 같은 트리거로 나와 stamp 가 같지만(median 0.1 ms) depth 가 늦게 온다 — robot3 +83 ms,
        # robot8 +117 ms(최대 209, 9/8 실측). RGB 주기(100 ms)보다 길어서 '최신 한 장'만 들고 있으면 짝이 거의
        # 안 맞는다(robot8 0 %). 그래서 depth 를 여러 장 보관하고 RGB 프레임은 큐에서 depth_wait_s 까지
        # 기다린 뒤 도착 순서대로 낸다. 이 타이머가 큐를 20 ms 마다 비운다
        self.create_timer(0.02, self.flush_ready)

        pose_note = ' · aruco/pose 발행' if self.pub_pose else ''
        self.get_logger().info(f'마커 {self.size * 100:.1f} cm · 허용 ID {sorted(self.allowed)} · '
                               f'거리 {self.source} (depth 보정 k={self.depth_k:.4f}/m ×{self.depth_scale:.3f}, {self.depth_min} m 아래는 pnp) · '
                               f'한계 {self.max_dist} m{pose_note}')

    # ---------- 입력 ----------
    def info_cb(self, msg):
        if self.K is None:
            self.K = corrected_K(msg, self.fx_scale)
            d = np.array(msg.d, dtype=np.float64)
            self.D = d.reshape(-1, 1) if d.size else np.zeros((5, 1))
            self.get_logger().info(f'CameraInfo 수신 — fx {msg.k[0]:.1f} → {self.K[0, 0]:.1f} '
                                   f'(×{self.fx_scale:.4f} 보정) · 왜곡계수 {self.D.size}개')

    def depth_cb(self, msg):
        depth = decode_compressed_depth(msg)
        if depth is None:
            return
        self.depths.append((depth, stamp_s(msg.header)))
        self.depth_alive_t = time.monotonic()
        self.flush_ready()

    # ---------- 계산 ----------
    def depth_for(self, t_rgb):
        """같은 프레임(stamp 차 ≤ pair_tol_s)의 depth. 없으면 None."""
        best = min(self.depths, key=lambda d: abs(d[1] - t_rgb), default=None)
        return best if best is not None and abs(best[1] - t_rgb) <= self.pair_tol else None

    def depth_alive(self):
        return self.depth_alive_t is not None and time.monotonic() - self.depth_alive_t < 1.0

    def depth_distance(self, corners, pose, rgb_shape, depth):
        """짝 depth 로 (직선거리 | None, roi dict). ROI 가 불량이면 None.

        직선거리가 None 이면 depth 는 있었지만 depth_min_m 아래라 못 믿는 것이다.
        """
        depth_mm = depth[0]
        r = aruco.marker_depth(depth_mm, corners, scale=depth_mm.shape[1] / rgb_shape[1])
        if r is None:
            return None
        if r['z_raw'] < self.depth_min:
            return None, r
        r['z_cal'] = aruco.calibrated_depth(r['z_raw'], self.depth_k, self.depth_scale)   # 로봇별 시차 편향 보정
        return aruco.radial_from_depth(r['z_cal'], pose['tvec']), r

    def pnp_scale(self):
        """pnp 거리를 depth 의 자에 맞추는 비. 표본이 아직 없으면 1.0 (그대로 낸다)."""
        return statistics.median(self.ratios) if len(self.ratios) >= 5 else 1.0

    def none(self, why=None):
        self.pub_id.publish(Int32(data=-1))
        if why:
            self.get_logger().warn(why, throttle_duration_sec=2.0)

    def img_cb(self, msg):
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
        self.pending.append({'mid': mid, 'corners': corners, 'pose': pose, 'shape': frame.shape,
                             't': t, 'header': msg.header, 'arrived': time.monotonic()})
        self.flush_ready()

    def flush_ready(self):
        """큐 앞에서부터 — 짝 depth 가 왔으면 depth 경로로, 대기 시한이 지났으면 pnp 경로로 낸다.

        앞 프레임이 아직 기다리는 중이면 뒤 프레임도 기다린다 — 발행 순서가 뒤집히면 소비자가 오래된 값을
        최신으로 보게 된다. 지연은 최대 depth_wait_s 다.
        """
        now = time.monotonic()
        while self.pending:
            it = self.pending[0]
            depth = None if self.source == 'pnp' else self.depth_for(it['t'])
            if depth is None and self.source != 'pnp' and now - it['arrived'] < self.depth_wait:
                return                                   # 아직 올 수 있다 — 기다린다
            self.pending.popleft()
            self.publish(it, depth)

    def publish(self, it, depth):
        """프레임 하나를 발행한다. depth 는 짝 depth 또는 None(시한 초과·pnp 강제)."""
        src, dist = 'pnp', it['pose']['dist']
        if self.source != 'pnp':
            got = self.depth_distance(it['corners'], it['pose'], it['shape'], depth) if depth is not None else None
            if got is not None and got[0] is not None:
                src, dist = 'depth', got[0]
                self.paired_t = time.monotonic()
                if it['pose']['angle_deg'] <= 30:    # 계약 6절 범위. 그 밖은 ROI p20 이 가까운 쪽을 읽어 비가 낮아진다
                    self.ratios.append(got[1]['z_cal'] / it['pose']['z'])
            elif got is not None:                    # depth 는 있지만 근거리 — pnp 로. 비가 있으면 곱한다
                k = self.pnp_scale()
                dist *= k
                src = f'pnp×{k:.3f}(근거리)' if k != 1.0 else 'pnp(근거리)'
            elif self.source == 'depth':
                return self.none(f"ID {it['mid']} depth 없음 — 프레임 버림 (distance_source=depth)")
            elif len(self.ratios) < 5 and self.depth_alive():
                # depth 스트림은 살아 있는데 비 표본이 아직 없다(시작 직후). 원 pnp 는 자리에 따라 10 % 틀리므로
                # 한 장이라도 내보내면 관제 판정을 건드린다 — 비가 쌓일 때까지 '못 봤다'로 낸다.
                # 2 초 넘게 한 번도 짝이 안 맞으면 pair_tol_s·depth_wait_s 문제다 — 조용히 굶지 않게 경고한다
                why = None
                if self.paired_t is None and time.monotonic() - self.depth_alive_t > 2.0:
                    why = (f"depth 는 오는데 RGB 와 짝이 안 맞는다 — stamp 차 {self.pair_tol * 1000:.0f} ms, "
                           f"대기 {self.depth_wait * 1000:.0f} ms 안에 안 옴. pair_tol_s/depth_wait_s 확인")
                return self.none(why)
            else:
                k = self.pnp_scale()
                dist *= k
                src = f'pnp×{k:.3f}' if k != 1.0 else 'pnp'

        if not np.isfinite(dist) or dist > self.max_dist:
            return self.none(f"ID {it['mid']} 버림 — 거리 {dist:.2f} m (한계 {self.max_dist} m)")
        self.pub_id.publish(Int32(data=it['mid']))
        self.pub_dist.publish(Float32(data=float(dist)))
        if self.pub_pose is not None:
            self.pub_pose.publish(self.make_pose(it['header'], it['pose'], self.pnp_scale()))
        if self.pose_log:
            self.get_logger().info(f"ID {it['mid']} · {dist:.3f} m ({src}) · 기운각 {it['pose']['angle_deg']:.0f}°",
                                   throttle_duration_sec=1.0)

    # ---------- 출력 ----------
    @staticmethod
    def make_pose(header, pose, scale=1.0):
        """위치는 pnp 의 tvec 에 depth/pnp 비를 곱한다 — 좌우·앞뒤가 같은 배율로 틀리므로 같이 맞춘다."""
        m = PoseStamped()
        m.header.stamp = header.stamp
        m.header.frame_id = header.frame_id or CAMERA_FRAME
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) * scale for v in pose['tvec'])
        qx, qy, qz, qw = aruco.rvec_to_quat(pose['rvec'])
        m.pose.orientation.x, m.pose.orientation.y = float(qx), float(qy)
        m.pose.orientation.z, m.pose.orientation.w = float(qz), float(qw)
        return m

    def report_ratio(self):
        """depth/pnp 비의 중앙값. camera_fx_scale 에 곱하면 이 자리에서의 실효 계수다 (I-031 ②)."""
        if len(self.ratios) < 5:
            return
        med = statistics.median(self.ratios)
        self.get_logger().info(f'depth/pnp 비 {med:.3f} (최근 {len(self.ratios)}점) · '
                               f'실효 fx 계수 {self.fx_scale * med:.3f} (설정 {self.fx_scale:.4f})')


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
