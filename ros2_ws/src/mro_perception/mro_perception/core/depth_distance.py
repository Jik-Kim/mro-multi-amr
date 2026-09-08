#!/usr/bin/env python3
"""
depth_distance.py — 보정 적용 거리 측정 (공용 모듈 + 뷰어)

두 가지 용도로 씁니다.

1) 모듈로 import  (YOLO 노드 등에서)
    from depth_distance import distance_from_bbox, decode_compressed_depth
    r = distance_from_bbox(depth_mm, x1, y1, x2, y2, scale=2.0)
    if r: print(r['z_corr'])

2) 뷰어로 실행
    python3 depth_distance.py --ros-args -r __ns:=/robot3
    (네임스페이스 생략 시 ROBOT_NS 사용)

    RGB | depth 를 나란히 띄우고, 어느 쪽을 클릭해도 그 픽셀 주변 ROI 거리를
    "측정(원값) -> 보정" 으로 출력합니다. c / f 키로 캘리브레이션도 됩니다.

측정 방식과 보정식의 근거는 docs/DEPTH_TROUBLESHOOTING.md, docs/DEPTH_CORRECTION_EXPLAINED.md 참고.
"""

import csv
import os

import cv2
import numpy as np

try:                                   # ROS 는 아래 뷰어 노드(DepthViewer)만 쓴다
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image
except ImportError:                    # ROS 없이 roi_distance·decode_compressed_depth 만 쓰는 경우
    rclpy = CvBridge = CompressedImage = Image = None   # (core/aruco 단위 테스트, tools/aruco_offline_check)
    Node = object

# =====================================================================
# 설정
# =====================================================================
ROBOT_NS = '/robot3'                 # __ns 를 안 넘겼을 때 쓰는 기본 네임스페이스
DEPTH_TRANSPORT = 'compressedDepth'  # 'compressedDepth' (WiFi 권장) 또는 'raw'

# 보정식  1/Z_true = CORR_A / Z_meas + CORR_B
#   모형차 0.8~2.0 m, 5점 회귀 (RMS 8.2 mm), 검증 3점 최대 1.06 %
CORR_A = 1.02313
CORR_B = 0.02408
CALIB_RANGE_M = (0.8, 2.0)   # 이 밖은 외삽 — 값은 주되 표시로 알려줌
VALID_RANGE_M = (0.2, 5.0)   # 이 밖은 센서 신뢰 불가 → None

QUANTILE = 20                # ROI 하위 분위수 (%) — 경계에서 섞인 배경(더 먼 값) 제거
MIN_VALID_PIXELS = 10        # 유효 픽셀이 이보다 적으면 측정 포기
CLICK_HALF = 15              # 클릭 측정 ROI 반폭 → 31×31
COLOR_MAX_M = 3.0            # 시각화 스케일 (고정. 프레임마다 안 바뀜)

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'car_depth_calib.csv')   # 실행 위치와 무관하게 이 폴더의 CSV

# compressed_depth_image_transport 의 ConfigHeader:
#   int32 format + float depthParam[2] = 12 byte 가 PNG 앞에 붙어 있음
COMPRESSED_DEPTH_HEADER = 12


# =====================================================================
# 공용 함수 — 여기까지는 import 해도 아무 부작용 없음
# =====================================================================
def correct_depth(z_m):
    """보정식 적용. 입력/출력 m."""
    return 1.0 / (CORR_A / z_m + CORR_B)


def decode_compressed_depth(msg):
    """
    sensor_msgs/CompressedImage (compressedDepth) → uint16 numpy (mm).
    실패하면 None. cv_bridge 로는 12 byte 헤더 때문에 못 푼다.
    """
    if 'rvl' in msg.format.lower():
        return None                                  # RVL 코덱은 cv2 로 못 풂
    buf = np.frombuffer(msg.data, dtype=np.uint8)[COMPRESSED_DEPTH_HEADER:]
    depth = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)  # UNCHANGED 여야 16bit 유지
    if depth is None or depth.dtype != np.uint16:
        return None
    return depth


def roi_distance(depth_mm, x0, y0, x1, y1):
    """
    depth_mm[y0:y1, x0:x1] 의 통계로 거리 계산.
    반환: dict(z_raw, z_corr, n, lo, q, med, hi, spread, extrapolated)  단위 m / mm
          유효 픽셀 부족 또는 범위 밖이면 None
    """
    h, w = depth_mm.shape
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return None

    roi = depth_mm[y0:y1, x0:x1].astype(np.float32)
    vals = roi[roi > 0]                              # 0 = 매칭 실패 픽셀 제외
    if vals.size < MIN_VALID_PIXELS:
        return None

    lo, q, med, hi = np.percentile(vals, [0, QUANTILE, 50, 100])
    z_raw = float(q) / 1000.0
    if not (VALID_RANGE_M[0] < z_raw < VALID_RANGE_M[1]):
        return None

    return {
        'z_raw': z_raw,
        'z_corr': correct_depth(z_raw),
        'n': int(vals.size),
        'lo': float(lo), 'q': float(q), 'med': float(med), 'hi': float(hi),
        'spread': float(hi - lo),
        'extrapolated': not (CALIB_RANGE_M[0] <= z_raw <= CALIB_RANGE_M[1]),
        'roi': (x0, y0, x1, y1),
    }


def distance_at(depth_mm, u, v, half=CLICK_HALF):
    """픽셀 (u, v) 주변 (2·half+1)² ROI 로 측정."""
    return roi_distance(depth_mm, u - half, v - half, u + half + 1, v + half + 1)


def distance_from_bbox(depth_mm, x1, y1, x2, y2, inner_ratio=0.5, scale=1.0):
    """
    검출기 bbox 로 측정.
      inner_ratio : bbox 중심에서 이 비율만큼의 안쪽 영역만 ROI 로 사용 (가장자리 배경 회피)
      scale       : bbox 좌표계 → depth 좌표계 배율.
                    예) YOLO 가 352×352 preview 에서 돌고 depth 가 704×704 면 2.0
    """
    x1, y1, x2, y2 = (v * scale for v in (x1, y1, x2, y2))
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    hw, hh = (x2 - x1) * inner_ratio / 2.0, (y2 - y1) * inner_ratio / 2.0
    return roi_distance(depth_mm,
                        int(cx - hw), int(cy - hh), int(cx + hw) + 1, int(cy + hh) + 1)


def colorize_depth(depth_mm, max_m=COLOR_MAX_M):
    """고정 스케일 JET 컬러맵. 무효 픽셀(0)은 회색."""
    norm = np.clip(depth_mm.astype(np.float32), 0, max_m * 1000)
    norm = (norm / (max_m * 1000) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    colored[depth_mm == 0] = (128, 128, 128)
    return colored


def format_result(r):
    """로그 한 줄용 문자열."""
    if r is None:
        return '측정 불가'
    tag = ' (외삽)' if r['extrapolated'] else ''
    return (f"n={r['n']} | min={r['lo']:.0f} p{QUANTILE}={r['q']:.0f} "
            f"med={r['med']:.0f} max={r['hi']:.0f} mm | spread={r['spread']:.0f} mm"
            f" | 측정 {r['z_raw'] * 1000:.0f} mm -> 보정 {r['z_corr'] * 1000:.0f} mm{tag}")


# =====================================================================
# 뷰어 노드
# =====================================================================
WINDOW = 'RGB (left) | Depth (right)'   # 비ASCII 문자가 들어가면 Qt 백엔드에서 setMouseCallback 이 창을 못 찾음


class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_distance_viewer')
        self.bridge = CvBridge()
        self.depth_mm = None
        self.rgb = None
        self.last = None           # 마지막 측정 결과 dict
        self.samples = []          # (z_raw, z_true)
        self.should_exit = False

        ns = self.get_namespace()
        if ns == '/':
            ns = ROBOT_NS
        rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        if DEPTH_TRANSPORT == 'compressedDepth':
            depth_topic = f'{ns}/oakd/stereo/image_raw/compressedDepth'
            self.create_subscription(CompressedImage, depth_topic, self.depth_cb_compressed, 1)
        else:
            depth_topic = f'{ns}/oakd/stereo/image_raw'
            self.create_subscription(Image, depth_topic, self.depth_cb_raw, 1)
        self.create_subscription(CompressedImage, rgb_topic, self.rgb_cb, 1)

        self.load_samples()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1408, 704)
        cv2.setMouseCallback(WINDOW, self.mouse_cb)
        self.get_logger().info(f'depth: {depth_topic}\n           rgb:   {rgb_topic}')
        self.get_logger().info('클릭=측정  c=실측값 입력  f=보정계수 회귀  q=종료')

    # ---------- 구독 콜백 ----------
    def depth_cb_compressed(self, msg):
        depth = decode_compressed_depth(msg)
        if depth is None:
            self.get_logger().warn(f'compressedDepth 디코드 실패 (format="{msg.format}")',
                                   throttle_duration_sec=5.0)
            return
        self.depth_mm = depth
        self.render()

    def depth_cb_raw(self, msg):
        self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.render()

    def rgb_cb(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.rgb = img

    # ---------- 클릭 ----------
    def mouse_cb(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.depth_mm is None:
            return
        w = self.depth_mm.shape[1]
        if x >= w:                      # 오른쪽(depth) 절반을 클릭하면 좌표를 왼쪽 기준으로
            x -= w
        self.last = distance_at(self.depth_mm, x, y)
        self.get_logger().info(f'({x},{y}) {format_result(self.last)}')

    # ---------- 화면 ----------
    def render(self):
        depth_vis = colorize_depth(self.depth_mm)
        h, w = self.depth_mm.shape

        if self.rgb is not None and self.rgb.shape[:2] == (h, w):
            rgb_vis = self.rgb.copy()
        else:                           # RGB 아직 없거나 크기 다르면 검은 판
            rgb_vis = np.zeros((h, w, 3), np.uint8)

        if self.last is not None:
            x0, y0, x1, y1 = self.last['roi']
            label = f"{self.last['z_corr']:.3f} m" + (' *' if self.last['extrapolated'] else '')
            for img, color in ((rgb_vis, (0, 255, 0)), (depth_vis, (255, 255, 255))):
                cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), color, 1)
                cv2.putText(img, label, (x0, max(15, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow(WINDOW, np.hstack((rgb_vis, depth_vis)))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True
        elif key == ord('c'):
            self.add_sample()
        elif key == ord('f'):
            self.fit()

    # ---------- 캘리브레이션 ----------
    def load_samples(self):
        try:
            with open(CALIB_FILE, newline='') as f:
                for row in csv.reader(f):
                    try:
                        self.samples.append((float(row[0]), float(row[1])))
                    except (ValueError, IndexError):
                        continue
        except FileNotFoundError:
            return
        if self.samples:
            self.get_logger().info(f'{os.path.basename(CALIB_FILE)} 샘플 {len(self.samples)}개 로드')

    def add_sample(self):
        if self.last is None:
            self.get_logger().warn('먼저 클릭해서 측정하세요')
            return
        try:
            z_true = float(input('실측 거리 (m): ').strip())
        except ValueError:
            self.get_logger().warn('숫자를 입력하세요')
            return
        z_raw = self.last['z_raw']
        self.samples.append((z_raw, z_true))
        with open(CALIB_FILE, 'a', newline='') as f:
            csv.writer(f).writerow([f'{z_raw:.4f}', f'{z_true:.4f}'])
        self.get_logger().info(f'샘플 {len(self.samples)}개 저장 (측정 {z_raw:.3f} / 실측 {z_true:.3f})')

    def fit(self):
        """역깊이 공간 최소제곱:  1/z_true = a·(1/z_meas) + b"""
        if len(self.samples) < 3:
            self.get_logger().warn(f'샘플 부족 ({len(self.samples)}/3)')
            return
        zm = np.array([s[0] for s in self.samples])
        zt = np.array([s[1] for s in self.samples])
        M = np.vstack([1.0 / zm, np.ones_like(zm)]).T
        (a, b), *_ = np.linalg.lstsq(M, 1.0 / zt, rcond=None)
        res_mm = (1.0 / (a / zm + b) - zt) * 1000.0
        self.get_logger().info(f'CORR_A = {a:.5f}, CORR_B = {b:.5f}  (n={len(self.samples)})')
        self.get_logger().info(f'잔차 mm: {np.array2string(res_mm, precision=1)} '
                               f'| RMS {np.sqrt((res_mm ** 2).mean()):.1f} mm')


def main():
    rclpy.init()
    node = DepthViewer()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
