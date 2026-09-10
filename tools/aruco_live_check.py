#!/usr/bin/env python3
"""줄자 검증용 실시간 뷰어 — 마커를 화면에 그리고 거리를 띄운다. 9/9 체커보드 계수 확인용.

    python3 tools/aruco_live_check.py --ns /robot3 --tape 1.05

무엇을 보나
  · 마커가 잡히나 (네 모서리 · ID)
  · 이 코드가 내는 거리 — 노드와 **같은 함수**(mro_perception.core.aruco)를 부른다
  · 노드가 실제로 발행 중인 aruco/id · aruco/distance (노드가 떠 있으면 같이 뜬다)
  · **마커의 map 좌표** — TF(map ← 카메라 광학 프레임)로 옮겨 x · y · 방위를 띄운다.
    AMCL 이 돌고 있어야 나온다. ID 4 면 arena.yaml 의 loc_check_marker.xy 와 차이도 같이 띄운다
    (관제 FR-35 가 하는 그 대조다). arena 값이 아직 [0, 0] 이면 '미실측'으로 표시한다 —
    이 뷰어로 읽은 map 좌표가 그 자리에 들어갈 값이다.
  · 줄자값을 주면 오차를 cm 로, 2 cm 안이면 초록으로

키
    SPACE   지금 자세로 N 프레임(기본 50) 모아 중앙값·퍼짐을 낸다 → 화면·터미널·CSV
    r       모은 기록 표시 지우기
    q / ESC 끝내기

줄자 재는 법 (9/9 와 같게)
  · **카메라 렌즈 맨 앞면**에서 마커 중앙까지. 이 코드가 내는 것은 광학중심 기준이라
    렌즈 두께만큼(약 1 cm 이내) 계통 차이가 있을 수 있다 — 9/9 실측도 같은 방식이었다.
  · 마커는 정면(±30° 안), 화면 가운데 근처에 둔다.

노드도 같이 보려면 다른 터미널에서 로봇 런치를 띄워 두면 된다:
    ros2 launch mro_bringup robot.launch.py namespace:=/robot3
노드가 없어도 이 뷰어는 혼자 돈다 (카메라 토픽만 있으면 된다).

모니터가 없는 로봇 PC(SSH)면 --no-window 로 터미널 출력만 쓴다.
"""
import argparse
import csv
import math
from collections import deque
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Int32, Float32
import tf2_ros
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mro_perception'))
from mro_perception.core import aruco                                    # noqa: E402
from mro_perception.core.camera import FX_SCALE_MEASURED, corrected_K    # noqa: E402

CSV_PATH = Path(__file__).resolve().parent / 'aruco_live_check_log.csv'
ARENA_DEFAULT = Path(__file__).resolve().parents[1] / 'ros2_ws/src/mro_bringup/params/arena.yaml'
CAMERA_FRAME = 'oakd_rgb_camera_optical_frame'


def _walk(d):
    """중첩 dict 를 전부 훑는다 — arena.yaml 의 계층이 바뀌어도 loc_check_marker 를 찾게."""
    if isinstance(d, dict):
        yield d
        for v in d.values():
            yield from _walk(v)


def quat_to_R(x, y, z, w):
    """쿼터니언 → 3×3 회전행렬. tf2_geometry_msgs 의 API 가 배포판마다 달라 직접 만든다."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
GREEN, RED, YELLOW, WHITE, GRAY = (90, 220, 90), (60, 60, 235), (60, 200, 240), (255, 255, 255), (170, 170, 170)


class LiveCheck(Node):
    def __init__(self, a):
        super().__init__('aruco_live_check')
        self.a = a
        self.K = self.D = None
        self.fx_info = None
        self.info_wh = None        # camera_info 가 말하는 해상도
        self.img_wh = None         # 실제로 오는 영상 해상도 — 둘이 다르면 fx 가 안 맞는다
        self.det = aruco.make_detector()
        self.frame = None          # (그림, 검출결과, 자세)
        self.node_id = None        # 노드가 발행 중인 값
        self.node_dist = None
        self.node_seen_t = 0.0
        self.samples = []          # SPACE 로 모으는 중인 (거리, map x, map y)
        self.side_px = None            # 화면에서 마커 한 변 (픽셀) — 크기 가정이 맞는지 보려고
        self.hits = deque(maxlen=30)   # 최근 30프레임 검출 성공 여부 — 끊기는지 보려고
        self.ids_all = []              # 이번 프레임에 보인 ID 전부 (허용 목록 거르기 전)
        self.rejected = 0              # 네모까지는 갔는데 무늬를 못 읽은 후보 수
        self.map_xy = None         # 마커의 map 좌표 (x, y, yaw_deg) · TF 가 없으면 None
        self.map_why = 'waiting for TF'   # map 좌표가 안 나오는 이유 (화면용이라 영문)
        self.cam_frame = CAMERA_FRAME
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.arena_xy, self.arena_id = None, None
        if a.arena and Path(a.arena).exists():
            try:
                d = yaml.safe_load(open(a.arena, encoding='utf-8'))
                m = next(v['loc_check_marker'] for v in _walk(d) if isinstance(v, dict) and 'loc_check_marker' in v)
                self.arena_id, self.arena_xy = int(m['aruco_id']), [float(v) for v in m['xy']]
            except Exception as e:                                    # noqa: BLE001
                self.get_logger().warn(f'arena.yaml 을 못 읽었다 — {e}')
        self.result = None         # 마지막 표본 결과 (문자열 여러 줄)
        self.fps_t, self.fps = time.monotonic(), 0.0

        ns = a.ns.rstrip('/')
        self.create_subscription(CameraInfo, f'{ns}/oakd/rgb/camera_info', self.info_cb, 10)
        self.create_subscription(CompressedImage, f'{ns}/oakd/rgb/image_raw/compressed',
                                 self.img_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, f'{ns}/aruco/id', self.nid_cb, 10)
        self.create_subscription(Float32, f'{ns}/aruco/distance', self.ndist_cb, 10)
        self.get_logger().info(f'{ns} 구독 · 계수 ×{a.scale:.4f} · 마커 {a.size * 100:.0f} cm · '
                               f'표본 {a.samples} 프레임' + (f' · 줄자 {a.tape:.3f} m' if a.tape else ''))

    # ---------- 입력 ----------
    def info_cb(self, msg):
        if self.K is None:
            self.fx_info = float(msg.k[0])
            self.info_wh = (msg.width, msg.height)
            self.K = corrected_K(msg, self.a.scale)
            d = np.array(msg.d, dtype=np.float64)
            self.D = d.reshape(-1, 1) if d.size else np.zeros((5, 1))
            self.get_logger().info(f'CameraInfo — fx {self.fx_info:.1f} → {self.K[0, 0]:.1f}')

    def nid_cb(self, msg):
        self.node_id, self.node_seen_t = msg.data, time.monotonic()

    def ndist_cb(self, msg):
        self.node_dist = float(msg.data)

    def to_map(self, pose):
        """카메라 광학 프레임의 마커 자세 → map 좌표 (x, y, yaw_deg). 못 하면 None."""
        try:
            tf = self.tf_buf.lookup_transform(self.a.map_frame, self.cam_frame, rclpy.time.Time())
        except Exception as e:                                        # noqa: BLE001
            self.map_why = f'{type(e).__name__} - run localization?'
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        p = R @ np.asarray(pose['tvec'], dtype=np.float64) + np.array([t.x, t.y, t.z])
        Rm = R @ cv2.Rodrigues(np.asarray(pose['rvec'], dtype=np.float64).reshape(3, 1))[0]
        self.map_why = ''
        return float(p[0]), float(p[1]), float(np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0])))

    def img_cb(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        self.img_wh = (img.shape[1], img.shape[0])
        now = time.monotonic()
        self.fps = 0.8 * self.fps + 0.2 / max(now - self.fps_t, 1e-3)
        self.fps_t = now
        if msg.header.frame_id:
            self.cam_frame = msg.header.frame_id
        # 진단 — 허용 목록을 거르기 전에 무엇이 보였나. 끊길 때 원인을 가른다
        raw_c, raw_ids, rej = self.det.detectMarkers(img)
        self.ids_all = sorted(int(i[0]) for i in raw_ids) if raw_ids is not None else []
        self.rejected = len(rej) if rej is not None else 0
        cand = aruco.detect(self.det, img, set(self.a.ids)) if self.K is not None else []
        self.hits.append(bool(cand))
        pose = None
        if cand:
            c = cand[0][1]
            self.side_px = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
            pose = aruco.solve_pose(c, self.K, self.D, self.a.size)
            self.map_xy = self.to_map(pose) if pose is not None else None
            if pose is not None and self.collecting:
                self.samples.append((pose['dist'],) + (self.map_xy[:2] if self.map_xy else (float('nan'),) * 2))
                if len(self.samples) >= self.a.samples:
                    self.finish_sample()
        else:
            self.map_xy = None
            self.side_px = None
        self.frame = (img, cand[0] if cand else None, pose)

    # ---------- 표본 ----------
    collecting = False

    def start_sample(self):
        self.samples, self.collecting, self.result = [], True, None

    def finish_sample(self):
        """모은 표본을 정리한다. 터미널·CSV 는 한글, 화면은 영문 — cv2 가 한글을 못 그린다."""
        self.collecting = False
        v = sorted(x[0] for x in self.samples)
        med = statistics.median(v)
        lo, hi = v[int(len(v) * 0.1)], v[int(len(v) * 0.9)]
        ko = [f'{len(v)} 프레임 · 중앙값 {med:.4f} m',
              f'퍼짐 p10~p90  {lo:.4f} ~ {hi:.4f} m  ({(hi - lo) * 100:.1f} cm)']
        en = [f'{len(v)} frames   median {med:.4f} m',
              f'p10~p90  {lo:.4f} ~ {hi:.4f} m   ({(hi - lo) * 100:.1f} cm)']
        mx = [x[1] for x in self.samples if math.isfinite(x[1])]
        my = [x[2] for x in self.samples if math.isfinite(x[2])]
        mxm = mym = float('nan')
        if mx:
            mxm, mym = statistics.median(mx), statistics.median(my)
            ko.append(f'마커 map 좌표 중앙값  x {mxm:+.3f}  y {mym:+.3f} m  ({len(mx)} 프레임)')
            en.append(f'map median   x {mxm:+.3f}   y {mym:+.3f} m')
            seen = self.frame[1][0] if self.frame[1] else None
            if self.arena_xy and self.arena_id == seen:
                if self.arena_xy == [0.0, 0.0]:
                    ko.append('arena.yaml loc_check_marker.xy 는 아직 [0, 0] — 위 값이 그 자리에 넣을 값이다')
                    en.append('-> put this into arena.yaml loc_check_marker.xy')
                else:
                    dd = math.hypot(mxm - self.arena_xy[0], mym - self.arena_xy[1]) * 100
                    ko.append(f'arena 기준점과 차이 {dd:.1f} cm  (관제 허용 10 cm · FR-35)')
                    en.append(f'vs arena  {dd:.1f} cm   (limit 10 cm, FR-35)')
        else:
            ko.append(f'map 좌표 없음 — {self.map_why}')
            en.append('map: not available')
        ok = None
        if self.a.tape:
            err = (med - self.a.tape) * 100
            ok = abs(err) <= 2.0
            ko.append(f'줄자 {self.a.tape:.3f} m 대비 {err:+.1f} cm  → {"합격" if ok else "재확인"} (기준 ±2 cm)')
            en.append(f'tape {self.a.tape:.3f} m -> {err:+.1f} cm   {"PASS" if ok else "CHECK"} (+-2 cm)')
        self.result = (en, ok)
        print('\n'.join('  ' + t for t in ko) + '\n')
        new_file = not CSV_PATH.exists()
        with open(CSV_PATH, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(['시각', 'ns', '계수', 'camera_info fx', '보정 fx', '마커 m',
                            '표본', '중앙값 m', 'p10 m', 'p90 m', '줄자 m', '오차 cm', 'map x', 'map y'])
            w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), self.a.ns, f'{self.a.scale:.4f}',
                        f'{self.fx_info:.2f}' if self.fx_info else '', f'{self.K[0, 0]:.2f}',
                        self.a.size, len(v), f'{med:.4f}', f'{lo:.4f}', f'{hi:.4f}',
                        self.a.tape or '', f'{(med - self.a.tape) * 100:+.2f}' if self.a.tape else '',
                        f'{mxm:.4f}' if mx else '', f'{mym:.4f}' if mx else ''])
        print(f'  → {CSV_PATH} 에 추가했다\n')


def put(img, text, xy, color=WHITE, scale=0.6, thick=2):
    cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def draw(n):
    """오버레이를 그린 그림. 한글은 cv2 가 못 그려서 라벨은 영문·숫자로 둔다."""
    img, cand, pose = n.frame
    view = img.copy()
    a = n.a

    if n.K is None:
        put(view, 'waiting for camera_info ...', (20, 40), YELLOW)
    if cand is not None:
        mid, c = cand
        pts = c.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(view, [pts], True, GREEN, 2, cv2.LINE_AA)
        for i, (x, y) in enumerate(c.astype(int)):
            cv2.circle(view, (x, y), 5, (0, 165, 255) if i == 0 else GREEN, -1, cv2.LINE_AA)
        cx, cy = c.mean(axis=0).astype(int)
        put(view, f'ID {mid}', (cx - 28, cy - 14), GREEN, 0.8, 2)
        if pose is not None:
            d = pose['dist']
            col = GREEN
            if a.tape:
                col = GREEN if abs(d - a.tape) <= 0.02 else (YELLOW if abs(d - a.tape) <= 0.05 else RED)
            put(view, f'{d:.3f} m', (cx - 60, cy + 34), col, 1.1, 3)

    y = 34
    ids = ','.join(str(i) for i in a.ids)
    put(view, f'ns {a.ns}   scale x{a.scale:.4f}   marker {a.size * 100:.0f} cm   allowed [{ids}]',
        (16, y), WHITE, 0.6); y += 26
    if n.K is not None:
        iw = f'{n.img_wh[0]}x{n.img_wh[1]}' if n.img_wh else '?'
        cw = f'{n.info_wh[0]}x{n.info_wh[1]}' if n.info_wh else '?'
        bad = n.img_wh and n.info_wh and n.img_wh != n.info_wh
        put(view, f'fx {n.fx_info:.1f} -> {n.K[0, 0]:.1f}    img {iw}  info {cw}'
                  f'{"  MISMATCH!" if bad else ""}    {n.fps:4.1f} fps',
            (16, y), RED if bad else GRAY, 0.55); y += 26
    if pose is not None:
        t = pose['tvec']
        put(view, f"x {t[0]:+.3f}  y {t[1]:+.3f}  z {t[2]:+.3f} m", (16, y), WHITE, 0.55); y += 24
        put(view, f"tilt {pose['angle_deg']:4.1f} deg   reproj {pose['reproj_px']:.2f} px", (16, y), WHITE, 0.55); y += 26
    else:
        put(view, 'no marker', (16, y), YELLOW, 0.6); y += 26

    if n.map_xy is not None:
        mx, my, myaw = n.map_xy
        put(view, f'map  x {mx:+.3f}  y {my:+.3f} m   yaw {myaw:+6.1f} deg', (16, y), (120, 255, 255), 0.6); y += 26
        if cand is not None and n.arena_id == cand[0] and n.arena_xy:
            if n.arena_xy == [0.0, 0.0]:
                put(view, 'arena loc_check_marker.xy = [0, 0] (not measured yet)', (16, y), YELLOW, 0.5); y += 22
            else:
                dd = math.hypot(mx - n.arena_xy[0], my - n.arena_xy[1]) * 100
                put(view, f'vs arena {n.arena_xy[0]:+.3f},{n.arena_xy[1]:+.3f}  diff {dd:.1f} cm',
                    (16, y), GREEN if dd <= 10 else RED, 0.55); y += 24
    elif pose is not None:
        put(view, f'map: - ({n.map_why})', (16, y), GRAY, 0.5); y += 24

    if n.side_px and n.K is not None:
        line = f'side {n.side_px:.0f} px'
        if a.tape:
            # 줄자 거리가 맞다면 마커 실제 한 변 = tape × 픽셀 / fx. 가정과 다르면 여기서 드러난다
            implied = a.tape * n.side_px / n.K[0, 0] * 100
            col = GREEN if abs(implied - a.size * 100) <= 0.5 else RED
            line += f'   implied marker {implied:.1f} cm  (assumed {a.size * 100:.0f})'
            put(view, line, (16, y), col, 0.55); y += 26
        else:
            put(view, line, (16, y), GRAY, 0.55); y += 26

    if n.hits:
        rate = 100.0 * sum(n.hits) / len(n.hits)
        col = GREEN if rate > 95 else (YELLOW if rate > 70 else RED)
        seen = ','.join(str(i) for i in n.ids_all) or '-'
        put(view, f'detect {rate:3.0f}% ({sum(n.hits)}/{len(n.hits)})   ids seen [{seen}]'
                  f'   rejected {n.rejected}', (16, y), col, 0.55); y += 26
        blocked = [i for i in n.ids_all if i not in a.ids]
        if blocked:
            put(view, f'ID {blocked} seen but FILTERED OUT by --ids {ids}', (16, y), RED, 0.6); y += 26

    fresh = time.monotonic() - n.node_seen_t < 1.0
    if fresh:
        nd = f'{n.node_dist:.3f} m' if (n.node_id not in (None, -1) and n.node_dist is not None) else '-'
        line = f'node: id {n.node_id}  {nd}'
        if pose is not None and n.node_id not in (None, -1) and n.node_dist is not None:
            line += f'   diff {(n.node_dist - pose["dist"]) * 100:+.1f} cm'
        put(view, line, (16, y), (255, 200, 120), 0.55)
    else:
        put(view, 'node: (not publishing)', (16, y), GRAY, 0.55)
    y += 30

    if a.tape:
        put(view, f'tape {a.tape:.3f} m', (16, y), YELLOW, 0.6); y += 26
    if n.collecting:
        put(view, f'SAMPLING {len(n.samples)}/{a.samples}', (16, y), (60, 200, 255), 0.7); y += 28
    if n.result:
        lines, ok = n.result
        for s in lines:
            put(view, s, (16, y), GREEN if ok is not False else RED, 0.55); y += 24

    put(view, 'SPACE sample   r reset   q quit', (16, view.shape[0] - 16), GRAY, 0.5, 1)
    return view


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/robot3', help='로봇 네임스페이스 (예: /robot3 · /robot8)')
    ap.add_argument('--tape', type=float, help='줄자로 잰 참 거리 [m]. 주면 오차를 바로 띄운다')
    ap.add_argument('--scale', type=float, default=FX_SCALE_MEASURED, help='camera_fx_scale (기본 = 체커보드 실측)')
    ap.add_argument('--size', type=float, default=0.10, help='마커 한 변 [m]')
    ap.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4])
    ap.add_argument('--samples', type=int, default=50, help='SPACE 한 번에 모을 프레임 수')
    ap.add_argument('--map-frame', default='map', help='map 좌표를 낼 기준 프레임')
    ap.add_argument('--arena', default=str(ARENA_DEFAULT),
                    help='arena.yaml 경로 — ID 4 의 loc_check_marker.xy 와 대조한다. 빈 문자열이면 끔')
    ap.add_argument('--no-window', action='store_true', help='창 없이 터미널로만 (SSH)')
    a = ap.parse_args()

    rclpy.init()
    n = LiveCheck(a)
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.frame is None:
                continue
            if a.no_window:
                now = time.monotonic()
                if now - last > 0.5:
                    last = now
                    _, cand, pose = n.frame
                    if pose is not None:
                        err = f'  오차 {(pose["dist"] - a.tape) * 100:+.1f} cm' if a.tape else ''
                        mp = (f'  map {n.map_xy[0]:+.3f},{n.map_xy[1]:+.3f} m' if n.map_xy
                              else f'  map -({n.map_why})')
                        print(f'ID {cand[0]} · {pose["dist"]:.3f} m · 기운각 {pose["angle_deg"]:.0f}°'
                              f' · 재투영 {pose["reproj_px"]:.2f} px{err}{mp}')
                    else:
                        print('마커 없음')
                    if not n.collecting and not n.samples:
                        n.start_sample()          # 창이 없으면 자동으로 계속 모은다
                continue
            cv2.imshow('aruco live check', draw(n))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord(' ') and not n.collecting:
                n.start_sample()
            if k == ord('r'):
                n.result, n.samples, n.collecting = None, [], False
    except (KeyboardInterrupt, ExternalShutdownException):
        pass    # Ctrl-C · 종료 신호를 크래시처럼 보이지 않게
    finally:
        if not a.no_window:
            cv2.destroyAllWindows()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
