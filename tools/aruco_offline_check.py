#!/usr/bin/env python3
"""로봇 없이 ArUco 거리 정확도를 검증한다 — 알려진 자세로 합성 영상을 만들어 실제 검출기에 넣는다.

    python3 tools/aruco_offline_check.py                 # 기본 (보정 계수 16/9)
    python3 tools/aruco_offline_check.py --scale 1.744   # 다른 계수를 썼다면 얼마나 틀리나

로봇이 방전됐거나 아레나에 못 갈 때 쓴다. **실기 검증을 대신하지는 못한다** — 합성 영상에는
모션 블러·조명 반사·JPEG 아티팩트가 없어서 여기 오차는 하한이다. 계수 확정은 TS-U05 줄자
대조로 한다 (계약 9.1, FR-34).

무엇을 잡아내나
  · camera_info fx 보정이 빠지거나 계수가 틀어지면 → '보정 전' 열과 같은 크기로 어긋난다 (I-031)
  · solvePnP 의 물체점 순서(obj)가 ArUco 꼭짓점 순서와 어긋나면 → 거리·기운각이 같이 망가진다
  · 왜곡계수를 안 쓰면(np.zeros) → 화면 가장자리에서 오차가 커진다
  · 거리 게이트가 동작하는지 → max_distance_m 밖이 걸러지는지

왜 호모그래피로 그리지 않나 — 호모그래피는 핀홀 사영이라 왜곡계수가 반영되지 않는다. 그러면
왜곡을 쓰는 코드와 안 쓰는 코드가 똑같은 답을 내서, 정작 검증하려던 것을 못 본다. 그래서
화소마다 시선을 왜곡 풀어 되쏘고 마커 평면과 만나는 점을 찍는다 (render()).

**영상은 언제나 실제 카메라(fx = camera_info × 16/9)로 렌더하고, 푸는 쪽만 --scale 을 바꾼다.**
둘 다 바꾸면 계수가 뭐든 자기일관이라 아무것도 검증하지 못한다 — --scale 1.0 이 합격으로 나온다.

카메라 값은 robot4 실측이다 (9/7, `ros2 topic echo /robot4/oakd/rgb/camera_info`).
robot3 은 fx 566.654 로 0.58 % 다를 뿐 구조는 같다.

주의 — solve() 는 aruco_detector_node.ArucoDetector.solve() 와 **같은 계산을 따로 적은 것**이다
(노드를 import 하려면 rclpy 초기화가 필요하다). 노드의 거리 계산을 바꾸면 여기도 같이 고친다.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mro_perception'))
from mro_perception.core.camera import FX_SCALE_CROP_16_9   # noqa: E402  보정 계수는 한 곳에서만

W = H = 704
K_INFO = np.array([[569.951416, 0, 353.4016418],
                   [0, 569.951416, 354.1185913],
                   [0, 0, 1.0]])                    # camera_info 가 실어 보내는 값 (I-031 — 틀렸다)
D = np.array([-3.5190505981, -28.4767341614, -0.000302788394, 0.0010152061,
              235.2214813232, -3.6854214668, -26.7263298035, 228.3511657715])
MARKER_M = 0.10                                     # perception.yaml marker_size_m
QUIET = 0.5                                         # 검은 사각형 대비 흰 여백 비율 (계약 6절 '한 칸 이상')

ADICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
DET = cv2.aruco.ArucoDetector(ADICT, cv2.aruco.DetectorParameters())


def build_rays(K):
    """화소마다의 시선 방향(정규화 좌표). 왜곡을 푼 것이라 렌더가 왜곡을 그대로 재현한다."""
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    pix = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pix, K, D).reshape(H, W, 2)
    return np.dstack([norm, np.ones((H, W), np.float32)])


def marker_texture(mid, px=400):
    q = int(px * QUIET)
    img = np.full((px + 2 * q, px + 2 * q), 255, np.uint8)
    img[q:q + px, q:q + px] = cv2.aruco.generateImageMarker(ADICT, mid, px)
    return img


def render(rays, mid, z, tilt_deg):
    """거리 z(m)·기울기 tilt(도)로 놓인 마커의 704×704 영상. 배경은 회색 벽."""
    R, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(tilt_deg), 0.0]))
    C = np.array([0.0, 0.0, z])
    n = R[:, 2]
    denom = rays @ n
    t = (C @ n) / np.where(np.abs(denom) < 1e-9, np.nan, denom)
    L = (rays * t[..., None] - C) @ R                # 마커 로컬 좌표 (x 우, y 하 — 영상과 같은 향)
    half = MARKER_M * (1 + 2 * QUIET) / 2
    tex = marker_texture(mid)
    n_px = tex.shape[0]
    tx = (L[..., 0] + half) / (2 * half) * n_px
    ty = (L[..., 1] + half) / (2 * half) * n_px
    ok = (np.isfinite(tx) & np.isfinite(ty) & (t > 0)
          & (tx >= 0) & (tx < n_px) & (ty >= 0) & (ty < n_px))
    out = np.full((H, W), 110, np.uint8)
    out[ok] = tex[np.clip(ty, 0, n_px - 1).astype(np.int32)[ok],
                  np.clip(tx, 0, n_px - 1).astype(np.int32)[ok]]
    out = cv2.GaussianBlur(out, (3, 3), 0.7)         # 렌즈·JPEG 흐림
    out = np.clip(out.astype(np.int16) + np.random.normal(0, 2.0, out.shape), 0, 255).astype(np.uint8)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


def solve(corner, K):
    """aruco_detector_node.solve() 와 같은 계산 — 물체점 순서는 ArUco 꼭짓점 순서와 같아야 한다."""
    h = MARKER_M / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corner.reshape(4, 2).astype(np.float32), K, D,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None
    Rm, _ = cv2.Rodrigues(rvec)
    return float(np.linalg.norm(tvec)), float(np.degrees(np.arccos(np.clip(abs(Rm[2, 2]), -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scale', type=float, default=FX_SCALE_CROP_16_9,
                    help='푸는 쪽에 적용할 camera_fx_scale (기본 16/9). 렌더는 항상 16/9 로 한다')
    ap.add_argument('--true-scale', type=float, default=FX_SCALE_CROP_16_9,
                    help='실제 카메라의 계수. TS-U05 가 16/9 를 뒤집으면 여기를 바꾼다')
    ap.add_argument('--marker-id', type=int, default=4)
    a = ap.parse_args()

    np.random.seed(0)
    K_true = K_INFO.copy()                       # 실제 카메라 — 영상은 항상 이걸로 만든다
    K_true[0, 0] *= a.true_scale
    K_true[1, 1] *= a.true_scale
    K_use = K_INFO.copy()                        # 노드가 쓰는 값 — 이쪽만 --scale 을 받는다
    K_use[0, 0] *= a.scale
    K_use[1, 1] *= a.scale
    rays = build_rays(K_true)

    print(f'실제 카메라 fx {K_true[0,0]:.1f} (camera_info {K_INFO[0,0]:.1f} × {a.true_scale:.4f}) · '
          f'노드가 쓰는 fx {K_use[0,0]:.1f} (× {a.scale:.4f}) · 마커 {MARKER_M*100:.0f} cm · 왜곡 8계수\n')
    print(f"{'참거리':>7} {'기울기':>6} | {'보정 후':>8} {'오차':>8} | {'보정 전':>8} {'오차':>8} | {'기운각':>6}")
    print('-' * 74)
    worst = 0.0
    for tilt in (0, 15, 30, 45):
        for z in (0.5, 0.8, 1.0, 1.35, 1.5, 2.0):
            c, ids, _ = DET.detectMarkers(render(rays, a.marker_id, z, tilt))
            if ids is None or a.marker_id not in [int(i[0]) for i in ids]:
                print(f'{z:6.2f}m {tilt:5.0f}° | {"검출 실패":>38}')
                continue
            corner = c[[int(i[0]) for i in ids].index(a.marker_id)]
            d_ok, ang = solve(corner, K_use)
            d_bad, _ = solve(corner, K_INFO)
            if z <= 1.5 and tilt <= 30:              # 계약 6절 검출 조건 안에서만 합격 판정
                worst = max(worst, abs(d_ok - z))
            print(f'{z:6.2f}m {tilt:5.0f}° | {d_ok:7.3f}m {(d_ok-z)*100:+7.1f}cm | '
                  f'{d_bad:7.3f}m {(d_bad-z)*100:+7.1f}cm | {ang:5.1f}°')
        print()
    # 관제는 aruco/distance 와 amcl 을 10 cm 로 대조한다 (계약 9.2 ①). 그 절반을 내부 기준으로 둔다
    print(f'계약 6절 검출 조건(≤ 1.5 m · ±30°) 안 최대 오차 {worst*100:.1f} cm '
          f'— 관제 판정 기준 10 cm 대비 {"합격" if worst < 0.05 else "재확인 필요"}')
    return 0 if worst < 0.05 else 1


if __name__ == '__main__':
    sys.exit(main())
