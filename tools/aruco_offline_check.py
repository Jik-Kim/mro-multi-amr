#!/usr/bin/env python3
"""로봇 없이 ArUco 거리 정확도를 검증한다 — 알려진 자세로 합성 영상을 만들어 실제 계산에 넣는다.

    python3 tools/aruco_offline_check.py                 # 기본 (체커보드 실측 계수 1.8336)
    python3 tools/aruco_offline_check.py --scale 1.7778  # 9/8 까지 쓰던 어림값이면 거리가 얼마나 틀리나
    python3 tools/aruco_offline_check.py --robot robot3   # 어느 로봇 광학으로 볼지 (기본 robot8)

로봇이 방전됐거나 아레나에 못 갈 때 쓴다. **실기 검증을 대신하지는 못한다** — 합성 영상에는
모션 블러·조명 반사·JPEG 아티팩트가 없어서 여기 오차는 하한이다. 계수 확정은 체커보드 실측과
TS-U05 줄자 대조로 한다 (계약 9.1, FR-34). 9/9 확정값은 core/camera.FX_SCALE_MEASURED 다.

계산 함수는 노드와 **같은 것을 import 한다** (mro_perception.core.aruco). 검증 도구가 자기
계산을 따로 갖고 있으면 한쪽만 고쳐져 검증이 검증이 아니게 된다.

무엇을 잡아내나
  · camera_info fx 보정이 빠지거나 계수가 틀어지면 → 'pnp' 열이 '보정 전' 열 쪽으로 어긋난다 (I-031)
  · solvePnP 물체점 순서가 ArUco 꼭짓점 순서와 어긋나면 → 거리·기운각이 같이 망가진다
  · 왜곡계수를 안 쓰면 → 화면 가장자리에서 오차가 커진다

왜 호모그래피로 그리지 않나 — 호모그래피는 핀홀 사영이라 왜곡계수가 반영되지 않는다. 그러면
왜곡을 쓰는 코드와 안 쓰는 코드가 똑같은 답을 내서, 정작 검증하려던 것을 못 본다. 그래서
화소마다 시선을 왜곡 풀어 되쏘고 마커 평면과 만나는 점을 찍는다 (render()).

**영상은 언제나 실제 카메라(fx = camera_info × --true-scale)로 렌더하고, 푸는 쪽만 --scale 을 바꾼다.**
둘 다 바꾸면 계수가 뭐든 자기일관이라 아무것도 검증하지 못한다 — --scale 1.0 이 합격으로 나온다.

카메라 값은 **9/10 에 두 로봇에서 직접 받은 것이다** (`CAMERAS`). `--robot` 으로 고른다.
fx 는 0.11 % 밖에 안 다르지만 주점이 다르다 — robot8 은 cy 가 중앙에서 15.6 px 벗어나 있다.
남의 로봇 값으로 돌리면 남의 광학으로 합격을 매기게 되므로 검증하려는 로봇을 지정한다.

**합성 마커에는 흰 여백을 그린다 (QUIET).** 계약 6절이 요구하는 여백이고, 9/10 robot8 현장에서
여백 없는 마커가 최대 −16 % 짧게 나온 것이 바로 이 조건을 안 지켜서였다 (I-031).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/mro_perception'))
from mro_perception.core import aruco                                   # noqa: E402
from mro_perception.core.camera import FX_SCALE_MEASURED                # noqa: E402

W = H = 704
# camera_info 실측값 (`ros2 topic echo /robotN/oakd/rgb/camera_info --once`, 9/10).
# **로봇마다 다르다** — 특히 robot8 은 cy 가 중앙(352)에서 15.6 px 벗어나 있다. 이 도구는 이 값을
# "실제 카메라"로 삼아 영상을 렌더하므로, 남의 로봇 값으로 돌리면 남의 광학으로 합격을 매긴다.
# 9/9 까지는 여기에 9/7 robot4 값이 박혀 있었다 (기체 교체로 그 카메라는 이제 없다).
CAMERAS = {
    'robot3': {
        'fx': 566.6536254882812, 'cx': 352.513427734375, 'cy': 355.11474609375,
        'D': [17.71727752685547, -118.81301879882812, -0.0005236465367488563,
              -0.00047328477376140654, 428.64984130859375, 17.506752014160156,
              -117.90373992919922, 423.49481201171875]},
    'robot8': {
        'fx': 566.0105590820312, 'cx': 352.3825988769531, 'cy': 367.6126708984375,
        'D': [13.380138397216797, -150.46047973632812, 0.0022977159824222326,
              0.001133363926783204, 541.2130126953125, 13.132322311401367,
              -148.4298858642578, 533.8888549804688]},
}


def camera(name):
    """이름 → (camera_info K, 왜곡계수 D). 둘은 한 짝이라 따로 섞어 쓰면 안 된다."""
    c = CAMERAS[name]
    K = np.array([[c['fx'], 0, c['cx']], [0, c['fx'], c['cy']], [0, 0, 1.0]])
    return K, np.array(c['D'])


MARKER_M = 0.10                                     # perception.yaml marker_size_m
QUIET = 0.5                                         # 검은 사각형 대비 흰 여백 비율 (계약 6절 '한 칸 이상')
DET = aruco.make_detector()


def build_rays(K, D):
    """화소마다의 시선 방향(정규화 좌표). 왜곡을 푼 것이라 렌더가 왜곡을 그대로 재현한다."""
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    pix = np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pix, K, D).reshape(H, W, 2)
    return np.dstack([norm, np.ones((H, W), np.float32)])


def marker_texture(mid, px=400):
    """검은 사각형 + 흰 여백. 여백 포함 한 변의 실제 길이는 MARKER_M × (1 + 2·QUIET)."""
    q = int(px * QUIET)
    img = np.full((px + 2 * q, px + 2 * q), 255, np.uint8)
    img[q:q + px, q:q + px] = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(aruco.DICT), mid, px)
    return img


def render(rays, mid, z, tilt_deg):
    """거리 z(m)·기울기 tilt(도)로 놓인 마커의 704×704 영상. 마커는 회색 벽 위에 있다."""
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
    on_wall = np.isfinite(t) & (t > 0)
    ok = on_wall & (tx >= 0) & (tx < n_px) & (ty >= 0) & (ty < n_px)
    out = np.full((H, W), 110, np.uint8)
    out[ok] = tex[np.clip(ty, 0, n_px - 1).astype(np.int32)[ok],
                  np.clip(tx, 0, n_px - 1).astype(np.int32)[ok]]
    out = cv2.GaussianBlur(out, (3, 3), 0.7)         # 렌즈·JPEG 흐림
    out = np.clip(out.astype(np.int16) + np.random.normal(0, 2.0, out.shape), 0, 255).astype(np.uint8)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


def main():
    """거리·기울기 격자에서 pnp 거리와 '보정 전' 거리를 참값과 대조한다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--scale', type=float, default=FX_SCALE_MEASURED,
                    help='푸는 쪽에 적용할 camera_fx_scale (기본 = 체커보드 실측). 렌더는 --true-scale 로 한다')
    ap.add_argument('--true-scale', type=float, default=FX_SCALE_MEASURED,
                    help='실제 카메라의 계수. 체커보드를 다시 재면 여기를 바꾼다')
    ap.add_argument('--marker-id', type=int, default=4)
    ap.add_argument('--robot', default='robot8', choices=sorted(CAMERAS),
                    help='어느 로봇의 camera_info 로 영상을 만들지 (광학이 로봇마다 다르다)')
    a = ap.parse_args()
    K_INFO, D = camera(a.robot)

    np.random.seed(0)
    K_true = K_INFO.copy()                       # 실제 카메라 — 영상은 항상 이걸로 만든다
    K_true[0, 0] *= a.true_scale
    K_true[1, 1] *= a.true_scale
    K_use = K_INFO.copy()                        # 노드가 쓰는 값 — 이쪽만 --scale 을 받는다
    K_use[0, 0] *= a.scale
    K_use[1, 1] *= a.scale
    rays = build_rays(K_true, D)

    print(f'실제 카메라 fx {K_true[0, 0]:.1f} (camera_info {K_INFO[0, 0]:.1f} × {a.true_scale:.4f}) · '
          f'노드가 쓰는 fx {K_use[0, 0]:.1f} (× {a.scale:.4f}) · 마커 {MARKER_M * 100:.0f} cm · 왜곡 8계수\n')
    print(f"{'참거리':>7} {'기울기':>6} | {'pnp':>8} {'오차':>8} | {'보정 전':>8} {'오차':>8} | {'기운각':>6}")
    print('-' * 74)
    worst = {'pnp': 0.0}
    for tilt in (0, 15, 30, 45):
        for z in (0.5, 0.8, 1.0, 1.35, 1.5, 2.0):
            img = render(rays, a.marker_id, z, tilt)
            cand = aruco.detect(DET, img, {a.marker_id})
            if not cand:
                print(f'{z:6.2f}m {tilt:5.0f}° | {"검출 실패":>60}')
                continue
            corners = cand[0][1]
            pose = aruco.solve_pose(corners, K_use, D, MARKER_M)
            bad = aruco.solve_pose(corners, K_INFO, D, MARKER_M)
            d_pnp = pose['dist'] if pose else float('nan')
            d_bad = bad['dist'] if bad else float('nan')
            if z <= 1.5 and tilt <= 30:              # 계약 6절 검출 조건 안에서만 합격 판정
                worst['pnp'] = max(worst['pnp'], abs(d_pnp - z))
            print(f'{z:6.2f}m {tilt:5.0f}° | '
                  f'{d_pnp:7.3f}m {(d_pnp - z) * 100:+7.1f}cm | {d_bad:7.3f}m {(d_bad - z) * 100:+7.1f}cm | '
                  f"{pose['angle_deg'] if pose else float('nan'):5.1f}°")
        print()
    # 관제는 aruco/distance 와 amcl 을 10 cm 로 대조한다 (계약 9.2 ①). 그 절반을 내부 기준으로 둔다
    ok = all(v < 0.05 for v in worst.values())
    print(f"계약 6절 검출 조건(≤ 1.5 m · ±30°) 안 pnp 최대 오차 {worst['pnp'] * 100:.1f} cm "
          f"→ 관제 판정 기준 10 cm 대비 {'합격' if ok else '재확인 필요'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
