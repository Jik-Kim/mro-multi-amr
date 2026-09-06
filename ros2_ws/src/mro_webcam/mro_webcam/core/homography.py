#!/usr/bin/env python3
"""[이관] Kn1_mini/nav/webcam_homography.py → ros2_ws/src/mro_webcam/mro_webcam/core/homography.py
변경: 기본 json 경로, `main()` 진입점 추가(console_scripts: calibrate_homography).
"""
"""
webcam_homography.py — 삼각대 웹캠 픽셀 → map 좌표 (바닥 평면 호모그래피)

웹캠은 고정, 차는 바닥 위 → 바닥의 4점 이상을 (픽셀, map) 쌍으로 잡으면 3×3 호모그래피 H 로
어떤 픽셀이든 map (x, y) 로 바뀜. ROS 의존 없음 → 웹캠 노드에서 그냥 import.

  from webcam_homography import load_homography, bbox_to_map
  H = load_homography()                       # nav/webcam_homography.json
  mx, my = bbox_to_map(H, x1, y1, x2, y2)     # bbox 아래 중앙(바닥 접점) → map

캘리브레이션 (한 번, 웹캠·아레나 고정 후):
  python3 webcam_homography.py calib --cam 2
    바닥의 특징점(벽 모서리, 테이프 표시 등)을 클릭 → 터미널에 그 점의 map x y 입력 (RViz Publish Point 로 얻음)
    4점 이상 → 's' 저장.  'u' 마지막 점 취소,  'q' 종료.
검증:
  python3 webcam_homography.py test --cam 2   # 아무 픽셀 클릭 → map 좌표 출력. 차를 두고 실측과 비교
  (--image 파일경로 로 웹캠 대신 저장된 사진 사용 가능)

--map ../map/auto_map.yaml 을 주면 입력한 map 좌표를 그 맵의 **가장 가까운 벽 꼭짓점**으로 자동 보정(snap)합니다.
RViz 에서 costmap 색 띠 때문에 벽 모서리를 20~40 cm 안쪽으로 찍게 되는 문제를 없애 줌. 스냅 거리가 크면 경고.
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_JSON = os.path.join(HERE, '..', '..', 'config', 'WebCam1', 'homography_cam1.json')  # 설치본에서는 --out/--json 으로 share/config 경로를 명시할 것


# --------------------------------------------------------------- 사용 함수
def load_homography(path=DEFAULT_JSON):
    """새 `homography`와 기존 `H` JSON 키를 모두 지원한다."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    matrix_data = d.get('homography', d.get('H'))
    if matrix_data is None:
        raise KeyError("JSON에 'homography' 또는 'H' 행렬이 없습니다.")
    matrix = np.array(matrix_data, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('Homography는 유한한 수로 된 3x3 행렬이어야 합니다.')
    return matrix


def pixel_to_map(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def bbox_to_map(H, x1, y1, x2, y2):
    """bbox 아래 중앙 = 차가 바닥에 닿는 점. 호모그래피는 바닥 평면 전제라 이 점을 써야 함."""
    return pixel_to_map(H, (x1 + x2) / 2.0, y2)


def compute_homography(pairs):
    """pairs: [(u, v, mx, my), ...] 4개 이상 → (H, 재투영 오차 m 리스트)"""
    src = np.array([[p[0], p[1]] for p in pairs], dtype=np.float64)
    dst = np.array([[p[2], p[3]] for p in pairs], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, 0)
    errs = [float(np.hypot(*(np.array(pixel_to_map(H, u, v)) - np.array([mx, my])))) for u, v, mx, my in pairs]
    return H, errs


# --------------------------------------------------------------- 맵 꼭짓점 스냅
class MapCorners:
    """맵(pgm+yaml)에서 벽 윤곽 꼭짓점을 뽑아, 입력 좌표를 가장 가까운 꼭짓점으로 보정"""
    def __init__(self, yaml_path):
        import yaml
        meta = yaml.safe_load(open(yaml_path))
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][:2]
        img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image']), cv2.IMREAD_GRAYSCALE)
        self.h = img.shape[0]
        occ = (img < 50).astype(np.uint8)
        cnts, _ = cv2.findContours(occ, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        pts = [p for c in cnts for p in cv2.approxPolyDP(c, 1.5, True).reshape(-1, 2)]
        self.corners = np.array(pts, dtype=float)

    def snap(self, mx, my):
        px, py = (mx - self.ox) / self.res, self.h - 1 - (my - self.oy) / self.res
        j = np.argmin(np.hypot(self.corners[:, 0] - px, self.corners[:, 1] - py))
        cx, cy = self.corners[j]
        sx, sy = self.ox + (cx + 0.5) * self.res, self.oy + (self.h - 1 - cy + 0.5) * self.res
        return round(sx, 2), round(sy, 2), float(np.hypot(sx - mx, sy - my))


# --------------------------------------------------------------- 도구
def open_source(args):
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            sys.exit(f'이미지 못 읽음: {args.image}')
        return None, img
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        sys.exit(f'/dev/video{args.cam} 못 엶. v4l2-ctl --list-devices 로 번호 확인')
    return cap, None


def grab(cap, still):
    if cap is None:
        return still.copy()
    ok, frame = cap.read()
    return frame if ok else None


def calib(args):
    cap, still = open_source(args)
    pairs = []
    click = []
    snapper = MapCorners(args.map) if args.map else None
    if snapper:
        print(f'스냅 모드: 입력 좌표를 {args.map} 의 가장 가까운 벽 꼭짓점으로 보정')
    win = 'calib: click floor point, then type map x y in terminal'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, lambda e, x, y, f, p: click.append((x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    print('바닥 점 클릭 → 터미널에 map x y 입력.  s=저장(4점 이상)  u=취소  q=종료')
    while True:
        frame = grab(cap, still)
        if frame is None:
            continue
        if click:
            u, v = click.pop()
            try:
                mx, my = map(float, input(f'  픽셀 ({u},{v}) 의 map x y: ').split())
                if snapper:
                    sx, sy, dist = snapper.snap(mx, my)
                    print(f'  → 벽 꼭짓점 ({sx},{sy}) 로 스냅 (이동 {dist:.2f} m)'
                          + ('  ※ 0.5 m 넘음 — 엉뚱한 꼭짓점일 수 있음, u 로 취소 가능' if dist > 0.5 else ''))
                    mx, my = sx, sy
                pairs.append((u, v, mx, my))
                print(f'  #{len(pairs)} 저장')
            except ValueError:
                print('  숫자 두 개를 공백으로 (예: -1.25 0.8)')
        for i, (u, v, mx, my) in enumerate(pairs):
            cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
            cv2.putText(frame, f'{i+1}:({mx:.2f},{my:.2f})', (int(u) + 6, int(v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f'points: {len(pairs)}  (s=save u=undo q=quit)', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(win, frame)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break
        if k == ord('u') and pairs:
            pairs.pop()
        if k == ord('s'):
            if len(pairs) < 4:
                print('4점 이상 필요')
                continue
            H, errs = compute_homography(pairs)
            with open(args.out, 'w') as f:
                json.dump({'points': pairs, 'H': H.tolist()}, f, indent=1)
            print(f'저장: {args.out}')
            print('재투영 오차(m):', ' '.join(f'{e:.2f}' for e in errs), f'| 최대 {max(errs):.2f}')
            if max(errs) > 0.15:
                print('  → 15 cm 넘는 점이 있음. 그 점의 클릭/좌표를 다시 확인 (u 로 취소 후 재입력)')
            break
    if cap:
        cap.release()
    cv2.destroyAllWindows()


def test(args):
    H = load_homography(args.out)
    cap, still = open_source(args)
    click = []
    win = 'test: click -> map coord'   # 창 이름은 ASCII 만 (Qt 백엔드에서 비ASCII 면 setMouseCallback 실패)
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, lambda e, x, y, f, p: click.append((x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    last = None
    while True:
        frame = grab(cap, still)
        if frame is None:
            continue
        if click:
            u, v = click.pop()
            last = (u, v, *pixel_to_map(H, u, v))
            print(f'픽셀 ({u},{v}) → map ({last[2]:.2f}, {last[3]:.2f})')
        if last:
            cv2.circle(frame, (last[0], last[1]), 5, (0, 0, 255), -1)
            cv2.putText(frame, f'map ({last[2]:.2f}, {last[3]:.2f})', (last[0] + 6, last[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(win, frame)
        if (cv2.waitKey(30) & 0xFF) == ord('q'):
            break
    if cap:
        cap.release()
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['calib', 'test'])
    ap.add_argument('--cam', type=int, default=0, help='웹캠 번호 (/dev/videoN)')
    ap.add_argument('--image', help='웹캠 대신 사진 파일')
    ap.add_argument('--out', default=DEFAULT_JSON)
    ap.add_argument('--map', help='calib 시 입력 좌표를 이 맵(yaml)의 가장 가까운 벽 꼭짓점으로 스냅')
    a = ap.parse_args()
    calib(a) if a.mode == 'calib' else test(a)


if __name__ == '__main__':
    main()
