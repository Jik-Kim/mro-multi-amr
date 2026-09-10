#!/usr/bin/env python3
"""체커보드로 카메라 내부 파라미터(fx·fy·cx·cy·왜곡)를 실측한다.

왜 필요한가
  camera_info 의 fx 가 자르기 전 기준이라 틀리다 (I-031). 이 스크립트가 그 배율을 직접 잰다.

  **체커보드가 하는 일과 못 하는 일을 갈라 두자 (9/10 교훈).**
    하는 일   "지금 쓰는 값이 틀렸다"를 밝힌다. 9/8 까지 쓰던 1.7778 이 3~9 % 틀렸다는 것은
              이 도구로만 알 수 있었다. 대략 어디쯤인지(1.83 쯤)까지가 여기서 나온다.
    못 하는 일 소수점 아래를 확정한다. 같은 사진 묶음도 20장씩 골라 다시 풀면 답이 8 % 폭으로
              움직인다. RMS 는 오차 큰 사진을 자동으로 뺀 뒤의 값이라 그 흔들림을 안 보여준다.

  그래서 calib 이 **재현성 검사**를 같이 찍는다 (reproducibility, 합격선 2 %). 흔들림이 크면
  마지막 한 칸은 줄자가 좁힌다 — tools/aruco_live_check.py 로 운용 구간 3점 이상.
  지금 쓰는 1.8336 도 그렇게 나온 값이다: 체커보드가 범위를 주고 줄자가 그 안에서 확정했다.

두 단계로 쓴다.

  1) 사진 모으기 — 로봇 카메라가 보는 그 스트림 그대로
     python3 checker_calib.py capture --ns /robot8 --cols 9 --rows 6
       체커보드를 들고 화면 여기저기·여러 각도로 움직인다. 자동으로 좋은 장면만 저장한다.
       터미널이 "아직 안 채운 구역"을 알려준다. 20장쯤 모이면 Ctrl-C.

  2) 계산하기 — 로봇 없이, 저장된 사진으로
     python3 checker_calib.py calib --square 19.0 --cols 9 --rows 6
       --square 는 인쇄물의 한 칸을 **자로 잰 실제 mm** (설계값 19.0). 프린터가 축소해서 뽑으므로 꼭 재서 넣는다.

  3) 결과 다시 보기 — 값마다 좋은지 나쁜지 판정을 붙여 준다
     python3 checker_calib.py show

  --cols/--rows 는 **내부 코너 수**다. 10×7 칸짜리 판이면 9×6 이다. 칸 수가 아니다.
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SHOTS = os.path.join(HERE, 'checker_shots')

# 지금 코드가 쓰는 배율. 여기 숫자를 따로 적지 않는다 — 한쪽만 고쳐지는 사고를 막는다
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'mro_perception'))
from mro_perception.core.camera import FX_SCALE_MEASURED as FX_IN_USE   # noqa: E402


def tag_of(ns):
    """네임스페이스 → 파일 이름에 쓸 태그. `/robot8` → `robot8`."""
    return (ns or '').strip('/').replace('/', '_') or 'default'


def shots_dir(ns):
    """로봇마다 따로 쌓는다 — 한 폴더를 같이 쓰면 뒤에 찍은 로봇이 앞 로봇 사진을 지운다."""
    return os.path.join(SHOTS, tag_of(ns))


def result_path(ns):
    """그 로봇의 결과 npz 경로."""
    return os.path.join(HERE, f'checker_result_{tag_of(ns)}.npz')


def pick_ns(a):
    """--ns 를 안 줬을 때: 폴더가 하나뿐이면 그걸 쓰고, 여럿이면 물어본다."""
    if getattr(a, 'ns', None):
        return a.ns
    found = sorted(d for d in (os.listdir(SHOTS) if os.path.isdir(SHOTS) else [])
                   if os.path.isdir(os.path.join(SHOTS, d)))
    if len(found) == 1:
        print(f'--ns 를 안 줘서 {found[0]} 로 본다 (폴더가 하나뿐)')
        return found[0]
    sys.exit(f'--ns 를 지정하세요. 지금 있는 것: {found or "없음"}')


FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def find_board(gray, cols, rows, refine=True):
    """체커보드 내부 코너를 찾는다. → (N,1,2) 또는 못 찾으면 None."""
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), FLAGS)
    if not ok:
        return None
    if refine:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRIT)
    return corners


# ─────────────────────────────────────────────────────────────── 1) 사진 모으기
def capture(a):
    """로봇 카메라 스트림에서 쓸 만한 체커보드 장면만 골라 저장한다."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, CameraInfo

    d = shots_dir(a.ns)
    os.makedirs(d, exist_ok=True)
    old = sorted(glob.glob(os.path.join(d, 'shot_*.png')))
    if old and a.fresh:
        for f in glob.glob(os.path.join(d, '*')):
            os.remove(f)
        print(f'{tag_of(a.ns)} 의 이전 사진 {len(old)}장 지움 (--fresh)')
        old = []
    elif old:
        print(f'{tag_of(a.ns)} 에 이미 {len(old)}장 있다 — 지우지 않고 이어서 쌓는다 '
              f'(새로 찍으려면 --fresh)')

    start = int(old[-1].rsplit('_', 1)[1].split('.')[0]) if old else 0
    state = {'saved': [], 'cells': set(), 'n': start, 'last': 0.0,
             'seen': 0, 'found': 0, 'fx': None}

    def info_cb(msg):
        if state['fx'] is None:
            state['fx'] = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], len(msg.d))

    def img_cb(msg):
        state['seen'] += 1
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_board(gray, a.cols, a.rows)
        if corners is None:
            return
        state['found'] += 1
        now = time.time()
        if now - state['last'] < a.gap:
            return
        c = corners.reshape(-1, 2)
        ctr = c.mean(axis=0)
        area = cv2.contourArea(cv2.convexHull(c.astype(np.float32)))
        # 이미 비슷한 장면이 있으면 건너뛴다 — 같은 자세 20장은 도움이 안 된다
        for pc, pa in state['saved']:
            if np.linalg.norm(ctr - pc) < a.min_move and abs(area - pa) / max(pa, 1) < 0.25:
                return
        state['saved'].append((ctr, area))
        state['last'] = now
        state['n'] += 1
        cell = (min(2, int(ctr[0] / (w / 3))), min(2, int(ctr[1] / (h / 3))))
        state['cells'].add(cell)
        cv2.imwrite(os.path.join(d, f'shot_{state["n"]:03d}.png'), frame)
        vis = cv2.drawChessboardCorners(frame.copy(), (a.cols, a.rows), corners, True)
        cv2.imwrite(os.path.join(d, f'vis_{state["n"]:03d}.jpg'), vis)
        want = [f'{"좌중우"[x]}{"상중하"[y]}' for x in range(3) for y in range(3)
                if (x, y) not in state['cells']]
        print(f'  [{state["n"]:2d}장] 저장 · 판 크기 {np.sqrt(area):.0f} px · '
              f'남은 구역: {" ".join(want) if want else "없음 (다 채움)"}')

    rclpy.init()
    n = Node('checker_capture')
    n.create_subscription(CompressedImage, f'{a.ns}/oakd/rgb/image_raw/compressed',
                          img_cb, qos_profile_sensor_data)
    n.create_subscription(CameraInfo, f'{a.ns}/oakd/rgb/camera_info', info_cb, 10)
    print(f'{a.ns} 구독 중. 체커보드(내부 코너 {a.cols}×{a.rows})를 카메라 앞에서 움직이세요.')
    print('  · 화면 가운데만 말고 네 귀퉁이에도 오게 — 왜곡은 가장자리에서만 드러난다')
    print('  · 정면뿐 아니라 좌우·상하로 30° 쯤 기울여서도')
    print('  · 가까이(판이 화면 가득)와 멀리(1/4 크기) 둘 다')
    print('  20장쯤 모으고 Ctrl-C\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    # 요약을 먼저 찍는다 — shutdown 이 실패해도 사람이 볼 값은 남아야 한다
    print(f'\n프레임 {state["seen"]}장 받음 · 체커보드 검출 {state["found"]}장 · '
          f'저장 누적 {state["n"]}장 → {d}')
    if state['fx']:
        fx, fy, cx, cy, nd = state['fx']
        print(f'현재 camera_info: fx {fx:.1f} · fy {fy:.1f} · cx {cx:.1f} · cy {cy:.1f} · 왜곡 {nd}개')
        # 배율 = 실측 fx / 이 값. 로봇마다 다르니 넘겨짚지 말고 여기서 받아 적는다
        with open(os.path.join(d, 'camera_info.txt'), 'w') as fh:
            fh.write(f'{fx}\n')
        print(f'  camera_info fx 를 {os.path.join(d, "camera_info.txt")} 에 적어 뒀다 — calib 이 자동으로 쓴다')
    try:
        n.destroy_node()
    except Exception:
        pass
    if rclpy.ok():          # Ctrl-C 는 이미 컨텍스트를 내려놓는다 — 또 부르면 RCLError 로 크래시처럼 보인다
        rclpy.shutdown()
    if state['n'] < 12:
        print('⚠ 12장 미만이면 결과를 믿기 어렵다. 더 모으세요.')
    else:
        print(f'\n다음: python3 tools/checker_calib.py calib --ns {a.ns} '
              f'--square <자로 잰 한 칸 mm> --cols {a.cols} --rows {a.rows}')


def reproducibility(objpoints, imgpoints, size, flags, K0=None, trials=12, n=20, seed=0):
    """같은 사진 묶음을 나눠 다시 풀어 fx 가 얼마나 흔들리는지 잰다.

    → (반쪽 두 값, 무작위 목록).

    **RMS 만으로는 값을 믿을 수 없다.** RMS 는 오차 큰 사진을 자동으로 뺀 뒤의 값이라, 서로 말이
    맞는 사진만 골라내면 좋아 보인다 — 9/9 robot3 이 그랬다: 사진을 빼면서 RMS 가 0.73 → 0.31 로
    좋아졌는데 fx 는 1035 → 1096 으로 5.8 % 움직였다. 정확해진 게 아니라 자기들끼리 맞는
    부분집합을 고른 것이다.

    데이터가 fx 를 결정했다면 어느 부분집합으로 풀어도 같은 답이 나와야 한다. 그래서 ① 홀수장·
    짝수장으로 갈라 따로 풀고 ② 무작위 n 장씩 여러 번 풀어 폭을 본다. 2 % 를 합격선으로 둔다 —
    1.5 m 에서 3 cm 이고 관제 판정 기준 10 cm 의 1/3 이다.
    """
    def fx_of(obj, img):
        if len(obj) < 6:
            return float('nan')
        try:
            _, K, _, _, _ = cv2.calibrateCamera(obj, img, size, None if K0 is None else K0.copy(),
                                                None, flags=flags)
            return float(K[0, 0])
        except cv2.error:
            return float('nan')

    halves = (fx_of(objpoints[0::2], imgpoints[0::2]), fx_of(objpoints[1::2], imgpoints[1::2]))
    rng = np.random.default_rng(seed)
    vals = []
    if len(objpoints) >= n:
        for _ in range(trials):
            idx = rng.choice(len(objpoints), n, replace=False)
            vals.append(fx_of([objpoints[i] for i in idx], [imgpoints[i] for i in idx]))
    return halves, [v for v in vals if v == v]


def print_reproducibility(halves, vals, n=20):
    """재현성 결과를 판정과 함께 찍는다. 합격이면 체커보드 값을 그대로 써도 된다."""
    a, b = halves
    gap = abs(a / b - 1) * 100 if b == b and b else float('nan')
    print(f'\n{"─" * 66}\n이 값을 믿어도 되나 — 같은 사진을 나눠 다시 풀어 본다')
    print(f'  홀수장만 fx {a:8.2f}     짝수장만 fx {b:8.2f}     차이 {gap:5.1f} %')
    spread = gap
    if vals:
        lo, hi = min(vals), max(vals)
        spread = (hi / lo - 1) * 100
        print(f'  {n}장씩 무작위로 {len(vals)}번  fx {lo:.1f} ~ {hi:.1f}  폭 {spread:5.1f} % '
              f'= 1.5 m 에서 {spread * 1.5:.1f} cm')
    else:
        print(f'  (사진이 {n}장 미만이라 무작위 재추출은 건너뜀)')
    worst = max(gap, spread)
    if worst <= 2.0:
        print(f'  → 합격 ({worst:.1f} % ≤ 2 %). 이 사진들이 fx 를 한 값으로 좁혔다.')
        print('     그래도 마지막 판정은 줄자로 한다 — tools/aruco_live_check.py 로 3점 이상.')
    else:
        print(f'  → **소수점 아래는 이 사진들로 못 정한다** (흔들림 {worst:.1f} % > 2 %).')
        print('     "대략 얼마쯤"까지는 이 결과가 맞다 — 지금 쓰는 값이 틀렸는지 아닌지는 여기서 알 수 있다.')
        print('     하지만 위 폭 안 어디가 참값인지는 **줄자가 정한다** — aruco_live_check.py 로 3점 이상.')
        print('     (흔들림을 줄이려면: 판을 하드보드에 평평하게 · 흐린 장면 배제 · 각도와 거리를 더 다양하게)')
    return worst


# ─────────────────────────────────────────────────────────────── 2) 계산하기
def calib(a):
    """저장된 사진으로 내부 파라미터를 풀고, 그 값을 믿어도 되는지까지 판정한다."""
    ns = pick_ns(a)
    d = shots_dir(ns)
    files = sorted(glob.glob(os.path.join(d, 'shot_*.png')))
    if not files:
        sys.exit(f'{d} 에 사진이 없다. 먼저 capture 를 돌리세요.')

    objp = np.zeros((a.rows * a.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * (a.square / 1000.0)

    objpoints, imgpoints, used, size = [], [], [], None
    for f in files:
        img = cv2.imread(f)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        c = find_board(gray, a.cols, a.rows)
        if c is None:
            print(f'  건너뜀 (검출 실패): {os.path.basename(f)}')
            continue
        objpoints.append(objp)
        imgpoints.append(c)
        used.append(os.path.basename(f))
    if len(objpoints) < 6:
        sys.exit(f'쓸 수 있는 사진이 {len(objpoints)}장뿐이다. 최소 6장, 권장 15장 이상.')

    flags = cv2.CALIB_RATIONAL_MODEL if a.rational else 0

    def run(obj, img):
        rms, K, D, rv, tv = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
        er = [float(np.linalg.norm(img[i] - cv2.projectPoints(obj[i], rv[i], tv[i], K, D)[0],
                                   axis=2).mean())
              for i in range(len(obj))]
        return rms, K, D, np.array(er)

    rms, K, D, errs = run(objpoints, imgpoints)
    # 흔들리거나 판이 휜 사진 한두 장이 결과를 통째로 끌고 간다 — 2차로 걸러 다시 푼다
    thr = a.max_err if a.max_err > 0 else max(0.8, 2.5 * float(np.median(errs)))
    keep = [i for i in range(len(errs)) if errs[i] <= thr]
    dropped = [used[i] for i in range(len(errs)) if errs[i] > thr]
    if dropped and len(keep) >= 10:
        print(f'  1차 RMS {rms:.3f} px · 오차 {thr:.2f} px 넘는 {len(dropped)}장 빼고 다시 계산 '
              f'({", ".join(dropped[:5])}{" …" if len(dropped) > 5 else ""})')
        objpoints = [objpoints[i] for i in keep]
        imgpoints = [imgpoints[i] for i in keep]
        used = [used[i] for i in keep]
        rms, K, D, errs = run(objpoints, imgpoints)

    halves, boot = reproducibility(objpoints, imgpoints, size, flags)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f'\n{"="*66}\n{tag_of(ns)} · 사진 {len(objpoints)}장 · '
          f'해상도 {size[0]}×{size[1]} · 한 칸 {a.square} mm')
    if size != (704, 704):
        print('⚠ 704×704 가 아니다. 로봇이 쓰는 그 스트림으로 다시 찍어야 의미가 있다.')
    print(f'{"="*66}')
    print(f'  전체 재투영 오차(RMS)  {rms:.3f} px    {"좋음" if rms < 0.5 else "높다 — 판이 휘었거나 흐린 사진이 섞였다"}')
    print(f'  fx {fx:8.2f}     fy {fy:8.2f}')
    print(f'  cx {cx:8.2f}     cy {cy:8.2f}      (영상 중앙 {size[0]/2:.1f}, {size[1]/2:.1f})')
    print('  cx·cy 가 중앙에서 크게 벗어나면 사진이 화면 한쪽에만 몰린 것이다')
    print(f'  왜곡계수 {len(D.ravel())}개  ' + ' '.join(f'{v:+.4f}' for v in D.ravel()))
    worst = sorted(zip(errs, used), reverse=True)[:3]
    print('  오차 큰 사진: ' + ', '.join(f'{n} {e:.2f}px' for e, n in worst))

    worst = print_reproducibility(halves, boot)

    print(f'\n{"─"*66}\n지금 설정과 비교')
    saved = os.path.join(d, 'camera_info.txt')
    if a.info_fx > 0:
        info_fx = a.info_fx
    elif os.path.exists(saved):
        info_fx = float(open(saved).read().strip())
        print(f'  (camera_info fx {info_fx:.1f} — 찍을 때 이 로봇에서 받아 적은 값)')
    else:
        sys.exit('camera_info fx 를 모른다. --info-fx 로 주거나, capture 를 다시 돌리세요.')
    ratio = fx / info_fx
    print(f'  camera_info 가 싣고 오는 fx      {info_fx:.1f}')
    print(f'  실측 fx                          {fx:.1f}')
    print(f'  → 실제 배율(camera_fx_scale)     {ratio:.4f}')
    print(f'  지금 코드가 쓰는 값              {FX_IN_USE:.4f}  (줄자로 확정, core/camera.py)')
    diff = (ratio / FX_IN_USE - 1) * 100
    print(f'  차이                             {diff:+.2f} %  '
          f'= 1.5 m 에서 {abs(diff) * 1.5:.1f} cm 의 pnp 거리 오차')
    print('\n  이 값으로 줄자 대조를 해 보려면 (코드 수정 없음):')
    print(f'    python3 tools/aruco_live_check.py --ns /{tag_of(ns)} '
          f'--scale {ratio:.4f} --tape <줄자값>')
    print(f'  운용 구간 0.9~1.5 m 에서 3점 이상. 지금 값({FX_IN_USE:.4f})보다 나아야 바꾼다.')
    print('  바꾼다면 perception.yaml 의 camera_fx_scale (두 노드 공용 — 짝에게 먼저 알린다)')
    np.savez(result_path(ns), K=K, D=D, rms=rms, size=size, ns=tag_of(ns),
             square_mm=a.square, ratio=ratio, n=len(objpoints),
             halves=np.array(halves), boot=np.array(boot), worst=worst)
    print(f'\n  결과 저장: {os.path.basename(result_path(ns))}')


# ─────────────────────────────────────────────────────────────── 3) 결과 다시 보기
def show(a):
    """checker_result.npz 를 사람이 읽는 형태로 — 각 값이 좋은지 나쁜지 판정까지."""
    ns = pick_ns(a)
    f = result_path(ns)
    if not os.path.exists(f):
        sys.exit(f'{f} 가 없다. 먼저 calib --ns {ns} 를 돌리세요.')
    z = np.load(f)
    K, D, rms = z['K'], z['D'], float(z['rms'])
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    w, h = (int(v) for v in z['size'])
    ratio = float(z['ratio'])

    def ok(c):
        return '좋음' if c else '⚠ 확인 필요'

    print(f'\n{"="*68}')
    print(f'  {tag_of(ns)} · 사진 {int(z["n"])}장 · 해상도 {w}×{h} · '
          f'한 칸 {float(z["square_mm"]):.1f} mm')
    print(f'{"="*68}')
    print('\n[1] 계산이 잘 됐나 — 재투영 오차')
    print(f'    RMS {rms:.3f} px      {ok(rms < 0.5)}   (0.5 미만이면 좋음)')
    if rms >= 0.5:
        print('        판이 휘었거나 흐린 사진이 섞였다. checker_shots/vis_*.jpg 를 눈으로 확인.')

    print('\n[2] 초점거리 — 이게 우리가 재려던 값이다')
    print(f'    fx {fx:8.2f}    fy {fy:8.2f}')
    r = fx / fy
    print(f'    fx/fy {r:.4f}        {ok(abs(r - 1) < 0.02)}   (1.0 근처여야 한다 — 등방)')

    print('\n[3] 주점 — 영상 중앙이어야 한다')
    print(f'    cx {cx:7.2f}  (중앙 {w/2:.1f}, 차이 {cx-w/2:+.1f} px)')
    print(f'    cy {cy:7.2f}  (중앙 {h/2:.1f}, 차이 {cy-h/2:+.1f} px)')
    off = max(abs(cx - w/2), abs(cy - h/2))
    print(f'    {ok(off < w * 0.05)}   (중앙에서 {w*0.05:.0f} px 넘게 벗어나면 사진이 한쪽에 몰린 것)')

    print(f'\n[4] 왜곡계수 {len(D.ravel())}개')
    print('    ' + '  '.join(f'{v:+.4f}' for v in D.ravel()))
    print(f'    k1 {D.ravel()[0]:+.4f} — 음수면 통형(barrel), 양수면 실패형(pincushion). 광각 렌즈는 보통 음수')

    print('\n[5] 이 값을 믿어도 되나 — 같은 사진을 나눠 다시 풀어 본 결과')
    worst = float(z['worst']) if 'worst' in z.files else float('nan')
    if 'halves' in z.files:
        a, b = (float(v) for v in z['halves'])
        print(f'    홀수장만 fx {a:8.2f}     짝수장만 fx {b:8.2f}')
    if 'boot' in z.files and len(z['boot']):
        bt = np.asarray(z['boot'], dtype=float)
        print(f'    20장씩 무작위   fx {bt.min():.1f} ~ {bt.max():.1f}')
    if worst == worst:
        print(f'    가장 큰 흔들림  {worst:.1f} %      {ok(worst <= 2.0)}   (2 % 이하여야 한다)')
    else:
        print('    (이 npz 는 재현성 검사 전에 만들어졌다 — calib 을 다시 돌리면 나온다)')

    print('\n[6] 결론 — 지금 설정과 얼마나 다른가')
    print(f'    이 사진들이 낸 배율          {ratio:.4f}')
    print(f'    지금 코드가 쓰는 값          {FX_IN_USE:.4f}   (줄자로 확정, core/camera.py)')
    d = (ratio / FX_IN_USE - 1) * 100
    print(f'    차이                         {d:+.2f} %   = 1.5 m 에서 {abs(d)*1.5:.1f} cm')
    if worst == worst and worst > 2.0:
        print(f'    → 이 {ratio:.4f} 는 **후보이지 확정값이 아니다.** [5] 에서 답이 {worst:.0f} % 폭으로')
        print('       움직였으므로 이것도 그 안의 한 점이다. 바꾸려면 줄자로 먼저 확인한다.')
    elif abs(d) < 1:
        print('    → 지금 값과 사실상 같다. 바꿀 이유가 없다')
    else:
        print(f'    → 재현성은 통과했다. 바꾸려면 줄자로 지금 값({FX_IN_USE:.4f})보다 나은지 먼저 본다')

    print('\n[7] 다음 — **판정은 언제나 줄자가 한다**')
    print(f'    python3 tools/aruco_live_check.py --ns /{tag_of(ns)} '
          f'--scale {ratio:.4f} --tape <줄자값>')
    print('    운용 구간 0.9~1.5 m 에서 3점 이상. 화면의 implied marker 가 10.0 cm 로 와야 한다')
    print('    (마커에 흰 여백이 없으면 이 값이 맞아도 거리가 틀린다 — 계약 6절)\n')


ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
sub = ap.add_subparsers(dest='cmd', required=True)
c1 = sub.add_parser('capture', help='로봇 카메라에서 체커보드 사진 모으기')
c1.add_argument('--ns', default='/robot8')
c1.add_argument('--gap', type=float, default=0.7, help='저장 최소 간격(초)')
c1.add_argument('--min-move', type=float, default=60.0, help='이만큼(px) 움직여야 새 장면으로 본다')
c1.add_argument('--fresh', action='store_true', help='그 로봇 폴더를 비우고 새로 찍는다 (기본은 이어서 쌓기)')
c2 = sub.add_parser('calib', help='모은 사진으로 계산')
c2.add_argument('--ns', default=None, help='어느 로봇 사진으로 계산할지 (예 /robot8)')
c2.add_argument('--square', type=float, required=True, help='자로 잰 한 칸 크기 (mm)')
c2.add_argument('--info-fx', type=float, default=0.0,
                help='camera_info 의 fx. 0 이면 찍을 때 저장해 둔 값을 쓴다 (로봇마다 다르다)')
c2.add_argument('--max-err', type=float, default=0.0,
                help='이 재투영 오차(px)를 넘는 사진은 빼고 다시 계산. 0 이면 자동')
c2.add_argument('--rational', action='store_true', help='왜곡을 8계수(rational)로 — camera_info 와 같은 모델')
c3 = sub.add_parser('show', help='저장된 결과를 판정과 함께 다시 본다')
c3.add_argument('--ns', default=None, help='어느 로봇 결과를 볼지 (예 /robot8)')
for c in (c1, c2):
    c.add_argument('--cols', type=int, default=9, help='내부 코너 가로 개수')
    c.add_argument('--rows', type=int, default=6, help='내부 코너 세로 개수')
a = ap.parse_args()
{'capture': capture, 'calib': calib, 'show': show}[a.cmd](a)
