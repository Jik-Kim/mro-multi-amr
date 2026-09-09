#!/usr/bin/env python3
"""YOLO 추론 속도 측정 — 배포 PC 에서 10 Hz 가 나오는지 확인한다.

  python3 tools/bench_yolo.py --model <worker.pt> --source <이미지 폴더> --imgsz 640

계약 4절이 웹캠 추론 발행과 AMR OAK-D 를 **10 Hz** 로 정한다. 그러려면 호출 하나가
100 ms 안에 끝나야 한다. 학습 PC 에서 잰 값은 배포 PC 와 무관하므로 **돌릴 PC 에서 직접 잰다.**

  PC-W1  webcam_register   model.track   PC-R3  amr_detection  model.predict
  PC-W2  webcam_hint       model.track   PC-R8  amr_detection  model.predict

왜 이렇게 재는가
  - **벽시계로 호출 전체를 감싼다.** ultralytics 의 `results.speed` 는 전처리·추론·후처리만 세고
    ByteTrack(`model.track`) 시간이 빠진다. 타이머 콜백이 실제로 겪는 것은 호출 전체다.
  - **웜업 15회를 버린다.** 첫 호출들은 CUDA 초기화·cudnn autotune 때문에 몇 배 느리다.
  - **실제 이미지를 돌린다.** 검은 화면·노이즈는 검출이 0건이라 NMS 가 비현실적으로 빨라진다.
  - **평균이 아니라 max 를 본다.** 10 Hz 타이머는 한 프레임만 100 ms 를 넘겨도 그 주기를 흘린다.
    `webcam_hint` 는 두 타이머가 겹쳐 돌 수 있어(I-027) 최악값이 특히 중요하다.

가능하면 다른 노드가 함께 도는 상태에서 잰다 — 웹캠 PC 는 캡처 스레드가 같이 돈다.
모델·클래스 정의는 `mro_perception/models/README.md`. 담당 인지 정정인·민범진
"""
import argparse
import statistics as st
import sys
import time
from pathlib import Path

IMG_EXT = {'.jpg', '.jpeg', '.png', '.bmp'}


def load_frames(source, limit=60):
    import cv2
    p = Path(source).expanduser()
    if p.is_dir():
        files = sorted(f for f in p.iterdir() if f.suffix.lower() in IMG_EXT)[:limit]
    elif p.is_file():
        files = [p]
    else:
        sys.exit(f'source 를 찾을 수 없다: {p}')
    if not files:
        sys.exit(f'이미지가 없다: {p}')
    frames = [cv2.imread(str(f)) for f in files]
    frames = [f for f in frames if f is not None]
    if not frames:
        sys.exit('이미지를 읽지 못했다')
    return frames


def bench(model, frames, mode, n, warmup):
    call = (lambda im: model.track(im, persist=True, verbose=False)) if mode == 'track' \
        else (lambda im: model.predict(im, verbose=False))

    for i in range(warmup):
        call(frames[i % len(frames)])

    wall, parts = [], []
    for i in range(n):
        im = frames[i % len(frames)]
        t0 = time.perf_counter()
        r = call(im)
        wall.append((time.perf_counter() - t0) * 1000.0)
        try:
            s = r[0].speed
            parts.append((s.get('preprocess', 0), s.get('inference', 0), s.get('postprocess', 0)))
        except (IndexError, AttributeError, TypeError):
            pass
    return wall, parts


def p95(xs):
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--source', required=True, help='이미지 폴더 또는 파일')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--mode', default='both', choices=['predict', 'track', 'both'])
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--warmup', type=int, default=15)
    ap.add_argument('--budget-hz', type=float, default=10.0, help='계약 발행 주기 (기본 10 Hz)')
    a = ap.parse_args()

    import torch
    from ultralytics import YOLO

    dev = 'cuda' if torch.cuda.is_available() else ('mps' if torch.backends.mps.is_available() else 'cpu')
    model = YOLO(a.model)
    model.overrides['imgsz'] = a.imgsz

    frames = load_frames(a.source)
    budget = 1000.0 / a.budget_hz

    print()
    print('=' * 66)
    print(f'  모델      {Path(a.model).name}   imgsz={a.imgsz}')
    print(f'  클래스    {model.names}')
    print(f'  장치      {dev}   (torch {torch.__version__})')
    if dev == 'cuda':
        print(f'  GPU       {torch.cuda.get_device_name(0)}')
    print(f'  프레임    실제 이미지 {len(frames)}장 순환 · 웜업 {a.warmup} 버림 · 측정 {a.n}회')
    print(f'  예산      {a.budget_hz:.0f} Hz → 호출 하나가 {budget:.0f} ms 안에 끝나야 한다')
    print('=' * 66)

    modes = ['predict', 'track'] if a.mode == 'both' else [a.mode]
    for mode in modes:
        wall, parts = bench(model, frames, mode, a.n, a.warmup)
        lo, hi, med = min(wall), max(wall), st.median(wall)
        note = '  ← webcam_register·webcam_hint 가 쓰는 호출' if mode == 'track' else ''
        print()
        print(f'[{mode}]{note}')
        print(f'  min {lo:7.1f} ms   median {med:7.1f} ms   mean {st.mean(wall):7.1f} ms'
              f'   p95 {p95(wall):7.1f} ms   max {hi:7.1f} ms')
        if parts:
            pre, inf, post = (st.mean(x) for x in zip(*parts))
            print(f'  내역(평균)  전처리 {pre:.1f} · 추론 {inf:.1f} · 후처리 {post:.1f} ms'
                  f'   → 합 {pre + inf + post:.1f} ms')
            gap = st.mean(wall) - (pre + inf + post)
            if gap > 1.0:
                print(f'  ultralytics 가 안 세는 시간 {gap:.1f} ms'
                      + ('  (ByteTrack 등)' if mode == 'track' else ''))
        over = sum(1 for w in wall if w > budget)
        if hi <= budget:
            print(f'  판정  통과 — 최악값도 예산 안 ({budget - hi:.1f} ms 여유)')
        elif med <= budget:
            print(f'  판정  불안정 — 중앙값은 통과하나 {over}/{len(wall)}회'
                  f' ({100 * over / len(wall):.1f} %)가 {budget:.0f} ms 초과. 최악 {hi:.1f} ms')
        else:
            print(f'  판정  미달 — 중앙값 {med:.1f} ms > {budget:.0f} ms.'
                  f' 더 작은 모델이나 낮은 imgsz 필요')
    print()


if __name__ == '__main__':
    main()
