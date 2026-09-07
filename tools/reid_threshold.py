#!/usr/bin/env python3
"""재등록 임계값(`rematch_threshold`) 분포 측정 — 계약 v1.3 '눈대중 금지'.

  python3 tools/reid_threshold.py --dataset 2PM_Project_yolo/2PM_Project_yolo

웹캠1 은 트랙이 끊겼다 다시 들어온 대상이 기존 등록자인지 색 서술자로 판정한다
(`webcam_register_node.py`). S1 동선에서 작업자는 웹캠1 시야를 두 번 지나므로
**재등록 판정이 반드시 일어난다.** 임계를 눈대중으로 두면 두 가지로 깨진다.

  너무 높다  같은 인형인데 못 알아본다 → 되돌아온 작업자가 재등록되지 않는다.
             MAX_WORKERS 가 이미 차 있어 새로 등록되지도 않으므로 그대로 방치된다.
  너무 낮다  다른 인형을 같은 사람으로 본다 → W1·W2 가 뒤바뀐다 (AC-04 ID Swap).

지금 코드는 **같은 복장 클래스끼리만** 비교하므로(`outfit != cls_name: continue`)
검출이 정확한 한 아래쪽 위험은 구조적으로 막혀 있다. 그래서 실제로 무서운 것은
**너무 높은 쪽**이다. 다만 YOLO 가 worker_a/worker_b 를 뒤바꾸는 순간에는 아래쪽도
살아나므로 두 분포를 다 낸다.

재는 법
  - 서술자는 `mro_perception.core.reid.hsv_hist()` 를 그대로 호출한다. 웹캠1·AMR 과
    같은 함수를 써야 여기서 정한 값이 실제 판정과 같은 척도가 된다 (계약 2절).
  - crop 은 라벨 bbox 전체 — 계약 v1.3 이 '인형 머리부터 바지 밑단까지' 로 확정했다.
  - 같은 인형 쌍 / 다른 인형 쌍을 전부 돌려 분포를 낸다.

한계 — 데이터셋은 학습용으로 찍은 것이라 거리·각도가 실제 출입구 배치와 다르다.
여기서 나온 값은 **출발점**이고, 실기동 로그의 `재등록 ... (sim ...)` 값과 대조해야 한다.
"""
import argparse
import itertools
import random
import sys
from pathlib import Path

# 데이터셋 원본 이름 → 계약 이름 (mro_perception/models/README.md)
CLASS_MAP = {1: 'worker_b', 2: 'worker_a'}   # 0 = amr, 등록 대상이 아니다
IMG_EXT = ('.jpg', '.jpeg', '.png')


def crops_by_class(root, splits):
    """라벨 bbox 를 잘라 클래스별 히스토그램 목록으로 돌려준다."""
    import cv2
    from mro_perception.core.reid import hsv_hist

    out = {name: [] for name in CLASS_MAP.values()}
    for split in splits:
        label_dir, image_dir = root / split / 'labels', root / split / 'images'
        if not label_dir.is_dir():
            continue
        for lbl in sorted(label_dir.glob('*.txt')):
            img_path = next((p for e in IMG_EXT
                             if (p := image_dir / (lbl.stem + e)).exists()), None)
            if img_path is None:
                continue
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            h, w = img.shape[:2]
            for line in lbl.read_text().split('\n'):
                f = line.split()
                if len(f) < 5 or int(f[0]) not in CLASS_MAP:
                    continue
                cx, cy, bw, bh = (float(v) for v in f[1:5])   # YOLO 정규화 좌표
                x1, y1 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
                x2, y2 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
                crop = img[max(0, y1):y2, max(0, x1):x2]
                if crop.size == 0:
                    continue
                out[CLASS_MAP[int(f[0])]].append(hsv_hist(crop))
    return out


def stats(xs):
    s = sorted(xs)
    def q(p):
        return s[min(len(s) - 1, int(round(p * (len(s) - 1))))]
    return {'n': len(s), 'min': s[0], 'p5': q(0.05), 'p50': q(0.50),
            'p95': q(0.95), 'max': s[-1]}


def line(label, d):
    return (f'  {label:24s} n={d["n"]:6d}   min {d["min"]:.3f}   p5 {d["p5"]:.3f}   '
            f'p50 {d["p50"]:.3f}   p95 {d["p95"]:.3f}   max {d["max"]:.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True, help='data.yaml 이 있는 폴더')
    ap.add_argument('--splits', nargs='+', default=['train', 'valid', 'test'])
    ap.add_argument('--max-pairs', type=int, default=40000, help='묶음당 표본 상한')
    ap.add_argument('--current', type=float, default=0.7, help='지금 쓰는 임계값')
    ap.add_argument('--seed', type=int, default=42)
    a = ap.parse_args()

    from mro_perception.core.reid import similarity
    random.seed(a.seed)

    hists = crops_by_class(Path(a.dataset).expanduser(), a.splits)
    for name, hs in hists.items():
        if len(hs) < 2:
            sys.exit(f'{name} crop 이 {len(hs)}개뿐이다 — 분포를 낼 수 없다')

    def sample(pairs):
        pairs = list(pairs)
        if len(pairs) > a.max_pairs:
            pairs = random.sample(pairs, a.max_pairs)
        return [similarity(x, y) for x, y in pairs]

    same = {name: sample(itertools.combinations(hs, 2)) for name, hs in hists.items()}
    cross = sample(itertools.product(hists['worker_a'], hists['worker_b']))

    print()
    print('=' * 78)
    print('  재등록 임계값 분포 — mro_perception.core.reid.hsv_hist / similarity')
    print(f'  crop {len(hists["worker_a"])} (worker_a) + {len(hists["worker_b"])} (worker_b)'
          f'   splits={",".join(a.splits)}')
    print('=' * 78)
    print()
    print('같은 인형끼리 — 재등록이 성공해야 하는 쌍')
    s_a, s_b = stats(same['worker_a']), stats(same['worker_b'])
    print(line('worker_a ↔ worker_a', s_a))
    print(line('worker_b ↔ worker_b', s_b))
    print()
    print('다른 인형끼리 — 붙으면 W1·W2 가 뒤바뀐다 (AC-04)')
    c = stats(cross)
    print(line('worker_a ↔ worker_b', c))
    print()

    floor = min(s_a['p5'], s_b['p5'])       # 같은 인형이 이 밑으로는 잘 안 내려간다
    ceil_ = c['p95']                        # 다른 인형이 이 위로는 잘 안 올라간다
    print('-' * 78)
    print(f'  같은 인형 p5   {floor:.3f}   ← 임계가 이보다 높으면 재등록을 놓치기 시작한다')
    print(f'  다른 인형 p95  {ceil_:.3f}   ← 임계가 이보다 낮으면 뒤바뀔 여지가 생긴다')
    if floor > ceil_:
        rec = round((floor + ceil_) / 2, 2)
        print(f'  두 분포가 갈린다 (여유 {floor - ceil_:.3f}). 권고 = 가운데 {rec:.2f}')
    else:
        rec = round(floor - 0.02, 2)
        print(f'  두 분포가 겹친다 (겹침 {ceil_ - floor:.3f}). 한 값으로 둘 다 만족시킬 수 없다.')
        print('  같은 복장끼리만 비교하므로(코드) 놓치는 쪽이 더 아프다 → 아래쪽에 붙인다.')
        print(f'  권고 = 같은 인형 p5 바로 아래 {rec:.2f}')
    miss_a = sum(1 for v in same['worker_a'] if v < a.current) / len(same['worker_a']) * 100
    miss_b = sum(1 for v in same['worker_b'] if v < a.current) / len(same['worker_b']) * 100
    swap = sum(1 for v in cross if v >= a.current) / len(cross) * 100
    print()
    print(f'  지금 값 {a.current:.2f} 로 두면')
    print(f'    같은 인형인데 못 알아보는 쌍   worker_a {miss_a:5.1f} %   worker_b {miss_b:5.1f} %')
    print(f'    다른 인형인데 붙는 쌍          {swap:5.1f} %')
    print('-' * 78)
    print()


if __name__ == '__main__':
    main()
