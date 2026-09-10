#!/usr/bin/env python3
"""인형별 기준 색 지문을 데이터셋에서 만들어 models/worker_hist.json 으로 굽는다 (I-058).

  python3 tools/make_worker_hist.py --dataset 2PM_Project_yolo/2PM_Project_yolo

왜 굽는가 — 종전에는 웹캠1 이 등록 순간에 본 것으로 지문을 만들었다. 그러면 지문의
품질이 그 1 초의 운(거리·각도·조명·가림)에 걸린다. 실기동에서 등록 4 초 사이 지문이
0.92 → 0.43 까지 흔들려 자동 페어링이 한 번도 안 걸린 적이 있다 (I-058).

인형은 **둘뿐이고 생김새가 고정**이다. 매번 새로 재는 것보다 데이터셋 전체(거리·각도·
조명이 골고루 섞인 1,350 장)의 평균을 한 번 구워 두는 편이 안정적이다. 실측 비교:

  사전 지문   같은 인형 0.527~0.626 · 다른 인형 0.423~0.498   (안 겹침)
  등록 지문   같은 인형 0.348~0.470 · 다른 인형 0.248~0.361   (겹침)

crop 은 라벨 bbox 전체 — 계약 v1.3 이 '인형 머리부터 바지 밑단까지' 로 확정했다.
hsv_hist() 를 그대로 호출하므로 웹캠1·AMR 과 같은 척도다 (계약 2절).
"""
import argparse
import json
import sys
from datetime import date
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reid_threshold import crops_by_class   # noqa: E402  (같은 crop 규칙을 공유한다)


def main():
    """데이터셋에서 클래스별 평균 지문을 구워 json 으로 쓴다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True, help='data.yaml 이 있는 폴더')
    ap.add_argument('--splits', nargs='+', default=['train', 'valid', 'test'])
    ap.add_argument('--out', default='ros2_ws/src/mro_perception/models/worker_hist.json')
    a = ap.parse_args()

    from mro_perception.core.reid import similarity

    hists = crops_by_class(Path(a.dataset).expanduser(), a.splits)
    out = {'_source': {'dataset': a.dataset, 'splits': a.splits, 'made': str(date.today()),
                       'note': 'hsv_hist H16xS8=128, L2=1. tools/make_worker_hist.py 로 생성'},
           '_counts': {k: len(v) for k, v in hists.items()}}
    canon = {}
    for name, hs in sorted(hists.items()):
        if len(hs) < 2:
            sys.exit(f'{name} crop 이 {len(hs)}개뿐이다 — 기준 지문을 만들 수 없다')
        m = np.mean(np.stack(hs), axis=0).astype(np.float32)
        m /= float(np.linalg.norm(m))
        canon[name] = m
        out[name] = [round(float(v), 8) for v in m]
        print(f'  {name:9s} {len(hs):5d} 장 평균')

    names = sorted(canon)
    print(f'\n  두 기준 지문 사이 유사도 {similarity(canon[names[0]], canon[names[1]]):.3f}'
          f'  (낮을수록 잘 갈린다)')
    Path(a.out).write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print(f'  → {a.out}')


if __name__ == '__main__':
    main()
