#!/usr/bin/env python3
"""서술자 crop 이 몇 픽셀인지 실제 배치에서 잰다 — I-015 전제 재확인.

  python3 tools/measure_crop.py --model <worker.pt> --index 2 --seconds 20

왜 재는가
  계약 v1.3 이 `color_hist` 추출 영역을 **인형 전체(머리~바지 밑단)** 로 넓힌 근거는
  I-015 였다 — "상의만 자르면 웹캠1 거리에서 픽셀이 부족해 히스토그램이 뭉개진다".
  그때 웹캠1 은 **관제탑 구역까지** 보고 있었다.

  그런데 계약 v1.10(9/7)에서 웹캠1 시야가 **출입구로 좁혀졌다.** 거리가 가까워졌으므로
  그 전제가 아직 유효한지 다시 재야 한다 (인지 정정인, 9/7). 전제가 깨지면 crop 을
  상의로 되돌릴 수 있고, 그러면 하의·머리라는 **공통 성분이 빠져 두 인형의 분포가 갈린다**
  — tools/reid_threshold.py 가 보여준 '분포 겹침' 이 근본적으로 해소된다.

무엇을 내는가
  검출된 인형마다 bbox 전체 면적과, 그중 **상의에 해당하는 위쪽 띠**의 면적을 센다.
  상의 비율은 실측 치수에서 온다 — 전고 265 mm(worker_a) / 285 mm(worker_b) 중
  상의 120 mm (SDD HW-09, I-005). 머리가 위에 있으므로 상의 띠는 bbox 맨 위가 아니라
  머리 아래에서 시작한다. --head-ratio 로 조정한다.

판단 기준
  HSV 히스토그램은 H 16 x S 8 = **128 빈**이다. 빈당 평균 10 px 는 있어야 모양이 남는다고
  보면 상의 영역이 대략 **1,300 px² 이상**이어야 한다. 그 아래면 상의만으로는 뭉개지므로
  계약 v1.3 의 '인형 전체' 를 유지해야 한다. 이 값은 판단 근거이지 계약값이 아니다.

  카메라·인형 배치를 실제 시연 상태로 두고 재야 의미가 있다.
"""
import argparse
import statistics as st
import sys
import time

TARGET = ('worker_a', 'worker_b')
BINS = 16 * 8               # reid.hsv_hist 의 빈 수
PX_PER_BIN = 10             # 빈당 이만큼은 있어야 모양이 남는다고 본다
SHIRT_MM = 120.0            # 상의 세로 (SDD HW-09)
HEIGHT_MM = {'worker_a': 265.0, 'worker_b': 285.0}   # 전고 (I-005 실측)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--index', type=int, default=2, help='/dev/videoN')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--head-ratio', type=float, default=0.20,
                    help='bbox 위쪽에서 머리가 차지하는 비율 — 상의 띠는 그 아래부터')
    a = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    cap = cv2.VideoCapture(a.index)
    if not cap.isOpened():
        sys.exit(f'/dev/video{a.index} 를 열 수 없다. v4l2-ctl --list-devices 로 확인한다')
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    model = YOLO(a.model)
    names = model.names

    need = BINS * PX_PER_BIN
    print(f'\n/dev/video{a.index} · conf {a.conf} · {a.seconds:.0f} 초 수집')
    print(f'상의 판단 기준 {need:,} px²  (H16 x S8 = {BINS} 빈 x {PX_PER_BIN} px)\n')

    boxes = {k: [] for k in TARGET}
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        ok, img = cap.read()
        if not ok or img is None:
            continue
        r = model.predict(img, conf=a.conf, verbose=False)[0]
        if r.boxes is None:
            continue
        for b in r.boxes:
            cls = names.get(int(b.cls[0]), '?')
            if cls not in TARGET:
                continue
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            boxes[cls].append((max(1, x2 - x1), max(1, y2 - y1)))
    cap.release()

    if not any(boxes.values()):
        sys.exit('인형이 한 번도 잡히지 않았다. 시야 안에 두고 다시 돌린다')

    print('=' * 70)
    for cls in TARGET:
        v = boxes[cls]
        if not v:
            print(f'  {cls:9s} 검출 없음')
            continue
        areas = [w * h for w, h in v]
        med_w = st.median(w for w, _ in v)
        med_h = st.median(h for _, h in v)
        # 상의 띠: 머리 아래에서 시작해 (상의 mm / 전고 mm) 만큼
        shirt_frac = SHIRT_MM / HEIGHT_MM[cls]
        shirt_h = med_h * shirt_frac
        shirt_area = med_w * shirt_h
        print(f'  {cls}   검출 {len(v)}회')
        print(f'    bbox        중앙 {med_w:.0f} x {med_h:.0f} px'
              f'   면적 중앙 {st.median(areas):,.0f} px²'
              f'   최소 {min(areas):,.0f} px²')
        print(f'    상의 띠     {med_w:.0f} x {shirt_h:.0f} px'
              f'   면적 {shirt_area:,.0f} px²'
              f'   (전고 {HEIGHT_MM[cls]:.0f} mm 중 상의 {SHIRT_MM:.0f} mm)')
        verdict = '충분 — 상의만으로도 히스토그램이 성립한다' if shirt_area >= need \
            else '부족 — 계약 v1.3 의 인형 전체 crop 을 유지해야 한다'
        print(f'    판정        {verdict}')
        print()
    print('-' * 70)
    print(f'  I-015 (9/6, 관제탑까지 보던 시야) 인형 전체 3,640 px² 와 대조한다.')
    print(f'  지금이 그보다 크면 시야를 좁힌 효과이고, 상의만으로도 될 여지가 생긴다.')
    print('=' * 70 + '\n')


if __name__ == '__main__':
    main()
