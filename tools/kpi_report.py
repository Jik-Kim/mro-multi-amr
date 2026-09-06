#!/usr/bin/env python3
"""kpi_logger CSV → 추종거리 p95 · IPH · 리드타임 · 성공률 · Lost 재획득 시간 요약.

  python3 tools/kpi_report.py ~/mro_kpi/kpi_20260909_140000.csv
  python3 tools/kpi_report.py kpi_*.csv          # 여러 세션 합산

정의는 BRD v3.1 5.3 기준.
  - 개입(IPH) 에서 SAFETY_POLICY 는 제외한다 — 안전 정책에 의한 정지는 개입이 아니다
  - 추종거리는 target/distance 를 기준값으로 쓴다 (FR-29). 좌표로 역산하지 않는다
  - 모든 시각은 ROS 시각 (kpi_logger 가 그렇게 기록한다)
"""
import csv
import sys
from collections import defaultdict

# 개입으로 세지 않는 kind (BRD 5.3, interfaces.md)
NOT_INTERVENTION = {'SAFETY_POLICY'}

# Mission.msg 상수
MISSION_STATE = {
    '0': 'CREATED', '1': 'ASSIGNED', '2': 'SHUTTLE', '3': 'AT_BAY', '4': 'WAITING',
    '5': 'PAIRED', '6': 'FOLLOWING', '7': 'STANDBY', '8': 'DONE', '9': 'FAILED',
}


def percentile(values, q):
    """q 분위 (0~100). numpy 없이 선형보간."""
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


def load(paths):
    rows = []
    for p in paths:
        with open(p, encoding='utf-8') as f:
            for r in csv.DictReader(f):
                try:
                    r['t'] = float(r['t_ros'])
                except (TypeError, ValueError):
                    continue
                rows.append(r)
    rows.sort(key=lambda r: r['t'])
    return rows


def follow_distance(rows):
    """추종거리 — target/distance 를 로봇별로 모은다 (FR-29)."""
    per = defaultdict(list)
    for r in rows:
        if r['source'] != 'distance':
            continue
        try:
            per[r['robot']].append(float(r['value']))
        except ValueError:
            continue
    return per


def interventions(rows):
    """개입 — SAFETY_POLICY 는 제외. (계상 대상, 제외분) 을 돌려준다."""
    counted, excluded = defaultdict(int), defaultdict(int)
    for r in rows:
        if r['source'] != 'intervention':
            continue
        (excluded if r['key'] in NOT_INTERVENTION else counted)[r['key']] += 1
    return counted, excluded


def missions(rows):
    """미션별 상태 전이 시각 → 리드타임·성공률."""
    per = defaultdict(dict)     # mission_id -> {state_name: t}
    for r in rows:
        if r['source'] != 'mission':
            continue
        name = MISSION_STATE.get(str(r['value']).strip(), str(r['value']))
        per[r['key']].setdefault(name, r['t'])
    return per


def lost_spans(rows):
    """approach/status 가 SEARCHING 으로 들어갔다 나온 구간 = Lost 재획득 시간."""
    spans, start = [], {}
    for r in rows:
        if r['source'] != 'status' or r['key'] != 'approach':
            continue
        ns, st = r['robot'], r['value']
        if st == 'SEARCHING':
            start.setdefault(ns, r['t'])
        elif ns in start:
            spans.append(r['t'] - start.pop(ns))
    return spans


def main():
    if len(sys.argv) < 2:
        sys.exit('사용법: kpi_report.py <kpi_*.csv> [...]')
    rows = load(sys.argv[1:])
    if not rows:
        sys.exit('행이 없습니다')

    span_s = rows[-1]['t'] - rows[0]['t']
    print(f'구간 {span_s / 60:.1f} 분 · {len(rows)} 행 · 파일 {len(sys.argv) - 1} 개\n')

    # 추종거리 (FR-29)
    print('추종거리 (target/distance, m)')
    per = follow_distance(rows)
    if not per:
        print('  기록 없음 — amr_detection 이 target/distance 를 발행했는지 확인')
    for ns, vals in sorted(per.items()):
        print(f'  {ns or "-":8s} n={len(vals):5d}  평균 {sum(vals)/len(vals):.2f}  '
              f'p50 {percentile(vals, 50):.2f}  p95 {percentile(vals, 95):.2f}  최대 {max(vals):.2f}')

    # IPH
    counted, excluded = interventions(rows)
    n = sum(counted.values())
    hours = span_s / 3600.0
    print('\n개입 (IPH)')
    print(f'  계상 {n} 건 / {hours:.2f} h → IPH {n / hours if hours else 0:.2f}')
    for k, v in sorted(counted.items()):
        print(f'    {k}: {v}')
    if excluded:
        print(f'  제외(개입 아님): {dict(excluded)}')

    # 리드타임·성공률
    per_m = missions(rows)
    lead = []
    done = failed = 0
    for mid, st in per_m.items():
        if 'DONE' in st:
            done += 1
            t0 = st.get('CREATED') or st.get('ASSIGNED')
            if t0:
                lead.append(st['DONE'] - t0)
        elif 'FAILED' in st:
            failed += 1
    total = len(per_m)
    print('\n미션')
    print(f'  전체 {total} · 완료 {done} · 실패 {failed}'
          + (f' → 성공률 {done / total * 100:.1f}%' if total else ''))
    if lead:
        print(f'  리드타임(요청→완료, s)  평균 {sum(lead)/len(lead):.1f}  '
              f'p50 {percentile(lead, 50):.1f}  p95 {percentile(lead, 95):.1f}')
    elif total:
        print('  리드타임: 완료된 미션 없음')

    # Lost 재획득
    spans = lost_spans(rows)
    print('\nLost 재획득 (approach SEARCHING 구간, s)')
    if spans:
        print(f'  n={len(spans)}  평균 {sum(spans)/len(spans):.1f}  '
              f'p50 {percentile(spans, 50):.1f}  p95 {percentile(spans, 95):.1f}  최대 {max(spans):.1f}')
    else:
        print('  기록 없음')


if __name__ == '__main__':
    main()
