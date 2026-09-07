#!/usr/bin/env python3
"""kpi_logger CSV(+ 관제 DB) → AC-05 / AC-08 / AC-13 / K-04 / K-05 요약.

  python3 tools/kpi_report.py ~/mro_kpi/kpi_*.csv --db ~/mro_fleet.db

정의는 docs/interfaces.md 9절 「KPI 산출 계약」과 BRD v3.1 5.3 을 따른다.
계약과 어긋나면 계약이 우선한다.

  AC-05 추종거리  FOLLOWING 구간에서 |distance - standoff| 의 p95 <= 0.2 m,
                  최소 이격 >= 0.3 m. 원값이 아니라 오차다
  AC-08 배송      result='SUCCESS' / 전체 미션 >= 95 %.
                  CSV 로는 낼 수 없어 관제 DB(missions)를 읽는다 (I-023)
  AC-13 IPH       개입 / 운용시간 <= 1.0. 운용시간은 undock~DOCKED 의 합이지
                  로그 전체 구간이 아니다. SAFETY_POLICY 는 개입이 아니다
  K-04  재획득    Lost 는 target/visible 1->0 (LOST_MIN_S 이상 지속).
                  재획득은 0->1. STANDBY/FAILED 로 끝난 건은 실패로 센다.
                  approach/status 의 SEARCHING 은 정상 절차에서도 쓰이므로 Lost 신호가 아니다
  K-05  리드타임  waiting_at - created_at (요청 -> 대기공간 도착). DONE 은 도킹 완료라 다르다

kpi_logger 는 거르지 않고 받아적는다. 구간을 자르고 세는 것은 여기서만 한다.
"""
import argparse
import csv
import os
import sqlite3
from collections import defaultdict

NOT_INTERVENTION = {'SAFETY_POLICY'}          # 개입으로 세지 않는다 (BRD 5.3)
LOST_MIN_S = 1.0                              # 이 시간 이상 안 보여야 Lost (SOT Lost 1.0 s)
FAIL_STATES = {'STANDBY', 'FAILED'}           # 재획득 실패로 끝난 상태
DOCKED_STATES = {'DOCKED', 'READY'}           # 운용시간에서 제외 (undock 전 / 도킹 완료 후)


def percentile(values, q):
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


def spans_of(rows, source, key, wanted, t_end):
    """(robot -> [(시작, 끝), ...]) — 상태값이 wanted 집합에 든 구간."""
    out, open_at = defaultdict(list), {}
    for r in rows:
        if r['source'] != source or r['key'] != key:
            continue
        ns, st = r['robot'], r['value']
        if st in wanted:
            open_at.setdefault(ns, r['t'])
        elif ns in open_at:
            out[ns].append((open_at.pop(ns), r['t']))
    for ns, t0 in open_at.items():           # 로그 끝까지 열려 있던 구간
        out[ns].append((t0, t_end))
    return out


def in_spans(t, spans):
    return any(a <= t <= b for a, b in spans)


# ── AC-05 추종거리 ────────────────────────────────────────────────
def follow_error(rows, standoff, t_end):
    """FOLLOWING 구간의 |distance - standoff| 와 최소 이격."""
    following = spans_of(rows, 'status', 'approach', {'FOLLOWING'}, t_end)
    err, raw = defaultdict(list), defaultdict(list)
    for r in rows:
        if r['source'] != 'distance':
            continue
        ns = r['robot']
        if not in_spans(r['t'], following.get(ns, [])):
            continue                          # 접근 단계·STANDBY 값은 섞지 않는다
        try:
            d = float(r['value'])
        except ValueError:
            continue
        raw[ns].append(d)
        err[ns].append(abs(d - standoff))
    return err, raw, following


# ── AC-13 개입 ────────────────────────────────────────────────────
def interventions(rows):
    counted, excluded = defaultdict(int), defaultdict(int)
    for r in rows:
        if r['source'] != 'intervention':
            continue
        (excluded if r['key'] in NOT_INTERVENTION else counted)[r['key']] += 1
    return counted, excluded


def operating_seconds(rows, t_end):
    """운용시간 = undock ~ DOCKED 의 합. 미션 사이 대기는 빼야 IPH 가 부풀지 않는다."""
    total, open_at = 0.0, {}
    seen = False
    for r in rows:
        if r['source'] != 'status' or r['key'] != 'shuttle':
            continue
        seen = True
        ns, st = r['robot'], r['value']
        if st in DOCKED_STATES:
            if ns in open_at:
                total += r['t'] - open_at.pop(ns)
        else:
            open_at.setdefault(ns, r['t'])
    for ns, t0 in open_at.items():
        total += t_end - t0
    return (total if seen else None)


# ── K-04 Lost / 재획득 ────────────────────────────────────────────
def reacquisition(rows, t_end):
    """target/visible 1->0 (LOST_MIN_S 이상) 을 Lost 로, 0->1 을 재획득으로 센다."""
    vis = defaultdict(list)
    for r in rows:
        if r['source'] == 'visible':
            try:
                vis[r['robot']].append((r['t'], int(r['value'])))
            except ValueError:
                pass
    appr = [r for r in rows if r['source'] == 'status' and r['key'] == 'approach']

    ok_spans, failed = [], 0
    for ns, seq in vis.items():
        lost_at = None
        for t, v in seq:
            if v == 0 and lost_at is None:
                lost_at = t
            elif v == 1 and lost_at is not None:
                dur = t - lost_at
                if dur >= LOST_MIN_S:
                    ok_spans.append(dur)
                lost_at = None                # 짧은 끊김은 Lost 로 세지 않는다
        if lost_at is not None and t_end - lost_at >= LOST_MIN_S:
            failed += 1                       # 끝까지 못 찾음
            lost_at_final = lost_at
        else:
            lost_at_final = None
        # 재획득 구간 안에서 STANDBY/FAILED 로 빠진 건은 성공이 아니다
        for a in appr:
            if a['robot'] != ns or a['value'] not in FAIL_STATES:
                continue
            if lost_at_final is not None and a['t'] >= lost_at_final:
                continue                      # 위에서 이미 실패로 셈
    return ok_spans, failed


def reacq_failures_by_state(rows, t_end):
    """Lost 구간 중 STANDBY/FAILED 로 끝난 건 수."""
    vis = defaultdict(list)
    for r in rows:
        if r['source'] == 'visible':
            try:
                vis[r['robot']].append((r['t'], int(r['value'])))
            except ValueError:
                pass
    appr = defaultdict(list)
    for r in rows:
        if r['source'] == 'status' and r['key'] == 'approach':
            appr[r['robot']].append((r['t'], r['value']))
    fails = 0
    for ns, seq in vis.items():
        lost_at = None
        for t, v in seq:
            if v == 0 and lost_at is None:
                lost_at = t
            elif v == 1 and lost_at is not None:
                if any(lost_at <= at <= t and st in FAIL_STATES for at, st in appr.get(ns, [])):
                    fails += 1
                lost_at = None
    return fails


# ── AC-08 / K-05 : 관제 DB ────────────────────────────────────────
def missions_from_db(db_path):
    if not db_path or not os.path.exists(db_path):
        return None
    con = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    try:
        return con.execute(
            'SELECT mission_id, result, created_at, waiting_at, finished_at FROM missions'
        ).fetchall()
    finally:
        con.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--db', default=os.path.expanduser('~/mro_fleet.db'),
                    help='관제 DB. AC-08·K-05 는 CSV 로 낼 수 없어 여기서 읽는다 (I-023)')
    ap.add_argument('--standoff', type=float, default=0.5, help='추종거리 기준 (m)')
    a = ap.parse_args()

    rows = load(a.csv)
    if not rows:
        raise SystemExit('행이 없습니다')
    t_end = rows[-1]['t']
    print(f'로그 {(t_end - rows[0]["t"]) / 60:.1f} 분 · {len(rows)} 행 · 파일 {len(a.csv)} 개\n')

    # AC-05
    err, raw, following = follow_error(rows, a.standoff, t_end)
    print(f'AC-05 추종거리 — FOLLOWING 구간, standoff {a.standoff} m 기준')
    if not err:
        n_span = sum(len(v) for v in following.values())
        print(f'  기록 없음 (FOLLOWING 구간 {n_span} 개) — target/distance 발행 여부 확인')
    for ns in sorted(err):
        e, d = err[ns], raw[ns]
        p95 = percentile(e, 95)
        print(f'  {ns or "-":8s} n={len(e):5d}  오차 p95 {p95:.3f} m '
              f'[{"PASS" if p95 <= 0.2 else "FAIL"} ≤0.2]  '
              f'최소 이격 {min(d):.3f} m [{"PASS" if min(d) >= 0.3 else "FAIL"} ≥0.3]')

    # AC-13
    counted, excluded = interventions(rows)
    n = sum(counted.values())
    op_s = operating_seconds(rows, t_end)
    print('\nAC-13 개입 (IPH) — 운용시간 = undock~DOCKED 합')
    if op_s is None:
        print(f'  개입 {n} 건 · 운용시간 산출 불가 (shuttle/status 기록 없음 — I-024)')
    elif op_s <= 0:
        print(f'  개입 {n} 건 · 운용시간 0 s')
    else:
        iph = n / (op_s / 3600.0)
        print(f'  개입 {n} 건 / 운용 {op_s / 60:.1f} 분 → IPH {iph:.2f} '
              f'[{"PASS" if iph <= 1.0 else "FAIL"} ≤1.0]')
    for k, v in sorted(counted.items()):
        print(f'    {k}: {v}')
    if excluded:
        print(f'  제외(개입 아님): {dict(excluded)}')

    # AC-08 / K-05
    print('\nAC-08 배송 성공률 · K-05 리드타임 — 관제 DB')
    ms = missions_from_db(a.db)
    if ms is None:
        print(f'  DB 없음: {a.db}')
        print('  ※ CSV 로는 낼 수 없습니다. Mission.msg 에 result 가 없어 판정이 로그에 남지 않고,')
        print('    끝내 도착 못 한 미션은 CSV 에 흔적이 없어 분모에서 빠집니다 (I-023)')
    else:
        total = len(ms)
        succ = sum(1 for m in ms if m[1] == 'SUCCESS')
        pending = sum(1 for m in ms if m[1] is None)
        lead = [m[3] - m[2] for m in ms if m[3] is not None and m[2] is not None]
        if total:
            rate = succ / total * 100
            print(f'  전체 {total} · SUCCESS {succ} · 미판정 {pending} '
                  f'→ {rate:.1f}% [{"PASS" if rate >= 95 else "FAIL"} ≥95]')
            print('  ※ 미판정(result IS NULL)도 분모에 남깁니다 — 빼면 못 간 미션일수록 성공률이 올라갑니다')
        by = defaultdict(int)
        for m in ms:
            by[m[1] or '(미판정)'] += 1
        for k, v in sorted(by.items()):
            print(f'    {k}: {v}')
        if lead:
            print(f'  리드타임(요청→대기공간 도착, s)  평균 {sum(lead)/len(lead):.1f}  '
                  f'p50 {percentile(lead, 50):.1f}  p95 {percentile(lead, 95):.1f}')
        else:
            print('  리드타임: waiting_at 이 기록된 미션 없음')

    # K-04
    ok, unfinished = reacquisition(rows, t_end)
    by_state = reacq_failures_by_state(rows, t_end)
    fails = unfinished + by_state
    print('\nK-04 Lost 재획득 — visible 1→0 (≥%.1f s) 기준' % LOST_MIN_S)
    tot = len(ok) + fails
    if tot == 0:
        print('  기록 없음 — target/visible 발행 여부 확인')
    else:
        rate = len(ok) / tot * 100
        print(f'  Lost {tot} 회 · 재획득 {len(ok)} · 실패 {fails}'
              f'(STANDBY/FAILED {by_state}, 미복귀 {unfinished}) '
              f'→ 성공률 {rate:.1f}% [{"PASS" if rate >= 90 else "FAIL"} ≥90]')
        if ok:
            print(f'  재획득 시간(s)  평균 {sum(ok)/len(ok):.1f}  p50 {percentile(ok, 50):.1f}  '
                  f'p95 {percentile(ok, 95):.1f} [{"PASS" if percentile(ok, 95) <= 5 else "FAIL"} ≤5]')


if __name__ == '__main__':
    main()
