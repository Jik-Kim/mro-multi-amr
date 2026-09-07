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
                  재획득은 0->1. 단 그 구간에 STANDBY/FAILED 가 있었으면 실패로만 센다 (I-028).
                  approach/status 의 SEARCHING 은 정상 절차에서도 쓰이므로 Lost 신호가 아니다
  K-05  리드타임  waiting_at - created_at (요청 -> 대기공간 도착). DONE 은 도킹 완료라 다르다
  K-06  교착해결  /fleet/zone_event 의 REJECT -> 같은 로봇·구역 GRANT 가 60 s 안.
                  단 그 사이 그 구역에 RECLAIM 이 있으면 미해결 — RECLAIM 은 보유자 ns 로
                  오므로 구역 단위로 본다 (I-032). 개입이 아니라 IPH 에 넣지 않는다 (v1.11)

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
ZONE_RESOLVE_S = 60.0                         # REJECT 후 이 시간 안에 GRANT 면 자동 해결 (계약 3절)


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
    """Lost 1건을 성공·실패 어느 한쪽으로만 센다 (I-028).

    Lost      target/visible 1->0 이 LOST_MIN_S 이상 지속된 것. 짧은 끊김은 세지 않는다
    실패      그 구간 안에 approach/status 가 STANDBY/FAILED 였다 — 로봇이 탐색을 포기했다.
              그 뒤 대상이 다시 보여도 로봇이 재획득한 것이 아니다
    성공      포기 없이 0->1 로 돌아온 것. 걸린 시간을 p95 에 넣는다
    미복귀    로그 끝까지 0 인 채로 끝난 것

    종전에는 성공을 reacquisition() 이, 실패를 reacq_failures_by_state() 가 각자
    로그를 훑어 같은 Lost 가 양쪽에 들어갔다. 두 함수를 하나로 합쳐 구간을 닫는
    그 자리에서 판정한다. 실패 쪽에 LOST_MIN_S 필터가 없던 것도 함께 고쳤다.
    """
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

    ok_spans, gave_up, unfinished = [], 0, 0
    for ns, seq in vis.items():
        states = appr.get(ns, [])
        lost_at = None
        for t, v in seq:
            if v == 0 and lost_at is None:
                lost_at = t
            elif v == 1 and lost_at is not None:
                dur = t - lost_at
                if dur >= LOST_MIN_S:
                    if any(lost_at <= at <= t and st in FAIL_STATES for at, st in states):
                        gave_up += 1
                    else:
                        ok_spans.append(dur)
                lost_at = None                # 짧은 끊김은 Lost 로 세지 않는다
        if lost_at is not None and t_end - lost_at >= LOST_MIN_S:
            unfinished += 1
    return ok_spans, gave_up, unfinished


# ── K-06 : 교착 자동 해결 ─────────────────────────────────────────
def zone_resolution(rows, t_end):
    """대기(REJECT) 가 사람 손 없이 풀렸는지 센다 — K-06 (계약 v1.11, I-030).

    세마포어가 /fleet/zone_event 로 "<zone> <robot> <REJECT|GRANT|RELEASE|RECLAIM>" 을 낸다.
    계약 3절이 REJECT 는 대기 시작 1건만, GRANT 는 새 획득만 내도록 정했으므로
    한 REJECT 와 그 뒤의 GRANT 하나가 짝이 된다.

      해결   같은 로봇·구역의 GRANT 가 ZONE_RESOLVE_S 안에 왔고,
             그 사이 그 구역에 RECLAIM 이 없었다
      미해결 [REJECT, GRANT) 구간에 그 구역의 RECLAIM 이 있었다 · 시간 초과 ·
             로그 끝까지 GRANT 가 안 옴

    RECLAIM 은 구역 단위로 본다 (I-032). 세마포어가 RECLAIM 을 **보유자** ns 로 내기
    때문이다 (zone_semaphore_node.py: self.event(z, h, 'RECLAIM'), h = holder).
    대기자 열만 보면 RECLAIM 이 절대 오지 않아, 강제로 뺏어서 넘어간 것을
    '자동 해결' 로 세고 강제 회수 카운트도 항상 0 이 된다.

    RELEASE 는 보유자의 정상 반납이라 대기와 짝짓지 않는다.
    이것은 개입이 아니다 — Intervention 과 섞으면 IPH 분자가 부풀어 AC-13 이 나빠 보인다.
    """
    per_robot = defaultdict(list)             # (robot, zone) -> [(t, action)]  대기 짝짓기용
    reclaims = defaultdict(list)              # zone -> [t]                      구역 단위
    for r in rows:
        if r['source'] != 'zone':
            continue
        if r['value'] == 'RECLAIM':
            reclaims[r['key']].append(r['t'])
        else:
            per_robot[(r['robot'], r['key'])].append((r['t'], r['value']))
    for ts in reclaims.values():
        ts.sort()

    def reclaimed_between(zone, t0, t1):
        """[t0, t1] 에 그 구역의 강제 회수가 있었나.

        끝점을 포함한다. 세마포어는 보유자에게서 뺏은 뒤 곧바로 대기자에게 넘기므로
        RECLAIM 과 GRANT 의 시각이 같게 찍힌다 (PM 재현: 둘 다 t=61). 반열린 구간으로
        두면 그 RECLAIM 이 빠져 '강제로 뺏어서 넘어간 것' 이 자동 해결로 잡힌다.
        """
        return any(t0 <= t <= t1 for t in reclaims.get(zone, ()))

    waits, resolved, spans = 0, 0, []
    for (_robot, zone), seq in per_robot.items():
        seq.sort(key=lambda x: x[0])
        rejected_at = None
        for t, action in seq:
            if action == 'REJECT':
                if rejected_at is None:       # 계약상 대기 시작 1건만 오지만 방어해 둔다
                    rejected_at = t
                    waits += 1
            elif rejected_at is None:
                continue                      # 대기 없이 온 GRANT/RELEASE 는 K-06 대상이 아니다
            elif action == 'GRANT':
                dur = t - rejected_at
                if dur <= ZONE_RESOLVE_S and not reclaimed_between(zone, rejected_at, t):
                    resolved += 1
                    spans.append(dur)
                rejected_at = None            # 시간 초과·강제 회수면 미해결로 두고 대기를 닫는다
        if rejected_at is not None and reclaimed_between(zone, rejected_at, t_end):
            pass                              # 끝까지 GRANT 가 안 온 대기 — 이미 미해결이다

    reclaimed = sum(len(ts) for ts in reclaims.values())
    return waits, resolved, reclaimed, spans


# ── AC-08 / K-05 : 관제 DB ────────────────────────────────────────
def missions_from_db(db_path):
    """(행, 안내문) 을 돌려준다. 행이 None 이면 안내문에 못 읽은 이유가 담긴다.

    관제 DB 가 아직 초기화되기 전이면 파일만 있고 missions 테이블이 없다. 그대로 두면
    OperationalError 로 리포트 전체가 죽어 나머지 네 지표까지 못 본다 — 잡아서 안내한다.
    """
    if not db_path or not os.path.exists(db_path):
        return None, f'DB 없음: {db_path}'
    try:
        con = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
        try:
            rows = con.execute(
                'SELECT mission_id, result, created_at, waiting_at, finished_at FROM missions'
            ).fetchall()
        finally:
            con.close()
    except sqlite3.Error as e:
        return None, (f'DB 를 읽지 못했습니다: {db_path}\n'
                      f'    {e}\n'
                      '    관제 DB 가 초기화됐는지 확인하세요 (fleet_manager 가 schema.sql 로 만듭니다)')
    return rows, None


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
    ms, why = missions_from_db(a.db)
    if ms is None:
        print(f'  {why}')
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
    ok, gave_up, unfinished = reacquisition(rows, t_end)
    fails = gave_up + unfinished
    print('\nK-04 Lost 재획득 — visible 1→0 (≥%.1f s) 기준' % LOST_MIN_S)
    tot = len(ok) + fails
    if tot == 0:
        print('  기록 없음 — target/visible 발행 여부 확인')
    else:
        rate = len(ok) / tot * 100
        print(f'  Lost {tot} 회 · 재획득 {len(ok)} · 실패 {fails}'
              f'(포기 {gave_up}, 미복귀 {unfinished}) '
              f'→ 성공률 {rate:.1f}% [{"PASS" if rate >= 90 else "FAIL"} ≥90]')
        if ok:
            print(f'  재획득 시간(s)  평균 {sum(ok)/len(ok):.1f}  p50 {percentile(ok, 50):.1f}  '
                  f'p95 {percentile(ok, 95):.1f} [{"PASS" if percentile(ok, 95) <= 5 else "FAIL"} ≤5]')

    # K-06
    waits, resolved, reclaimed, spans = zone_resolution(rows, t_end)
    print('\nK-06 교착 자동 해결 — /fleet/zone_event 기준')
    if waits == 0:
        print('  대기 기록 없음 — zone_semaphore 가 /fleet/zone_event 를 발행하는지 확인 (I-030)')
    else:
        rate = resolved / waits * 100
        print(f'  대기(REJECT) {waits} 회 · 자동 해결 {resolved} · 강제 회수 {reclaimed}'
              f' → {rate:.1f}%')
        if spans:
            print(f'  해결까지(s)  평균 {sum(spans)/len(spans):.1f}  p50 {percentile(spans, 50):.1f}  '
                  f'p95 {percentile(spans, 95):.1f}   [{ZONE_RESOLVE_S:.0f} s 안이면 해결]')
        print('  ※ 자동 해결은 개입이 아닙니다 — IPH(AC-13) 분자에 넣지 않습니다 (계약 v1.11)')


if __name__ == '__main__':
    main()
