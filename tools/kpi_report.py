#!/usr/bin/env python3
"""KPI CSV와 fleet DB에서 성공률·리드타임을 집계한다."""
import argparse
import csv
import os
import sqlite3


def percentile(values, percent):
    """선형 보간으로 백분위수를 계산한다."""
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def load_missions(db_path):
    """fleet DB에서 KPI 판정에 필요한 미션 원천값을 읽는다."""
    if not os.path.isfile(db_path):
        raise FileNotFoundError(f'fleet DB를 찾을 수 없습니다: {db_path}')
    connection = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    connection.row_factory = sqlite3.Row
    try:
        return connection.execute(
            'SELECT mission_id, worker_id, robot_ns, bay, created_at, '
            'waiting_at, result FROM missions ORDER BY created_at'
        ).fetchall()
    finally:
        connection.close()


def delivery_metrics(missions):
    """DB result로 AC-08, waiting_at-created_at으로 K-05를 계산한다."""
    finalized = [mission for mission in missions if mission['result']]
    successes = [mission for mission in finalized if mission['result'] == 'SUCCESS']
    lead_times = [mission['waiting_at'] - mission['created_at'] for mission in missions if mission['waiting_at'] is not None]
    return {
        'total': len(missions),
        'finalized': len(finalized),
        'pending': len(missions) - len(finalized),
        'successes': len(successes),
        'success_rate': (100.0 * len(successes) / len(finalized)) if finalized else None,
        'lead_times': lead_times,
    }


def print_delivery_report(metrics):
    """배송 KPI를 터미널에 출력한다."""
    rate = '계산 불가' if metrics['success_rate'] is None else f"{metrics['success_rate']:.1f}%"
    print('\n[배송 KPI — fleet DB]')
    print(f"미션: 전체 {metrics['total']}건 / 판정 {metrics['finalized']}건 / 미판정 {metrics['pending']}건")
    print(f"AC-08 배송 성공률: {rate} ({metrics['successes']}/{metrics['finalized']})")
    if metrics['lead_times']:
        print(f"K-05 리드타임: 평균 {sum(metrics['lead_times']) / len(metrics['lead_times']):.2f}s / p95 {percentile(metrics['lead_times'], 95):.2f}s")
    else:
        print('K-05 리드타임: 대기공간 도착 기록 없음')
    if metrics['pending']:
        print('경고: result가 NULL인 미판정 미션은 성공률 분모에서 제외됐습니다.')


def parse_args():
    parser = argparse.ArgumentParser(description='MRO KPI CSV/DB 리포트')
    parser.add_argument('csv_path', help='kpi_logger가 생성한 CSV 경로')
    parser.add_argument('--db', default=os.path.expanduser('~/mro_fleet.db'), help='fleet SQLite DB 경로')
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.csv_path, encoding='utf-8') as csv_file:
        rows = list(csv.DictReader(csv_file))
    print(f'KPI CSV: {len(rows)} rows')
    db_path = os.path.abspath(os.path.expanduser(args.db))
    print_delivery_report(delivery_metrics(load_missions(db_path)))


if __name__ == '__main__':
    main()
