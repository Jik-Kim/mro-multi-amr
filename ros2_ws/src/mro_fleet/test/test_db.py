"""관제 DB의 시각 기록과 T_lim 판정 단위 테스트."""
from pathlib import Path

from mro_fleet.core.db import FleetDB


SCHEMA = Path(__file__).parents[1] / 'config' / 'schema.sql'


def make_db(tmp_path):
    """테스트마다 독립된 SQLite DB를 만든다."""
    return FleetDB(str(tmp_path / 'fleet.db'), str(SCHEMA))


def test_worker_mission_and_event_are_stored(tmp_path):
    """작업자·미션·이벤트가 지정한 시각으로 저장되는지 확인한다."""
    db = make_db(tmp_path)
    try:
        db.upsert_worker('W1', 'worker_a', 10.0, '/robot3')
        mission_id = db.create_mission('W1', 'B', 20.0, '/robot3')
        db.set_state(mission_id, 1, 20.0)
        db.event(21.0, '/robot3', 'STATE', 'SHUTTLE')

        mission = db.mission(mission_id)
        assert db.robot_for('W1') == '/robot3'
        assert mission['created_at'] == 20.0
        assert mission['state'] == 1
        assert db.recent_events(1)[0]['t'] == 21.0
    finally:
        db.close()


def test_waiting_within_limit_is_success(tmp_path):
    """T_lim 이내 도착이 성공으로 판정되는지 확인한다."""
    db = make_db(tmp_path)
    try:
        mission_id = db.create_mission('W1', 'A', 100.0, '/robot3')
        arrival = db.mark_waiting(mission_id, 219.0, 120.0)

        assert arrival['first_arrival'] is True
        assert arrival['lead_time_s'] == 119.0
        assert arrival['result'] == 'SUCCESS'
        assert arrival['arrived_late'] is False
    finally:
        db.close()


def test_waiting_at_exact_limit_is_success(tmp_path):
    """T_lim과 정확히 같은 도착 시간은 성공인지 확인한다."""
    db = make_db(tmp_path)
    try:
        mission_id = db.create_mission('W1', 'A', 100.0, '/robot3')
        arrival = db.mark_waiting(mission_id, 220.0, 120.0)

        assert arrival['result'] == 'SUCCESS'
        assert arrival['arrived_late'] is False
    finally:
        db.close()


def test_late_arrival_is_not_counted_twice(tmp_path):
    """지연 도착 결과와 최초 도착 시각이 다시 쓰이지 않음을 확인한다."""
    db = make_db(tmp_path)
    try:
        mission_id = db.create_mission('W1', 'C', 100.0, '/robot3')
        first = db.mark_waiting(mission_id, 221.0, 120.0)
        repeated = db.mark_waiting(mission_id, 250.0, 120.0)
        mission = db.mission(mission_id)

        assert first['first_arrival'] is True
        assert first['result'] == 'FAIL_TLIM'
        assert repeated['first_arrival'] is False
        assert repeated['lead_time_s'] == 121.0
        assert mission['waiting_at'] == 221.0
        assert mission['result'] == 'FAIL_TLIM'
    finally:
        db.close()


def test_done_keeps_delivery_result(tmp_path):
    """도킹 완료 뒤에도 앞서 확정한 배송 결과가 유지되는지 확인한다."""
    db = make_db(tmp_path)
    try:
        mission_id = db.create_mission('W2', 'B', 100.0, '/robot8')
        db.mark_waiting(mission_id, 225.0, 120.0)
        db.set_state(mission_id, 8, 300.0)
        mission = db.mission(mission_id)

        assert mission['state'] == 8
        assert mission['finished_at'] == 300.0
        assert mission['result'] == 'FAIL_TLIM'
    finally:
        db.close()


def test_overdue_without_arrival_is_failed_once(tmp_path):
    """미도착 미션도 T_lim 초과 뒤 한 번만 실패로 확정되는지 확인한다."""
    db = make_db(tmp_path)
    try:
        mission_id = db.create_mission('W1', 'A', 100.0, '/robot3')

        at_limit = db.expire_overdue(220.0, 120.0)
        overdue = db.expire_overdue(220.001, 120.0)
        repeated = db.expire_overdue(300.0, 120.0)
        mission = db.mission(mission_id)

        assert at_limit == []
        assert [row['mission_id'] for row in overdue] == [mission_id]
        assert repeated == []
        assert mission['result'] == 'FAIL_TLIM'
        assert mission['state'] == 0
        assert mission['waiting_at'] is None
    finally:
        db.close()
