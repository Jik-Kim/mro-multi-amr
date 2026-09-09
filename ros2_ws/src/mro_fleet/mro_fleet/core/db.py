"""SQLite 접근 계층. ROS 비의존. 스키마는 config/schema.sql."""
import os
import sqlite3
import threading
import uuid


class FleetDB:
    """스레드 안전한 관제 SQLite 접근 계층."""

    def __init__(self, path, schema_sql):
        """DB 파일을 열고 필요한 테이블을 생성한다."""
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.con = sqlite3.connect(path, check_same_thread=False)
        self.con.row_factory = sqlite3.Row
        self._lock = threading.Lock()
        self.con.executescript(open(schema_sql, encoding='utf-8').read())

    def close(self):
        """SQLite DB 연결을 닫는다."""  # noqa: D403
        with self._lock:
            self.con.close()

    def upsert_worker(
        self,
        worker_id,
        outfit_class,
        registered_at,
    ):
        """작업자 등록 정보를 저장한다."""
        with self._lock, self.con:
            self.con.execute(
                'INSERT OR REPLACE INTO workers '
                '(worker_id, outfit_class, registered_at) VALUES (?,?,?)',
                (worker_id, outfit_class, registered_at),
            )

    def create_mission(
        self, worker_id, bay, created_at, robot_ns=None
    ):
        """CREATED 상태의 미션을 만들고 ID를 반환한다."""  # noqa: D401
        mid = f'M{int(created_at)}-{uuid.uuid4().hex[:4]}'
        with self._lock, self.con:
            self.con.execute(
                'INSERT INTO missions '
                '(mission_id, worker_id, robot_ns, bay, state, created_at) '
                'VALUES (?,?,?,?,0,?)',
                (mid, worker_id, robot_ns, bay, created_at),
            )
        return mid

    def set_state(self, mission_id, state, changed_at, note=None, result=None):
        """미션 상태를 갱신한다. DONE/FAILED 시각은 최초 값만 보존한다."""
        with self._lock, self.con:
            self.con.execute(
                'UPDATE missions SET state=?, note=COALESCE(?, note), '
                'result=COALESCE(result, ?) WHERE mission_id=?',
                (state, note, result, mission_id),
            )
            if state in (8, 9):
                self.con.execute(
                    'UPDATE missions SET finished_at=COALESCE(finished_at, ?) '
                    'WHERE mission_id=?',
                    (changed_at, mission_id),
                )

    def mark_waiting(self, mission_id, waiting_at, t_lim_s):
        """최초 대기공간 도착을 기록하고 T_lim 결과를 한 번만 확정한다."""
        with self._lock, self.con:
            row = self.con.execute(
                'SELECT created_at, waiting_at, result FROM missions '
                'WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            if row is None:
                return None

            first_arrival = row['waiting_at'] is None
            if first_arrival:
                effective_waiting_at = waiting_at
            else:
                effective_waiting_at = row['waiting_at']
            lead_time_s = max(0.0, effective_waiting_at - row['created_at'])
            result = row['result']
            if first_arrival and result is None:
                result = 'FAIL_TLIM' if lead_time_s > t_lim_s else 'SUCCESS'

            self.con.execute(
                'UPDATE missions SET state=4, '
                'waiting_at=COALESCE(waiting_at, ?), '
                'result=COALESCE(result, ?) WHERE mission_id=?',
                (waiting_at, result, mission_id),
            )
            return {
                'first_arrival': first_arrival,
                'lead_time_s': lead_time_s,
                'result': result,
                'arrived_late': result == 'FAIL_TLIM',
            }

    def expire_overdue(self, now, t_lim_s):
        """미도착 상태로 T_lim을 넘긴 미션을 실패로 한 번만 확정한다."""
        with self._lock, self.con:
            rows = self.con.execute(
                'SELECT mission_id, robot_ns, created_at FROM missions '
                'WHERE result IS NULL AND created_at + ? < ?',
                (t_lim_s, now),
            ).fetchall()
            self.con.executemany(
                "UPDATE missions SET result='FAIL_TLIM' "
                'WHERE mission_id=? AND result IS NULL',
                ((row['mission_id'],) for row in rows),
            )
            return [dict(row) for row in rows]

    def event(self, event_at, robot_ns, kind, value='', note=''):
        """ROS 시각이 포함된 관제 이벤트를 저장한다."""
        with self._lock, self.con:
            self.con.execute(
                'INSERT INTO events (t, robot_ns, kind, value, note) '
                'VALUES (?,?,?,?,?)',
                (event_at, robot_ns, kind, str(value), note),
            )

    def mission(self, mission_id):
        """미션 한 건을 dict로 반환한다."""
        with self._lock:
            row = self.con.execute(
                'SELECT * FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            return dict(row) if row else None

    def recent_events(self, limit=50):
        """최신 이벤트를 최근 순서로 반환한다."""
        with self._lock:
            rows = self.con.execute(
                'SELECT * FROM events ORDER BY id DESC LIMIT ?',
                (limit,),
            ).fetchall()
            return [dict(row) for row in rows]
