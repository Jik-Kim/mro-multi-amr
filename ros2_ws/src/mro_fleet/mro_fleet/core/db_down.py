"""schema.sql의 세 테이블을 터미널에서 점검하는 SQLite CRUD 도구."""
import argparse
import logging
import sqlite3
from pathlib import Path


LOGGER = logging.getLogger('fleet_db_crud')
DEFAULT_SCHEMA_PATH = (
    Path(__file__).resolve().parents[2] / 'config' / 'schema.sql'
)


class FleetDBCrud:
    """workers, missions, events 테이블의 기본 CRUD를 제공한다."""

    def __init__(self, database_path, schema_path=DEFAULT_SCHEMA_PATH):
        """SQLite 파일을 열고 schema.sql의 테이블을 준비한다."""
        database_path = Path(database_path).expanduser()
        database_path.parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(database_path)
        self.connection.row_factory = sqlite3.Row
        with open(schema_path, encoding='utf-8') as schema_file:
            self.connection.executescript(schema_file.read())
        LOGGER.info('DB 연결: %s', database_path)

    def close(self):
        """SQLite 연결을 닫는다."""  # noqa: D403
        self.connection.close()
        LOGGER.info('DB 연결 종료')

    @staticmethod
    def _as_dict(row):
        """SQLite Row를 dict로 변환한다."""
        return dict(row) if row is not None else None

    # workers CRUD
    def create_worker(self, worker_id, outfit_class, registered_at):
        """작업자를 생성한다."""
        with self.connection:
            self.connection.execute(
                'INSERT INTO workers '
                '(worker_id, outfit_class, registered_at) VALUES (?, ?, ?)',
                (worker_id, outfit_class, registered_at),
            )
        return self.read_worker(worker_id)

    def read_worker(self, worker_id):
        """작업자 한 명을 조회한다."""
        row = self.connection.execute(
            'SELECT * FROM workers WHERE worker_id = ?', (worker_id,)
        ).fetchone()
        return self._as_dict(row)

    def read_workers(self):
        """작업자 전체를 조회한다."""
        rows = self.connection.execute(
            'SELECT * FROM workers ORDER BY worker_id'
        ).fetchall()
        return [dict(row) for row in rows]

    def update_worker(self, worker_id, outfit_class, registered_at):
        """작업자 복장 분류와 등록 시각을 수정한다."""
        with self.connection:
            cursor = self.connection.execute(
                'UPDATE workers SET outfit_class = ?, registered_at = ? '
                'WHERE worker_id = ?',
                (outfit_class, registered_at, worker_id),
            )
        return cursor.rowcount > 0

    def delete_worker(self, worker_id):
        """작업자를 삭제한다."""
        with self.connection:
            cursor = self.connection.execute(
                'DELETE FROM workers WHERE worker_id = ?', (worker_id,)
            )
        return cursor.rowcount > 0

    # missions CRUD
    def create_mission(
        self,
        mission_id,
        worker_id,
        bay,
        state,
        created_at,
        robot_ns=None,
        waiting_at=None,
        finished_at=None,
        result=None,
        note=None,
    ):
        """미션을 생성한다."""
        with self.connection:
            self.connection.execute(
                'INSERT INTO missions '
                '(mission_id, worker_id, robot_ns, bay, state, created_at, '
                'waiting_at, finished_at, result, note) '
                'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                (
                    mission_id, worker_id, robot_ns, bay, state, created_at,
                    waiting_at, finished_at, result, note,
                ),
            )
        return self.read_mission(mission_id)

    def read_mission(self, mission_id):
        """미션 한 건을 조회한다."""
        row = self.connection.execute(
            'SELECT * FROM missions WHERE mission_id = ?', (mission_id,)
        ).fetchone()
        return self._as_dict(row)

    def read_missions(self):
        """미션 전체를 조회한다."""
        rows = self.connection.execute(
            'SELECT * FROM missions ORDER BY created_at, mission_id'
        ).fetchall()
        return [dict(row) for row in rows]

    def update_mission(
        self,
        mission_id,
        state,
        robot_ns=None,
        waiting_at=None,
        finished_at=None,
        result=None,
        note=None,
    ):
        """미션의 실행 상태와 결과를 수정한다."""
        with self.connection:
            cursor = self.connection.execute(
                'UPDATE missions SET state = ?, robot_ns = ?, '
                'waiting_at = ?, finished_at = ?, result = ?, note = ? '
                'WHERE mission_id = ?',
                (
                    state, robot_ns, waiting_at, finished_at, result, note,
                    mission_id,
                ),
            )
        return cursor.rowcount > 0

    def delete_mission(self, mission_id):
        """미션을 삭제한다."""
        with self.connection:
            cursor = self.connection.execute(
                'DELETE FROM missions WHERE mission_id = ?', (mission_id,)
            )
        return cursor.rowcount > 0

    # events CRUD
    def create_event(self, event_at, robot_ns, kind, value='', note=''):
        """이벤트를 생성하고 자동 생성된 ID를 반환한다."""
        with self.connection:
            cursor = self.connection.execute(
                'INSERT INTO events (t, robot_ns, kind, value, note) '
                'VALUES (?, ?, ?, ?, ?)',
                (event_at, robot_ns, kind, value, note),
            )
        return cursor.lastrowid

    def read_event(self, event_id):
        """이벤트 한 건을 조회한다."""
        row = self.connection.execute(
            'SELECT * FROM events WHERE id = ?', (event_id,)
        ).fetchone()
        return self._as_dict(row)

    def read_events(self, limit=None):
        """이벤트를 최신 순서로 조회한다. limit이 없으면 전체를 반환한다."""
        if limit is None:
            rows = self.connection.execute(
                'SELECT * FROM events ORDER BY id DESC'
            ).fetchall()
        else:
            rows = self.connection.execute(
                'SELECT * FROM events ORDER BY id DESC LIMIT ?', (limit,)
            ).fetchall()
        return [dict(row) for row in rows]

    def update_event(self, event_id, event_at, robot_ns, kind, value, note):
        """이벤트의 모든 사용자 입력 필드를 수정한다."""
        with self.connection:
            cursor = self.connection.execute(
                'UPDATE events SET t = ?, robot_ns = ?, kind = ?, '
                'value = ?, note = ? WHERE id = ?',
                (event_at, robot_ns, kind, value, note, event_id),
            )
        return cursor.rowcount > 0

    def delete_event(self, event_id):
        """이벤트를 삭제한다."""
        with self.connection:
            cursor = self.connection.execute(
                'DELETE FROM events WHERE id = ?', (event_id,)
            )
        return cursor.rowcount > 0


def run_crud_demo(database):
    """세 테이블의 CRUD 결과를 INFO 로그로 보여준다."""
    LOGGER.info('========== workers CRUD ==========')
    LOGGER.info(
        'CREATE %s', database.create_worker('CRUD_W1', 'worker_a', 10.0)
    )
    LOGGER.info('READ   %s', database.read_worker('CRUD_W1'))
    database.update_worker('CRUD_W1', 'worker_b', 11.0)
    LOGGER.info('UPDATE %s', database.read_worker('CRUD_W1'))

    LOGGER.info('========== missions CRUD ==========')
    LOGGER.info(
        'CREATE %s',
        database.create_mission(
            'CRUD_M1', 'CRUD_W1', 'A', 0, 20.0, robot_ns='/robot3'
        ),
    )
    LOGGER.info('READ   %s', database.read_mission('CRUD_M1'))
    database.update_mission(
        'CRUD_M1', 4, '/robot3', 30.0, None, 'SUCCESS', '대기공간 도착'
    )
    LOGGER.info('UPDATE %s', database.read_mission('CRUD_M1'))

    LOGGER.info('========== events CRUD ==========')
    event_id = database.create_event(
        30.0, '/robot3', 'STATE', 'WAITING', 'CRUD 점검'
    )
    LOGGER.info('CREATE %s', database.read_event(event_id))
    LOGGER.info('READ   %s', database.read_event(event_id))
    database.update_event(
        event_id, 31.0, '/robot3', 'STATE', 'RETURNING', '수정 확인'
    )
    LOGGER.info('UPDATE %s', database.read_event(event_id))

    LOGGER.info('========== DELETE ==========')
    LOGGER.info('events 삭제: %s', database.delete_event(event_id))
    LOGGER.info('missions 삭제: %s', database.delete_mission('CRUD_M1'))
    LOGGER.info('workers 삭제: %s', database.delete_worker('CRUD_W1'))
    LOGGER.info('남은 workers: %s', database.read_workers())
    LOGGER.info('남은 missions: %s', database.read_missions())
    LOGGER.info('남은 events: %s', database.read_events())


def print_table(table_name, rows):
    """조회 결과를 열 너비가 맞는 터미널 표로 출력한다."""
    print(f'\n========== {table_name} ({len(rows)}건) ==========')
    if not rows:
        print('(데이터 없음)')
        return

    columns = list(rows[0])
    values = [
        ['NULL' if row[column] is None else str(row[column]) for column in columns]
        for row in rows
    ]
    widths = [
        max(len(column), *(len(row[index]) for row in values))
        for index, column in enumerate(columns)
    ]
    header = ' | '.join(
        column.ljust(widths[index])
        for index, column in enumerate(columns)
    )
    separator = '-+-'.join('-' * width for width in widths)
    print(header)
    print(separator)
    for row in values:
        print(' | '.join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        ))


def show_all_tables(database):
    """관제 DB의 모든 테이블을 터미널에 출력한다."""
    print_table('workers', database.read_workers())
    print_table('missions', database.read_missions())
    print_table('events', database.read_events())


def parse_arguments():
    """터미널 실행 인자를 읽는다."""
    parser = argparse.ArgumentParser(
        description='mro_fleet SQLite 테이블 CRUD 점검 도구'
    )
    parser.add_argument(
        '--db',
        default='~/mro_fleet.db',
        help='점검할 SQLite 파일 경로',
    )
    parser.add_argument(
        '--schema',
        default=str(DEFAULT_SCHEMA_PATH),
        help='schema.sql 경로',
    )
    parser.add_argument(
        'command',
        nargs='?',
        default='show',
        choices=('demo', 'show'),
        help='show(기본): 테이블 출력, demo: CRUD 전체 실행',
    )
    return parser.parse_args()


def main():
    """CRUD 데모 또는 전체 테이블 조회를 실행한다."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
    )
    arguments = parse_arguments()
    database_path = Path(arguments.db).expanduser()
    if arguments.command == 'show' and not database_path.is_file():
        LOGGER.error('DB 파일이 없습니다: %s', database_path)
        return

    database = FleetDBCrud(database_path, arguments.schema)
    try:
        if arguments.command == 'demo':
            run_crud_demo(database)
        else:
            show_all_tables(database)
    finally:
        database.close()


if __name__ == '__main__':
    main()
