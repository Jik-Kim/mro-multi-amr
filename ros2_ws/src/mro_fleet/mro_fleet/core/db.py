"""SQLite 접근 계층. ROS 비의존. 스키마는 config/schema.sql."""
import os
import sqlite3
import time
import uuid


class FleetDB:
    def __init__(self, path, schema_sql):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.con = sqlite3.connect(path, check_same_thread=False)
        self.con.executescript(open(schema_sql, encoding='utf-8').read())

    def upsert_worker(self, worker_id, outfit_class, robot_ns=None):
        self.con.execute('INSERT OR REPLACE INTO workers VALUES (?,?,?,?)', (worker_id, outfit_class, robot_ns, time.time()))
        self.con.commit()

    def create_mission(self, worker_id, bay, robot_ns=None):
        mid = f'M{int(time.time())}-{uuid.uuid4().hex[:4]}'
        self.con.execute('INSERT INTO missions (mission_id, worker_id, robot_ns, bay, state, created_at) VALUES (?,?,?,?,0,?)',
                         (mid, worker_id, robot_ns, bay, time.time()))
        self.con.commit()
        return mid

    def set_state(self, mission_id, state, note=None):
        self.con.execute('UPDATE missions SET state=?, note=COALESCE(?, note) WHERE mission_id=?', (state, note, mission_id))
        if state == 4:   # WAITING
            self.con.execute('UPDATE missions SET waiting_at=? WHERE mission_id=? AND waiting_at IS NULL', (time.time(), mission_id))
        if state in (8, 9):
            self.con.execute('UPDATE missions SET finished_at=? WHERE mission_id=?', (time.time(), mission_id))
        self.con.commit()

    def event(self, robot_ns, kind, value='', note=''):
        self.con.execute('INSERT INTO events (t, robot_ns, kind, value, note) VALUES (?,?,?,?,?)', (time.time(), robot_ns, kind, str(value), note))
        self.con.commit()

    def robot_for(self, worker_id):
        r = self.con.execute('SELECT robot_ns FROM workers WHERE worker_id=?', (worker_id,)).fetchone()
        return r[0] if r else None
