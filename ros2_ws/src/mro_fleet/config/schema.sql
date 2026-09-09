-- 관제 DB (SQLite). day5 7_create_DB_TBL.py 방식. BRD v3.0 S1·S3·S9, BR-T13, BR-T26
CREATE TABLE IF NOT EXISTS workers (
  worker_id     TEXT PRIMARY KEY,      -- W1, W2
  outfit_class  TEXT NOT NULL,         -- worker_a | worker_b
  registered_at REAL NOT NULL          -- ROS 시각(s)
);
CREATE TABLE IF NOT EXISTS missions (
  mission_id  TEXT PRIMARY KEY,
  worker_id   TEXT NOT NULL,
  robot_ns    TEXT,
  bay         TEXT NOT NULL,           -- A | B | C
  state       INTEGER NOT NULL,        -- Mission.msg 상수
  created_at  REAL NOT NULL,
  waiting_at  REAL,                    -- Z-05 도착 (리드타임 = waiting_at - created_at, T_lim 판정)
  finished_at REAL,
  result      TEXT,                    -- SUCCESS | FAIL_TLIM | FAIL_WRONG_BAY | FAIL_INTERVENTION | ...
  note        TEXT
);
CREATE TABLE IF NOT EXISTS events (   -- 상태 전이·개입·안전 이벤트 (IPH·성공률 산출 원천)
  id        INTEGER PRIMARY KEY AUTOINCREMENT,
  t         REAL NOT NULL,
  robot_ns  TEXT,
  kind      TEXT NOT NULL,             -- STATE | INTERVENTION | ESTOP | LOST | REACQUIRED | ...
  value     TEXT,
  note      TEXT
);
