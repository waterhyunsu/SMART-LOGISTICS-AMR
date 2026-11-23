"""E의 SQLite 기록.

E가 저장하는 것은 '내가 A에게 무엇을 보냈고 A가 뭐라고 답했는가'(발신 기록)뿐이다.
미션·Job·TaskStep 의 실제 상태는 A가 소유하며 여기에 저장해 확정하지 않는다 (INTERFACES §11.4).
"""
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path

# 발신 기록 상태
#   PENDING_SEND : DB에 먼저 기록함, 아직 A의 결과를 모름 (전송 전/전송 중 종료 가능)
#   UNCONFIRMED  : 전송했지만 응답 미확인 (타임아웃·통신 오류·A 저장 예외)
#   RECOVERY_PENDING : A가 재시작 후 복구 확인 중이라 아직 처리하지 않음
#   ACCEPTED / REJECTED / CONFLICT : A의 결과를 받음 (최종)
OPEN_STATUSES = ("PENDING_SEND", "UNCONFIRMED", "RECOVERY_PENDING")
FINAL_STATUSES = ("ACCEPTED", "REJECTED", "CONFLICT")

SCHEMA = """
CREATE TABLE IF NOT EXISTS job_requests (
    request_id  TEXT PRIMARY KEY,      -- E가 발급 (재전송 시 재사용)
    pickup      TEXT NOT NULL,
    dropoff     TEXT NOT NULL,
    status      TEXT NOT NULL,
    job_id      TEXT,                  -- A가 부여
    mission_id  TEXT,
    reason      TEXT,
    attempts    INTEGER NOT NULL DEFAULT 0,
    last_error  TEXT,
    created_at  TEXT NOT NULL,
    updated_at  TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS handoff_events (
    event_id    TEXT PRIMARY KEY,      -- E가 작업자 조작마다 발급 (재전송 시 재사용)
    mission_id  TEXT NOT NULL,
    step_id     TEXT NOT NULL,
    status      TEXT NOT NULL,
    attempts    INTEGER NOT NULL DEFAULT 0,
    last_error  TEXT,
    created_at  TEXT NOT NULL,
    updated_at  TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_handoff_step ON handoff_events(mission_id, step_id);
"""


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class Database:
    def __init__(self, path: str | Path):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.Lock()
        with self._lock:
            self._conn.executescript(SCHEMA)
            self._conn.commit()

    def execute(self, sql: str, params=()) -> None:
        with self._lock:
            self._conn.execute(sql, params)
            self._conn.commit()

    def all(self, sql: str, params=()) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self._conn.execute(sql, params).fetchall()]

    def one(self, sql: str, params=()) -> dict | None:
        rows = self.all(sql, params)
        return rows[0] if rows else None

    def close(self):
        with self._lock:
            self._conn.close()
