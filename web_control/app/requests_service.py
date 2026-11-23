"""Job 요청·인계 요청의 발신 기록과 재전송 (E 가이드 1·2).

규칙 (INTERFACES §5.3):
  1. 작업자 조작마다 새 ID 발급 → **먼저 DB에 기록** → A로 전송 → 결과 기록
  2. 응답 미확인(타임아웃)은 실패가 아니다 → UNCONFIRMED 로 두고 **같은 ID로** 재전송
  3. A가 이미 처리한 이벤트를 재전송하면 A는 최초 결과를 돌려준다 → 같은 인계가 두 번 반영되지 않음
  4. 서버 재시작 시 PENDING_SEND / UNCONFIRMED / RECOVERY_PENDING 기록을 같은 ID로 다시 보낸다
  5. A가 복구 확인 중(RecoveryPending)이면 RECOVERY_PENDING 으로 두고, 복구가 끝난 뒤 같은 ID로 재전송한다
"""
import uuid

from .db import FINAL_STATUSES, OPEN_STATUSES, Database, now
from .mission_client import MissionClient, MissionRecoveryPending, MissionTimeout


_OPEN_SQL = "(" + ",".join(f"'{x}'" for x in OPEN_STATUSES) + ")"


class RequestRejected(Exception):
    """E 쪽에서 요청을 받지 않음 (API 409)."""


class RequestService:
    def __init__(self, db: Database, client: MissionClient, on_change=None):
        self.db, self.client = db, client
        self.on_change = on_change  # 기록이 바뀌면 호출 (웹소켓 방송용)

    async def _changed(self):
        if self.on_change:
            await self.on_change()

    # ------------------------------------------------------------------ Job 요청
    async def request_job(self, pickup: str, dropoff: str) -> dict:
        rid = f"req-{uuid.uuid4().hex[:12]}"
        t = now()
        self.db.execute(
            "INSERT INTO job_requests(request_id, pickup, dropoff, status, created_at, updated_at) "
            "VALUES (?,?,?,?,?,?)",
            (rid, pickup, dropoff, "PENDING_SEND", t, t),
        )
        return await self._send_job(rid)

    async def retry_job(self, request_id: str) -> dict:
        row = self.get_job_request(request_id)
        if row is None:
            raise KeyError(request_id)
        if row["status"] in FINAL_STATUSES:
            return row  # 이미 결과가 있음 — 다시 보내지 않는다
        return await self._send_job(request_id)

    async def _send_job(self, request_id: str) -> dict:
        row = self.get_job_request(request_id)
        self.db.execute("UPDATE job_requests SET attempts=attempts+1, updated_at=? WHERE request_id=?",
                        (now(), request_id))
        try:
            res = await self.client.submit_job(request_id, row["pickup"], row["dropoff"])
        except MissionTimeout as e:
            self._mark("job_requests", "request_id", request_id, "UNCONFIRMED", str(e) or "응답 시간 초과")
        except Exception as e:  # 통신 오류도 결과 미확인으로 취급
            self._mark("job_requests", "request_id", request_id, "UNCONFIRMED", f"{type(e).__name__}: {e}")
        else:
            status = "ACCEPTED" if res.get("outcome") == "accepted" else "REJECTED"
            self.db.execute(
                "UPDATE job_requests SET status=?, job_id=?, mission_id=?, reason=?, last_error=NULL, updated_at=? "
                f"WHERE request_id=? AND status IN {_OPEN_SQL}",
                (status, res.get("job_id"), res.get("mission_id"), res.get("reason"), now(), request_id),
            )
        await self._changed()
        return self.get_job_request(request_id)

    def get_job_request(self, request_id: str) -> dict | None:
        return self.db.one("SELECT * FROM job_requests WHERE request_id=?", (request_id,))

    def list_job_requests(self, limit=100) -> list[dict]:
        return self.db.all("SELECT * FROM job_requests ORDER BY created_at DESC LIMIT ?", (limit,))

    # ------------------------------------------------------------------ 인계 요청
    async def request_handoff(self, mission_id: str, step_id: str) -> dict:
        """작업자가 '인계 완료'를 누름 → 새 event_id 발급."""
        existing = self.db.one(
            "SELECT * FROM handoff_events WHERE mission_id=? AND step_id=? "
            "AND status IN ('PENDING_SEND','UNCONFIRMED','RECOVERY_PENDING','ACCEPTED') "
            "ORDER BY created_at DESC LIMIT 1",
            (mission_id, step_id),
        )
        if existing:
            if existing["status"] == "ACCEPTED":
                raise RequestRejected(f"{step_id} 인계는 이미 A가 수락했습니다 ({existing['event_id']})")
            raise RequestRejected(
                f"{step_id} 인계 요청({existing['event_id']})의 결과를 아직 모릅니다. 새로 누르지 말고 재전송하세요"
            )
        eid = f"evt-{uuid.uuid4().hex[:12]}"
        t = now()
        self.db.execute(
            "INSERT INTO handoff_events(event_id, mission_id, step_id, status, created_at, updated_at) "
            "VALUES (?,?,?,?,?,?)",
            (eid, mission_id, step_id, "PENDING_SEND", t, t),
        )
        return await self._send_handoff(eid)

    async def retry_handoff(self, event_id: str) -> dict:
        """같은 event_id·mission_id·step_id 로 재전송."""
        row = self.get_handoff(event_id)
        if row is None:
            raise KeyError(event_id)
        if row["status"] in FINAL_STATUSES:
            return row
        return await self._send_handoff(event_id)

    async def _send_handoff(self, event_id: str) -> dict:
        row = self.get_handoff(event_id)
        self.db.execute("UPDATE handoff_events SET attempts=attempts+1, updated_at=? WHERE event_id=?",
                        (now(), event_id))
        try:
            outcome = await self.client.send_handoff(row["event_id"], row["mission_id"], row["step_id"])
        except MissionRecoveryPending as e:
            self._mark("handoff_events", "event_id", event_id, "RECOVERY_PENDING", str(e) or "A 복구 확인 대기")
        except MissionTimeout as e:
            self._mark("handoff_events", "event_id", event_id, "UNCONFIRMED", str(e) or "응답 시간 초과")
        except Exception as e:
            self._mark("handoff_events", "event_id", event_id, "UNCONFIRMED", f"{type(e).__name__}: {e}")
        else:
            status = {"accepted": "ACCEPTED", "rejected": "REJECTED", "conflict": "CONFLICT"}.get(outcome)
            if status is None:
                self._mark("handoff_events", "event_id", event_id, "UNCONFIRMED", f"알 수 없는 응답: {outcome!r}")
            else:
                self._mark("handoff_events", "event_id", event_id, status, None)
        await self._changed()
        return self.get_handoff(event_id)

    def get_handoff(self, event_id: str) -> dict | None:
        return self.db.one("SELECT * FROM handoff_events WHERE event_id=?", (event_id,))

    def list_handoffs(self, limit=100) -> list[dict]:
        return self.db.all("SELECT * FROM handoff_events ORDER BY created_at DESC LIMIT ?", (limit,))

    # ------------------------------------------------------------------ 재시작 복구
    async def recover(self) -> dict:
        """E 재시작 후 결과를 모르는 요청을 같은 ID로 다시 보낸다."""
        jobs = self.db.all(f"SELECT request_id FROM job_requests WHERE status IN {OPEN_STATUSES} ORDER BY created_at")
        hos = self.db.all(f"SELECT event_id FROM handoff_events WHERE status IN {OPEN_STATUSES} ORDER BY created_at")
        for r in jobs:
            await self._send_job(r["request_id"])
        for r in hos:
            await self._send_handoff(r["event_id"])
        return {"job_requests": len(jobs), "handoffs": len(hos)}

    # ------------------------------------------------------------------
    def _mark(self, table, key_col, key, status, error):
        """결과가 아직 열린 상태(전송 중·미확인·복구 대기)일 때만 갱신한다.

        같은 ID 전송이 겹치면(예: 재전송 중 이전 전송의 타임아웃이 늦게 도착) 먼저 저장된
        최종 결과(ACCEPTED·REJECTED·CONFLICT)를 늦게 끝난 전송이 덮어쓰지 않는다.
        A도 같은 ID에는 처음 결과를 다시 돌려주므로(INTERFACES §5.3) 처음 받은 최종 결과를 유지한다.
        """
        self.db.execute(
            f"UPDATE {table} SET status=?, last_error=?, updated_at=? WHERE {key_col}=? AND status IN {_OPEN_SQL}",
            (status, error, now(), key),
        )
