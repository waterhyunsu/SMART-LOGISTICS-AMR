"""Job 요청·인계 요청의 발신 기록과 재전송 (E 가이드 1·2).

규칙 (INTERFACES §5.3):
  1. 작업자 조작마다 새 ID 발급 → **먼저 DB에 기록** → A로 전송 → 결과 기록
  2. 응답 미확인(타임아웃)은 실패가 아니다 → UNCONFIRMED 로 두고 **같은 ID로** 재전송
  3. A가 이미 처리한 이벤트를 재전송하면 A는 최초 결과를 돌려준다 → 같은 인계가 두 번 반영되지 않음
  4. 서버 재시작 시 PENDING_SEND / UNCONFIRMED / RECOVERY_PENDING 기록을 같은 ID로 다시 보낸다
  5. A가 복구 확인 중(RecoveryPending)이면 RECOVERY_PENDING 으로 두고, 복구가 끝난 뒤 같은 ID로 재전송한다
  6. 같은 ID 전송이 겹쳐도 먼저 저장된 최종 결과는 늦게 끝난 전송이 덮어쓰지 않는다
  7. A 상태와 E 기록이 어긋나면(E는 결과 미확인인데 A는 단계·Job을 이미 완료/등록, 또는 A 복구가 끝났는데
     E는 복구 대기) 같은 ID로 다시 보내 A의 결과를 받아와 맞춘다 (E 가이드 2 "A의 결과를 조회해 동기화")

작업 등록은 A `register_jobs(steps, start_waypoint_id, cost_revision)`에 맞춘다.
  - E가 주문마다 고유한 job_id를 만들고, 기록할 때 함께 저장한다 → 재전송해도 같은 job_id·step_id
  - A는 같은 job_id를 다시 받으면 'duplicate job or step'으로 거부한다. 이 job_id는 이 주문 기록에서만
    만들어지므로, 재전송(시도 2회 이상)에서 이 응답이 오면 앞선 전송이 이미 등록된 것으로 보고 ACCEPTED 처리한다.
    [초안-팀합의필요] A와 이 해석을 확인해야 한다.
  - start_waypoint_id·cost_revision 은 아직 누가 넣을지 미정이라 서버 설정값으로 둔다 (START_WAYPOINT, COST_REVISION).
"""
import time
import uuid

from .db import FINAL_STATUSES, OPEN_STATUSES, Database, now
from .mission_client import MissionClient, MissionInputError, MissionRecoveryPending, MissionTimeout, job_steps


_OPEN_SQL = "(" + ",".join(f"'{x}'" for x in OPEN_STATUSES) + ")"


class RequestRejected(Exception):
    """E 쪽에서 요청을 받지 않음 (API 409)."""


class RequestService:
    def __init__(self, db: Database, client: MissionClient, on_change=None,
                 start_waypoint_id: str = "HOME", cost_revision: str = "mock-table"):
        self.db, self.client = db, client
        self.on_change = on_change  # 기록이 바뀌면 호출 (웹소켓 방송용)
        self.start_waypoint_id, self.cost_revision = start_waypoint_id, cost_revision
        self._sync_tried: dict[str, float] = {}   # 자동 동기화 재전송 간격 제한 (같은 기록을 연달아 보내지 않음)

    async def _changed(self):
        if self.on_change:
            await self.on_change()

    # ------------------------------------------------------------------ Job 요청
    async def request_job(self, pickup: str, dropoff: str) -> dict:
        token = uuid.uuid4().hex[:12]
        rid, jid = f"req-{token}", f"JOB-{token[:8].upper()}"
        t = now()
        self.db.execute(
            "INSERT INTO job_requests(request_id, pickup, dropoff, status, job_id, created_at, updated_at) "
            "VALUES (?,?,?,?,?,?,?)",
            (rid, pickup, dropoff, "PENDING_SEND", jid, t, t),
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
        steps = job_steps(row["job_id"], row["pickup"], row["dropoff"])
        try:
            await self.client.register_jobs(steps, self.start_waypoint_id, self.cost_revision)
        except MissionInputError as e:
            if "duplicate" in str(e) and row["attempts"] >= 1:
                # 이 job_id는 이 기록에서만 만들어짐 → 앞선 전송(응답 유실)이 이미 등록함
                self._settle_job(request_id, "ACCEPTED", f"이미 등록됨 (A: {e})")
            else:
                self._settle_job(request_id, "REJECTED", f"A 입력 오류: {e}")
        except MissionRecoveryPending as e:
            self._mark("job_requests", "request_id", request_id, "RECOVERY_PENDING", str(e) or "A 복구 확인 대기")
        except MissionTimeout as e:
            self._mark("job_requests", "request_id", request_id, "UNCONFIRMED", str(e) or "응답 시간 초과")
        except Exception as e:  # 통신 오류도 결과 미확인으로 취급
            self._mark("job_requests", "request_id", request_id, "UNCONFIRMED", f"{type(e).__name__}: {e}")
        else:
            self._settle_job(request_id, "ACCEPTED", "")
        await self._changed()
        return self.get_job_request(request_id)

    def _settle_job(self, request_id, status, reason):
        mission_id = getattr(self.client, "mission_id", None)
        self.db.execute(
            "UPDATE job_requests SET status=?, mission_id=?, reason=?, last_error=NULL, updated_at=? "
            f"WHERE request_id=? AND status IN {_OPEN_SQL}",
            (status, mission_id if status == "ACCEPTED" else None, reason, now(), request_id),
        )

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
            outcome = await self.client.handoff(row["event_id"], row["mission_id"], row["step_id"])
        except MissionInputError as e:
            self._mark("handoff_events", "event_id", event_id, "REJECTED", f"A 입력 오류: {e}")
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

    # ------------------------------------------------------------------ A 상태와 동기화
    async def sync_with_a(self, a_state: dict, min_interval: float = 2.0) -> int:
        """A 상태와 E 기록이 어긋난 기록만 같은 ID로 다시 보낸다. 상태를 E가 추측해 바꾸지 않고 A의 응답으로 확정한다.
        PENDING_SEND(전송 중일 수 있음)는 건드리지 않는다. 보낸 건수를 돌려준다."""
        if not a_state or a_state.get("mission_id") is None:
            return 0
        done_steps = {s["step_id"] for s in a_state.get("steps", []) if s.get("status") == "completed"}
        a_jobs = {j["job_id"] for j in a_state.get("jobs", [])}
        recovering = bool(a_state.get("recovery_required"))
        due = []
        for r in self.db.all("SELECT * FROM handoff_events WHERE mission_id=? AND status IN ('UNCONFIRMED','RECOVERY_PENDING')",
                             (a_state["mission_id"],)):
            if (r["status"] == "UNCONFIRMED" and r["step_id"] in done_steps) or \
               (r["status"] == "RECOVERY_PENDING" and not recovering):
                due.append(("h", r["event_id"]))
        for r in self.db.all("SELECT * FROM job_requests WHERE status='UNCONFIRMED'"):
            if r["job_id"] in a_jobs:
                due.append(("j", r["request_id"]))
        sent, t = 0, time.monotonic()
        for kind, key in due:
            if t - self._sync_tried.get(key, -1e9) < min_interval:
                continue
            self._sync_tried[key] = t
            await (self._send_handoff(key) if kind == "h" else self._send_job(key))
            sent += 1
        return sent

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
