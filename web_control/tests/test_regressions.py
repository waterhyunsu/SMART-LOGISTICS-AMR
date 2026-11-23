"""PR #18 리뷰 반영 회귀 테스트.

1. 같은 ID 전송이 겹칠 때, 늦게 끝난 전송의 타임아웃·오류가 이미 저장된 최종 결과(ACCEPTED)를
   UNCONFIRMED로 덮어쓰지 않는다.
2. 같은 DB로 MOCK 서버를 재시작해도 새 미션·단계 ID가 이전 기록과 겹치지 않아, 새 인계가 409로 막히지 않는다.
"""
import asyncio

import pytest

from app.db import Database
from app.main import load_waypoints
from app.mission_client import MissionTimeout
from app.mock_mission import MockMissionManager
from app.requests_service import RequestService

from conftest import waiting_step


class SlowFirstThenAccept:
    """첫 전송은 gate가 열릴 때까지 붙잡혀 있다가 실패하고, 그 사이 재전송은 바로 수락되는 A 대역."""

    def __init__(self, late_error: Exception):
        self.gate = asyncio.Event()
        self.late_error = late_error
        self.calls = 0

    async def _call(self, accepted):
        self.calls += 1
        if self.calls == 1:
            await self.gate.wait()
            raise self.late_error
        return accepted

    async def send_handoff(self, event_id, mission_id, step_id):
        return await self._call("accepted")

    async def submit_job(self, request_id, pickup, dropoff):
        return await self._call({"outcome": "accepted", "job_id": "JOB-X", "mission_id": None, "reason": ""})


async def _until(cond):
    for _ in range(100):
        if cond():
            return
        await asyncio.sleep(0)
    raise AssertionError("조건이 충족되지 않음")


@pytest.mark.parametrize("late_error", [MissionTimeout("늦게 도착한 타임아웃"), ConnectionError("늦게 끊김")])
def test_late_timeout_does_not_overwrite_accepted_handoff(tmp_path, late_error):
    async def scenario():
        a = SlowFirstThenAccept(late_error)
        svc = RequestService(Database(tmp_path / "t.db"), a)
        first = asyncio.create_task(svc.request_handoff("M-1", "S-1"))   # 1차 전송: 응답 대기 중
        await _until(lambda: a.calls == 1)
        eid = svc.list_handoffs()[0]["event_id"]

        again = await svc._send_handoff(eid)                              # 겹친 재전송: 수락됨
        assert again["status"] == "ACCEPTED"

        a.gate.set()                                                      # 1차 전송이 뒤늦게 실패
        late = await first
        assert late["event_id"] == eid
        assert late["status"] == "ACCEPTED" and late["last_error"] is None
        assert svc.get_handoff(eid)["attempts"] == 2

    asyncio.run(scenario())


def test_late_timeout_does_not_overwrite_accepted_job(tmp_path):
    async def scenario():
        a = SlowFirstThenAccept(MissionTimeout("늦게 도착한 타임아웃"))
        svc = RequestService(Database(tmp_path / "t.db"), a)
        first = asyncio.create_task(svc.request_job("HOME", "A"))
        await _until(lambda: a.calls == 1)
        rid = svc.list_job_requests()[0]["request_id"]

        again = await svc._send_job(rid)
        assert again["status"] == "ACCEPTED" and again["job_id"] == "JOB-X"

        a.gate.set()
        late = await first
        assert late["status"] == "ACCEPTED" and late["job_id"] == "JOB-X" and late["last_error"] is None

    asyncio.run(scenario())


def test_open_record_still_becomes_unconfirmed(make_client, mock_a):
    """가드가 정상적인 미확인 기록까지 막지 않는지 확인."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.timeout_next = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"
    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["status"] == "ACCEPTED"


def test_mock_restart_with_same_db_gets_new_ids_and_new_handoff_is_not_blocked(make_client):
    # 1회차 MOCK 서버: 인계까지 수락
    a1 = MockMissionManager(load_waypoints(), auto_run=False)
    http1 = make_client(a1)
    mid1, step1 = waiting_step(http1, a1, pickup="HOME", dropoff="A")
    assert http1.post("/api/handoffs", json={"mission_id": mid1, "step_id": step1}).json()["status"] == "ACCEPTED"
    http1.__exit__(None, None, None)

    # 같은 DB로 MOCK 서버 재시작 (A 대역도 새로 시작)
    a2 = MockMissionManager(load_waypoints(), auto_run=False)
    http2 = make_client(a2)
    mid2, step2 = waiting_step(http2, a2, pickup="HOME", dropoff="A")

    assert a1.run_id != a2.run_id
    assert mid2 != mid1 and step2 != step1                    # 이전 기록과 ID가 겹치지 않음
    r = http2.post("/api/handoffs", json={"mission_id": mid2, "step_id": step2})
    assert r.status_code == 201 and r.json()["status"] == "ACCEPTED"   # 409로 막히지 않음
    assert len(http2.get("/api/handoffs").json()) == 2        # 이전 기록은 그대로 보존
