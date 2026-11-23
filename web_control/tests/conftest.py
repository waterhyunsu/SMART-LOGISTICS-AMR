import os
import sys
from pathlib import Path

os.environ["WEB_CONTROL_NO_APP"] = "1"  # import 시 기본 앱(기본 DB 경로) 생성 방지
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from app.main import create_app, load_waypoints  # noqa: E402
from app.mock_mission import MockMissionManager  # noqa: E402


@pytest.fixture
def mock_a():
    """자동 주행 없이 테스트가 단계를 직접 진행시키는 A 대역."""
    return MockMissionManager(load_waypoints(), auto_run=False, run_id="")


@pytest.fixture
def db_path(tmp_path):
    return tmp_path / "test.db"


@pytest.fixture
def make_client(db_path):
    opened = []

    def _make(a):
        c = TestClient(create_app(db_path=db_path, client=a, run_mode="MOCK", auto_sync=False))
        c.__enter__()
        opened.append(c)
        return c

    yield _make
    for c in opened:
        c.__exit__(None, None, None)


def run_mission(http, a):
    r = http.post("/api/mission/run", json={"mission_id": a.mission_id})
    assert r.status_code == 200 and r.json()["outcome"] == "accepted", r.text


def waiting_step(http, a, pickup="PICKUP_1", dropoff="DROP_1"):
    """주문 하나를 등록하고 A 대역에서 첫 목표를 선점·도착시킨 뒤 인계 대기 단계 ID를 돌려준다.
    흐름: register_jobs → (계획 승인) → 미션 실행 요청(문서 §11.1) → claim_next_goal → D 도착 보고 → 인계 대기."""
    r = http.post("/api/job-requests", json={"pickup": pickup, "dropoff": dropoff})
    assert r.status_code == 201 and r.json()["status"] == "ACCEPTED"
    run_mission(http, a)
    goal = a.claim_next_goal()
    assert goal and goal["step_id"]
    assert a.arrive(goal["step_id"])
    return a.mission_id, goal["step_id"]
