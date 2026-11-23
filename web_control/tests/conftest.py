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
    return MockMissionManager(load_waypoints(), auto_run=False)


@pytest.fixture
def db_path(tmp_path):
    return tmp_path / "test.db"


@pytest.fixture
def make_client(db_path):
    opened = []

    def _make(a):
        c = TestClient(create_app(db_path=db_path, client=a, run_mode="MOCK"))
        c.__enter__()
        opened.append(c)
        return c

    yield _make
    for c in opened:
        c.__exit__(None, None, None)


def waiting_step(http, a, pickup="B", dropoff="C"):
    """Job 하나를 등록·미션 시작·첫 단계 도착까지 진행하고 인계 대기 단계 ID를 돌려준다."""
    r = http.post("/api/job-requests", json={"pickup": pickup, "dropoff": dropoff})
    assert r.status_code == 201 and r.json()["status"] == "ACCEPTED"
    assert http.post("/api/mission/start").status_code == 200
    step = a.start_next()
    assert a.arrive(step)
    return a.mission_id, step
