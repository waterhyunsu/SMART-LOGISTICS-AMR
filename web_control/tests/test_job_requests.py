"""Job 등록 요청 기록과 재전송 (인계 요청과 분리). A register_jobs 계약에 맞춘다."""
from conftest import run_mission


def test_job_request_is_recorded_and_registered_as_step_pair(make_client, mock_a):
    http = make_client(mock_a)
    r = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    assert r.status_code == 201
    rec = r.json()
    assert rec["request_id"].startswith("req-") and rec["status"] == "ACCEPTED"
    jid = rec["job_id"]
    assert jid.startswith("JOB-") and rec["mission_id"] == mock_a.mission_id
    # A가 받은 형식: 주문 1건 = 수거·전달 TaskStep 쌍, 전달은 수거를 선행 조건으로
    p, d = mock_a.steps[f"{jid}-PICKUP"], mock_a.steps[f"{jid}-DROPOFF"]
    assert (p["kind"], p["waypoint_id"], p["prerequisite_ids"]) == ("PICKUP", "PICKUP_1", [])
    assert (d["kind"], d["waypoint_id"], d["prerequisite_ids"]) == ("DROPOFF", "DROP_1", [p["step_id"]])
    assert mock_a.cost_revision == "mock-table" and mock_a.plan_status == "applied"


def test_invalid_waypoint_rejected_by_e_before_sending(make_client, mock_a):
    http = make_client(mock_a)
    assert http.post("/api/job-requests", json={"pickup": "Z", "dropoff": "DROP_1"}).status_code == 422
    assert http.post("/api/job-requests", json={"pickup": "DROP_1", "dropoff": "DROP_1"}).status_code == 422
    assert http.get("/api/job-requests").json() == []
    assert mock_a.steps == {}


def test_job_ack_lost_then_retry_registers_once(make_client, mock_a):
    """A는 같은 job_id 재등록을 duplicate로 거부한다. E는 재전송에서 받은 duplicate를 '이미 등록됨'으로 본다."""
    http = make_client(mock_a)
    mock_a.drop_next_ack = True
    rec = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()
    assert rec["status"] == "UNCONFIRMED" and len(mock_a.steps) == 2

    again = http.post(f"/api/job-requests/{rec['request_id']}/retry").json()
    assert again["status"] == "ACCEPTED" and again["job_id"] == rec["job_id"]
    assert "이미 등록됨" in again["reason"] and "duplicate" in again["reason"]
    assert len(mock_a.steps) == 2                    # 같은 Job이 두 번 등록되지 않음
    assert mock_a.latest_request_id == 1             # 최적화 요청도 한 번


def test_restart_recovers_unconfirmed_job_request(make_client, mock_a):
    http = make_client(mock_a)
    mock_a.timeout_next = True
    rec = http.post("/api/job-requests", json={"pickup": "HOME", "dropoff": "PICKUP_1"}).json()
    assert rec["status"] == "UNCONFIRMED" and mock_a.steps == {}
    http.__exit__(None, None, None)

    http2 = make_client(mock_a)
    assert http2.get("/api/recovery").json() == {"job_requests": 1, "handoffs": 0}
    after = http2.get("/api/job-requests").json()[0]
    assert after["status"] == "ACCEPTED" and after["job_id"] == rec["job_id"]
    assert len(mock_a.steps) == 2


def test_state_view_comes_from_a(make_client, mock_a):
    """화면 상태는 A가 준 값(A 코드의 소문자 상태 값)이다."""
    http = make_client(mock_a)
    jid = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()["job_id"]
    s = http.get("/api/state").json()
    assert s["a_connected"] is True
    a = s["a_state"]
    assert a["mission_status"] == "active" and a["plan_status"] == "applied" and a["active_version"] == 1
    assert {x["step_id"]: x["status"] for x in a["steps"]} == {f"{jid}-PICKUP": "pending", f"{jid}-DROPOFF": "pending"}
    assert a["jobs"] == [{"job_id": jid, "status": "pending"}]
    assert a["current_goal"] is None and a["order"] == [f"{jid}-PICKUP", f"{jid}-DROPOFF"]


def test_new_job_during_mission_is_deferred_until_handoff(make_client, mock_a):
    """A node: 운행 중 받은 계획은 deferred, 현재 단계 인계가 끝나면 적용."""
    http = make_client(mock_a)
    first = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()["job_id"]
    run_mission(http, mock_a)
    goal = mock_a.claim_next_goal()
    second = http.post("/api/job-requests", json={"pickup": "DROP_1", "dropoff": "HOME"}).json()["job_id"]
    a = http.get("/api/state").json()["a_state"]
    assert a["current_goal"]["step_id"] == goal["step_id"] and a["active_version"] == 1
    assert a["plan_status"] == "deferred"
    assert a["deferred_order"] == [f"{first}-DROPOFF", f"{second}-PICKUP", f"{second}-DROPOFF"]

    mock_a.arrive(goal["step_id"])
    http.post("/api/handoffs", json={"mission_id": mock_a.mission_id, "step_id": goal["step_id"]})
    a = http.get("/api/state").json()["a_state"]
    assert a["plan_status"] == "applied" and a["active_version"] == 2
    assert a["order"] == [f"{first}-DROPOFF", f"{second}-PICKUP", f"{second}-DROPOFF"]


def test_a_input_error_is_rejected_not_retried(make_client, mock_a):
    """A가 입력을 거부하면(예: 비용표 버전 없음) REJECTED로 기록하고 사유를 남긴다."""
    http = make_client(mock_a)
    http.app.state.svc.cost_revision = ""
    rec = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()
    assert rec["status"] == "REJECTED" and "missing start waypoint or cost revision" in rec["reason"]
    assert mock_a.steps == {}


def test_sync_settles_unconfirmed_job_already_registered_in_a(make_client, mock_a):
    import asyncio
    http = make_client(mock_a)
    mock_a.drop_next_ack = True
    rec = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()
    assert rec["status"] == "UNCONFIRMED"
    svc = http.app.state.svc
    assert asyncio.run(svc.sync_with_a(mock_a._snapshot())) == 1
    row = svc.get_job_request(rec["request_id"])
    assert row["status"] == "ACCEPTED" and mock_a.latest_request_id == 1   # A 등록은 1회
