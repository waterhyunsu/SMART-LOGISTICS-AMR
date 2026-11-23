"""Job 등록 요청 기록과 재전송 (인계 요청과 분리)."""


def test_job_request_is_recorded_and_accepted(make_client, mock_a):
    http = make_client(mock_a)
    r = http.post("/api/job-requests", json={"pickup": "HOME", "dropoff": "A"})
    assert r.status_code == 201
    rec = r.json()
    assert rec["request_id"].startswith("req-") and rec["status"] == "ACCEPTED"
    assert rec["job_id"] == "JOB-001"


def test_invalid_waypoint_rejected_by_e_before_sending(make_client, mock_a):
    http = make_client(mock_a)
    assert http.post("/api/job-requests", json={"pickup": "Z", "dropoff": "A"}).status_code == 422
    assert http.post("/api/job-requests", json={"pickup": "A", "dropoff": "A"}).status_code == 422
    assert http.get("/api/job-requests").json() == []
    assert mock_a.jobs == {}


def test_job_ack_lost_then_retry_registers_once(make_client, mock_a):
    http = make_client(mock_a)
    mock_a.drop_next_ack = True
    rec = http.post("/api/job-requests", json={"pickup": "B", "dropoff": "C"}).json()
    assert rec["status"] == "UNCONFIRMED" and len(mock_a.jobs) == 1

    again = http.post(f"/api/job-requests/{rec['request_id']}/retry").json()
    assert again["status"] == "ACCEPTED" and again["job_id"] == "JOB-001"
    assert len(mock_a.jobs) == 1                     # 같은 Job이 두 번 등록되지 않음


def test_restart_recovers_unconfirmed_job_request(make_client, mock_a):
    http = make_client(mock_a)
    mock_a.timeout_next = True
    rec = http.post("/api/job-requests", json={"pickup": "D", "dropoff": "HOME"}).json()
    assert rec["status"] == "UNCONFIRMED" and mock_a.jobs == {}
    http.__exit__(None, None, None)

    http2 = make_client(mock_a)
    assert http2.get("/api/recovery").json() == {"job_requests": 1, "handoffs": 0}
    assert http2.get("/api/job-requests").json()[0]["status"] == "ACCEPTED"
    assert len(mock_a.jobs) == 1


def test_state_view_comes_from_a(make_client, mock_a):
    """화면 상태는 A가 준 값이다. E의 발신 기록이 ACCEPTED 여도 A 상태를 그대로 보여준다."""
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "B", "dropoff": "C"})
    s = http.get("/api/state").json()
    assert s["a_connected"] is True
    steps = {x["step_id"]: x["status"] for x in s["a_state"]["steps"]}
    assert steps == {"JOB-001-PICKUP": "READY", "JOB-001-DROPOFF": "PENDING"}


def test_new_job_during_mission_applies_after_current_step(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "B", "dropoff": "C"})
    http.post("/api/mission/start")
    step = mock_a.start_next()
    http.post("/api/job-requests", json={"pickup": "D", "dropoff": "HOME"})
    s = http.get("/api/state").json()["a_state"]
    assert s["current_step_id"] == step and s["active_version"] == 1
    assert s["pending_replan"] == ["JOB-002-PICKUP", "JOB-002-DROPOFF"]
