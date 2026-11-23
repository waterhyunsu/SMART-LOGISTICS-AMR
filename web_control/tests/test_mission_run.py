"""미션 실행 요청 (INTERFACES §11.1 "미션 실행 요청", §4.6 "A가 준비 상태를 종합하여 미션 실행 허용 여부를 결정").
A 코드에는 아직 없어 문서 기준으로 MOCK 대역에 넣었다. A 구현이 올라오면 호출 이름·결과 값을 맞춘다."""
from conftest import run_mission


def _run(http, mid):
    return http.post("/api/mission/run", json={"mission_id": mid})


def test_registered_jobs_do_not_move_until_run_is_allowed(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    assert mock_a.claim_next_goal() is None                    # 등록만으로는 출발하지 않음
    a = http.get("/api/state").json()["a_state"]
    assert a["run_authorized"] is False and a["current_goal"] is None

    run_mission(http, mock_a)
    assert mock_a.claim_next_goal()["waypoint_id"] == "PICKUP_1"
    assert http.get("/api/state").json()["a_state"]["run_authorized"] is True


def test_run_without_jobs_is_rejected(make_client, mock_a):
    http = make_client(mock_a)
    r = _run(http, mock_a.mission_id).json()
    assert r["outcome"] == "rejected" and r["reason"] == "등록된 작업 없음"


def test_run_is_rejected_when_not_ready_then_allowed(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    http.post("/api/mock/faults", json={"not_ready": True})
    r = _run(http, mock_a.mission_id).json()
    assert r["outcome"] == "rejected" and "준비 상태" in r["reason"]
    assert mock_a.claim_next_goal() is None

    http.post("/api/mock/faults", json={"not_ready": False})
    assert _run(http, mock_a.mission_id).json()["outcome"] == "accepted"


def test_repeated_run_request_returns_same_result(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    http.post("/api/mock/faults", json={"drop_next_ack": True})
    assert _run(http, mock_a.mission_id).status_code == 504   # A는 허용했지만 응답 유실 → 결과 미확인
    assert _run(http, mock_a.mission_id).json()["outcome"] == "accepted"   # 다시 요청 → 같은 결과


def test_run_for_other_mission_or_empty_id(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    assert _run(http, "OTHER").json()["outcome"] == "rejected"
    assert _run(http, " ").status_code == 422
    assert mock_a.run_authorized is False


def test_jobs_added_while_running_continue_without_new_run(make_client, mock_a):
    """실행 허용 후 추가한 주문은 다시 실행 요청 없이 재계획으로 이어진다."""
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    run_mission(http, mock_a)
    goal = mock_a.claim_next_goal()
    http.post("/api/job-requests", json={"pickup": "DROP_1", "dropoff": "HOME"})
    mock_a.arrive(goal["step_id"])
    http.post("/api/handoffs", json={"mission_id": mock_a.mission_id, "step_id": goal["step_id"]})
    assert mock_a.claim_next_goal()["step_id"].endswith("-DROPOFF")


def test_run_after_completion_is_rejected_even_if_previously_allowed(make_client, mock_a):
    """미션이 끝난 뒤의 실행 요청은 '이미 실행 중'이 아니라 '이미 완료된 미션'으로 거부."""
    from conftest import waiting_step
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a, pickup="PICKUP_1", dropoff="HOME")
    http.post("/api/handoffs", json={"mission_id": mid, "step_id": step})
    goal = mock_a.claim_next_goal(); mock_a.arrive(goal["step_id"])
    http.post("/api/handoffs", json={"mission_id": mid, "step_id": goal["step_id"]})
    assert mock_a.mission_status == "completed" and mock_a.run_authorized
    r = _run(http, mid).json()
    assert r["outcome"] == "rejected" and r["reason"] == "이미 완료된 미션"
