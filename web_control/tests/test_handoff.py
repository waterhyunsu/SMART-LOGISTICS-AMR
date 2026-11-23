"""E 가이드 완료 테스트: 응답 유실 뒤 재전송, 동일 키·다른 단계 충돌, 재시작 후 미확인 요청 복구,
같은 인계의 중복 반영 방지. 모두 임시 SQLite + A 테스트 대역(MOCK)."""
from conftest import run_mission, waiting_step


def test_normal_handoff_is_recorded_then_accepted(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)

    r = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step})
    assert r.status_code == 201
    rec = r.json()
    assert rec["event_id"].startswith("evt-")
    assert rec["status"] == "ACCEPTED" and rec["attempts"] == 1
    assert mock_a.handoff_accept_count[step] == 1


def test_ack_lost_then_retry_with_same_event_applies_once(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)

    mock_a.drop_next_ack = True                      # A는 처리했지만 응답이 사라짐
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"
    assert mock_a.handoff_accept_count[step] == 1    # A에서는 이미 완료

    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["event_id"] == rec["event_id"]      # 같은 키 재사용
    assert again["status"] == "ACCEPTED" and again["attempts"] == 2
    assert mock_a.handoff_accept_count[step] == 1    # 두 번 반영되지 않음


def test_timeout_without_processing_then_retry(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)

    mock_a.timeout_next = True                       # A가 처리하지 못함
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"
    assert step not in mock_a.handoff_accept_count

    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["status"] == "ACCEPTED"
    assert mock_a.handoff_accept_count[step] == 1


def test_same_event_id_on_different_step_is_conflict(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "ACCEPTED"

    # 같은 event_id 가 다른 단계로 기록된 상황 (예: 잘못된 복구) 을 만들고 재전송
    other = step.replace("-PICKUP", "-DROPOFF")
    db = http.app.state.db
    db.execute("INSERT INTO handoff_events(event_id, mission_id, step_id, status, created_at, updated_at) "
               "VALUES ('evt-dup', ?, ?, 'UNCONFIRMED', 'x', 'x')", (mid, other))
    mock_a.handoff_results["evt-dup"] = (step, "accepted")   # A 원장에는 이 키가 다른 단계로 기록됨
    out = http.post("/api/handoffs/evt-dup/retry").json()
    assert out["status"] == "CONFLICT"
    assert other not in mock_a.completed


def test_restart_recovers_unconfirmed_handoff(make_client, mock_a, db_path):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.drop_next_ack = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"
    http.__exit__(None, None, None)                  # E 종료 (A는 계속 실행 중)

    http2 = make_client(mock_a)                      # 같은 DB로 E 재시작 → 자동 복구
    assert http2.get("/api/recovery").json() == {"job_requests": 0, "handoffs": 1}
    after = http2.get("/api/handoffs").json()[0]
    assert after["event_id"] == rec["event_id"] and after["status"] == "ACCEPTED"
    assert mock_a.handoff_accept_count[step] == 1


def test_restart_recovers_record_saved_before_send(make_client, mock_a):
    """전송 직전에 E가 죽은 경우: DB에는 PENDING_SEND 로 남아 있어야 하고 재시작 때 보낸다."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    db = http.app.state.db
    db.execute("INSERT INTO handoff_events(event_id, mission_id, step_id, status, created_at, updated_at) "
               "VALUES ('evt-crash', ?, ?, 'PENDING_SEND', 'x', 'x')", (mid, step))
    http.__exit__(None, None, None)

    http2 = make_client(mock_a)
    rec = [h for h in http2.get("/api/handoffs").json() if h["event_id"] == "evt-crash"][0]
    assert rec["status"] == "ACCEPTED"
    assert mock_a.handoff_accept_count[step] == 1


def test_new_press_while_unconfirmed_is_blocked(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.timeout_next = True
    http.post("/api/handoffs", json={"mission_id": mid, "step_id": step})
    r = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step})
    assert r.status_code == 409                      # 새 이벤트 대신 재전송을 써야 함
    r = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step})
    assert r.status_code == 409


def test_second_press_after_accept_is_blocked(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    assert http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()["status"] == "ACCEPTED"
    assert http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).status_code == 409
    assert mock_a.handoff_accept_count[step] == 1


def test_handoff_for_step_not_waiting_is_rejected_and_new_event_allowed(make_client, mock_a):
    http = make_client(mock_a)
    http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"})
    run_mission(http, mock_a)
    step = mock_a.claim_next_goal()["step_id"]       # 이동 중 (아직 도착 전)
    mid = mock_a.mission_id

    first = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert first["status"] == "REJECTED"             # 도착 전 인계는 A가 거부

    mock_a.arrive(step)                              # 실제 도착 후 작업자가 다시 확인 → 새 이벤트
    second = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert second["event_id"] != first["event_id"]
    assert second["status"] == "ACCEPTED"


def test_retry_of_final_record_does_not_resend(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["attempts"] == 1 and again["status"] == "ACCEPTED"


def test_a_unreachable_is_unconfirmed_not_failed(make_client, mock_a):
    class Broken(type(mock_a)):
        async def handoff(self, *a, **k):
            raise ConnectionError("A 연결 끊김")
    a = Broken(mock_a.wps, auto_run=False)
    http = make_client(a)
    mid, step = waiting_step(http, a)
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED" and "ConnectionError" in rec["last_error"]


# ---------------------------------------------------------------- A 복구 확인 중 (RecoveryPending)
# A README "D·E 복구 연동 경계" 3번: RecoveryPending 은 아직 처리되지 않은 이벤트가 복구 확인을 기다리는 상태.
# 이미 처리된 이벤트의 재전송은 저장된 결과를 반환한다.

def test_new_handoff_during_a_recovery_is_recovery_pending(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.recovery_required = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "RECOVERY_PENDING"
    assert step not in mock_a.handoff_accept_count          # A는 처리하지 않음

    assert http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).status_code == 409

    mock_a.recovery_required = False                        # A 복구 확인 완료
    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["event_id"] == rec["event_id"] and again["status"] == "ACCEPTED"
    assert mock_a.handoff_accept_count[step] == 1


def test_known_event_replays_result_during_a_recovery(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.drop_next_ack = True                             # A 처리 후 응답 유실
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"

    mock_a.recovery_required = True                         # 그 사이 A가 재시작돼 복구 확인 중
    again = http.post(f"/api/handoffs/{rec['event_id']}/retry").json()
    assert again["status"] == "ACCEPTED"                    # 기록된 이벤트는 저장된 결과 재사용
    assert mock_a.handoff_accept_count[step] == 1


def test_e_restart_resends_recovery_pending(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.recovery_required = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "RECOVERY_PENDING"
    http.__exit__(None, None, None)

    mock_a.recovery_required = False
    http2 = make_client(mock_a)                             # E 재시작 → 같은 ID로 재전송
    assert http2.get("/api/recovery").json() == {"job_requests": 0, "handoffs": 1}
    after = http2.get("/api/handoffs").json()[0]
    assert after["event_id"] == rec["event_id"] and after["status"] == "ACCEPTED"
    assert mock_a.handoff_accept_count[step] == 1


def test_empty_ids_are_refused_by_e_before_sending(make_client, mock_a):
    """A는 빈 ID 인계를 ValueError로 거절한다. E는 보내기 전에 422로 막고 기록도 남기지 않는다."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    for body in ({"mission_id": "", "step_id": step}, {"mission_id": mid, "step_id": "  "}):
        assert http.post("/api/handoffs", json=body).status_code == 422
    assert http.get("/api/handoffs").json() == []


def test_last_dropoff_away_from_home_shows_home_return_then_completed(make_client, mock_a):
    """A와 같이: 마지막 하역이 HOME이 아니면 returning_home → HOME 복귀 목표(step_id 없음) → completed."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    assert http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()["status"] == "ACCEPTED"
    goal = mock_a.claim_next_goal()
    assert mock_a.arrive(goal["step_id"])
    assert http.post("/api/handoffs", json={"mission_id": mid, "step_id": goal["step_id"]}).json()["status"] == "ACCEPTED"

    a = http.get("/api/state").json()["a_state"]
    assert a["mission_status"] == "returning_home" and a["current_goal"] is None
    home = mock_a.claim_next_goal()
    assert home["step_id"] is None and home["waypoint_id"] == "HOME"
    assert http.get("/api/state").json()["a_state"]["current_goal"]["step_id"] is None
    mock_a.arrive_home()
    a = http.get("/api/state").json()["a_state"]
    assert a["mission_status"] == "completed" and [j["status"] for j in a["jobs"]] == ["completed"]
    r = http.post("/api/job-requests", json={"pickup": "PICKUP_1", "dropoff": "DROP_1"}).json()
    assert r["status"] == "REJECTED"   # A: 완료 후 등록은 입력 오류


def test_sync_resends_unconfirmed_when_a_already_completed_the_step(make_client, mock_a):
    """E 가이드 2: E는 결과 미확인, A는 이미 완료 → 같은 event_id로 다시 보내 A 결과로 맞춤. 반영은 1회."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.drop_next_ack = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "UNCONFIRMED"
    svc = http.app.state.svc

    import asyncio
    a_state = mock_a._snapshot()
    assert next(s for s in a_state["steps"] if s["step_id"] == step)["status"] == "completed"
    assert asyncio.run(svc.sync_with_a(a_state)) == 1
    after = svc.get_handoff(rec["event_id"])
    assert after["status"] == "ACCEPTED" and after["attempts"] == 2
    assert mock_a.handoff_accept_count[step] == 1
    assert asyncio.run(svc.sync_with_a(mock_a._snapshot())) == 0   # 맞춰진 뒤에는 보내지 않음


def test_sync_leaves_unprocessed_unconfirmed_alone(make_client, mock_a):
    """A가 처리하지 않은(단계 미완료) 미확인 기록은 자동으로 보내지 않는다 → 작업자가 재전송."""
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.timeout_next = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    import asyncio
    assert asyncio.run(http.app.state.svc.sync_with_a(mock_a._snapshot())) == 0
    assert http.app.state.svc.get_handoff(rec["event_id"])["status"] == "UNCONFIRMED"


def test_sync_resends_recovery_pending_after_a_recovers(make_client, mock_a):
    http = make_client(mock_a)
    mid, step = waiting_step(http, mock_a)
    mock_a.recovery_required = True
    rec = http.post("/api/handoffs", json={"mission_id": mid, "step_id": step}).json()
    assert rec["status"] == "RECOVERY_PENDING"
    import asyncio
    svc = http.app.state.svc
    assert asyncio.run(svc.sync_with_a(mock_a._snapshot())) == 0      # 아직 복구 중 → 보내지 않음
    mock_a.recovery_required = False
    assert asyncio.run(svc.sync_with_a(mock_a._snapshot())) == 1
    assert svc.get_handoff(rec["event_id"])["status"] == "ACCEPTED"
