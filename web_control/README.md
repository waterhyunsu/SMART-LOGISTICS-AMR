# web_control — E 웹 관제 (FastAPI·SQLite)

E 가이드([E_WEB_DASHBOARD.md](../docs/TEAM_GUIDES/E_WEB_DASHBOARD.md)) 1·2번 범위와 최소 관제 화면.

- **Job 요청과 인계 요청을 분리**해 각각 E가 ID(`request_id`, `event_id`)를 발급하고 **전송 전에 SQLite에 기록**한다.
- A의 결과를 `ACCEPTED` / `REJECTED` / `CONFLICT` / `UNCONFIRMED`(응답 미확인) / `RECOVERY_PENDING`(A 복구 확인 대기)로 구분해 기록한다.
- A 상태와 E 기록이 어긋나면 같은 ID로 다시 보내 A의 결과로 맞춘다 (자동 동기화).
- 재전송은 **같은 ID**를 사용한다. 서버 재시작 시 `PENDING_SEND`·`UNCONFIRMED`·`RECOVERY_PENDING` 기록을 같은 ID로 다시 보낸다.
- A가 재시작 후 복구 확인 중(`RecoveryPending`)이면 새 인계는 처리되지 않으므로 `RECOVERY_PENDING`으로 두고, 복구 후 같은 ID로 재전송한다 ([mission_manager README](../ros2_ws/src/mission_manager/README.md) "D·E 복구 연동 경계" 3번).
- 화면의 미션·Job·TaskStep 상태는 **A가 준 값만** 표시한다. E의 기록으로 상태를 확정하지 않는다 (INTERFACES §5.3, §11.4).

## 빠른 재현 (MOCK, A 테스트 대역)

아래 명령은 작성 시 실제로 실행해 확인했다. 출력의 ID·시각은 실행마다 다르다.

### 1. 서버 실행

```bash
cd web_control
python3 -m venv .venv && . .venv/bin/activate && pip install -r requirements.txt
env -u PYTHONPATH DB_PATH=/tmp/web_control_demo.db uvicorn app.main:app --host 0.0.0.0 --port 8000
```

- 브라우저: `http://localhost:8000` (다른 PC에서는 `http://<서버 IP>:8000`)
- `env -u PYTHONPATH`: ROS2를 `source`한 터미널에서도 venv 패키지만 쓰게 한다 (아래 "테스트" 주의 참고)
- 새 DB로 시작하려면 `DB_PATH` 파일을 지우고 다시 실행한다
- 아래 curl은 서버를 띄운 터미널과 **다른 터미널**에서 실행한다

### 2. 주문(Job) 1건 등록

화면: 왼쪽 아래 "물류 작업"에서 수거·전달 지점을 고르고 **작업 요청** → **미션 실행 요청**.
주문이 등록되면 A가 계획을 승인하지만, **미션 실행 요청이 허용되기 전에는 출발하지 않는다** (INTERFACES §11.1, §4.6).
실행이 허용된 뒤 추가한 주문은 다시 실행 요청 없이 현재 단계 인계 후 재계획에 반영된다.

curl:

```bash
curl -s -X POST localhost:8000/api/job-requests -H 'Content-Type: application/json' -d '{"pickup":"PICKUP_1","dropoff":"DROP_1"}'
```

```json
{"request_id":"req-0e109d77f586","pickup":"PICKUP_1","dropoff":"DROP_1","status":"ACCEPTED","job_id":"JOB-0E109D77","mission_id":"MISSION-MOCK-AB8805F648834D45B72B92B7D74B8944", ...}
```

미션 실행 요청 (`MID`는 `/api/state`에서 읽는다):

```bash
MID=$(curl -s localhost:8000/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["a_state"]["mission_id"])')
curl -s -X POST localhost:8000/api/mission/run -H 'Content-Type: application/json' -d "{\"mission_id\":\"$MID\"}"
```

```json
{"mission_id":"MISSION-MOCK-AB8805F648834D45B72B92B7D74B8944","outcome":"accepted","reason":""}
```

- 등록된 작업이 없거나, 완료된 미션이거나, A가 복구 확인 중이거나, 준비 상태가 정상이 아니면 `"outcome":"rejected"`와 사유가 온다.
  응답을 받지 못하면(504) 같은 요청을 다시 보낸다. 이미 허용된 미션에 다시 보내도 `accepted`다.
- **A 코드(main 92ce3eb)에는 아직 미션 실행 요청이 없다.** 문서 기준으로 MOCK 대역에 넣었고, A 구현이 올라오면 호출 이름·결과 값을 맞춘다.
- E는 주문 1건을 A의 `TaskStep` 두 개로 보낸다: `<job_id>-PICKUP`(수거)과 `<job_id>-DROPOFF`(전달, 수거가 선행 조건).
  A `register_jobs(steps, start_waypoint_id, cost_revision)`과 같은 모양이다.
- 지점은 `config/waypoints.json`의 `HOME`, `PICKUP_1`, `DROP_1` (D 공용 지점 파일 `ros2_ws/src/amr_navigation/config/waypoints.yaml`(main 반영)의 tb3_sandbox 좌표를 화면 표시용으로 복사, P-014).
- `start_waypoint_id`·`cost_revision`은 환경변수 `START_WAYPOINT`(기본 `HOME`)·`COST_REVISION`(기본 `mock-table`)로 준다. 누가 정할지는 A와 합의 전.
- MOCK 미션 ID는 `MISSION-MOCK-<run_id>`이며 `run_id`는 서버 실행마다 새로 만든 전체 UUID(16진수 32자리)다. 같은 DB로
  같은 초에 재시작해도 이전 인계 기록과 겹치지 않게 하기 위함이다. 그래서 아래 명령은 ID를 직접 쓰지 않고 `/api/state`에서 읽는다.

### 3. 인계 요청 1건 보내고 확인

로봇이 수거 지점에 도착해 A가 인계 대기가 될 때까지 기다린다 (MOCK에서 수 초). 현재 미션·목표 단계를 셸 변수로 읽는다.

```bash
read MID STEP WAIT < <(curl -s localhost:8000/api/state | python3 -c 'import sys,json;a=json.load(sys.stdin)["a_state"];g=a["current_goal"] or {};print(a["mission_id"],g.get("step_id"),a["waiting_confirmation"])'); echo $MID $STEP $WAIT
```

```text
MISSION-MOCK-AB8805F648834D45B72B92B7D74B8944 JOB-0E109D77-PICKUP True
```

`WAIT`가 `True`이면 인계를 보낸다. `False`이면 몇 초 뒤 위 명령을 다시 실행한다.

```bash
curl -s -X POST localhost:8000/api/handoffs -H 'Content-Type: application/json' -d "{\"mission_id\":\"$MID\",\"step_id\":\"$STEP\"}"
```

```json
{"event_id":"evt-04e2a36aa6e3","mission_id":"MISSION-MOCK-AB8805F648834D45B72B92B7D74B8944","step_id":"JOB-0E109D77-PICKUP","status":"ACCEPTED","attempts":1, ...}
```

`curl -s localhost:8000/api/handoffs`로 E가 기록한 인계 이력을 확인한다. 화면에서는 오른쪽 위 "현재 미션"의 **인계 완료** 버튼과 오른쪽 아래 "E 발신 기록" 표로 같은 흐름을 볼 수 있다.

### 4. 응답 유실 → 같은 event_id 재전송 → 1회만 반영

전달 지점에서 인계 대기가 되면(3번의 `read` 명령을 다시 실행해 `STEP`이 `...-DROPOFF`, `WAIT`가 `True`인지 확인):

```bash
curl -s -X POST localhost:8000/api/mock/faults -H 'Content-Type: application/json' -d '{"drop_next_ack":true}'
EID=$(curl -s -X POST localhost:8000/api/handoffs -H 'Content-Type: application/json' -d "{\"mission_id\":\"$MID\",\"step_id\":\"$STEP\"}" | python3 -c 'import sys,json;r=json.load(sys.stdin);print(r["status"],file=sys.stderr);print(r["event_id"])'); echo $EID
```

```text
UNCONFIRMED
evt-2b1f30c67c01
```

A 테스트 대역은 인계를 처리했지만 응답이 사라진 상태다(`last_error`: `[MOCK] 응답 유실 (A는 이미 처리함)`).
A 상태에서는 이 단계가 이미 완료로 보이므로, E가 **자동 동기화**로 같은 event_id를 다시 보내 A의 결과를 받아온다 (E 가이드 2, 1초 안팎):

```bash
sleep 1; curl -s localhost:8000/api/handoffs | python3 -c 'import sys,json;r=json.load(sys.stdin)[0];print(r["event_id"],r["status"],r["attempts"])'
```

```text
evt-2b1f30c67c01 ACCEPTED 2
```

- 같은 단계로 **새 인계를 누르면 409**로 막힌다 (새 event_id로 중복 반영되는 것을 방지)
- 작업자가 직접 **같은 event_id로 재전송**해도 된다. 이미 결과가 있으면 다시 보내지 않고 그 결과를 돌려준다:

```bash
curl -s -X POST localhost:8000/api/handoffs/$EID/retry
```

```json
{"event_id":"evt-2b1f30c67c01", ..., "status":"ACCEPTED","attempts":2,"last_error":null}
```

자동 동기화는 A 상태와 E 기록이 어긋날 때만 보낸다: E는 결과 미확인인데 A는 그 단계를 완료했거나 그 Job을 등록함,
또는 E는 복구 대기인데 A 복구가 끝남. A가 처리하지 않은 미확인 기록(`timeout_next`)은 자동으로 보내지 않고 작업자 재전송을 기다린다.
끄려면 `AUTO_SYNC=0`.

이 예시의 전달 지점(`DROP_1`)은 HOME이 아니므로, 인계 후 A 상태는 `returning_home` → HOME 복귀 목표(`step_id` 없음, 물품 인계 아님) → `completed`가 된다.

A 테스트 대역은 같은 event_id의 첫 결과를 다시 돌려줄 뿐 단계 완료를 다시 반영하지 않는다. 반영 횟수는 자동 테스트
`test_ack_lost_then_retry_with_same_event_applies_once`에서 `handoff_accept_count == 1`로 확인한다.

같은 event_id 전송이 겹쳐(예: 재전송 중 이전 전송의 타임아웃이 늦게 도착) 결과가 엇갈려도, 먼저 저장된 최종 결과
(`ACCEPTED` 등)는 `UNCONFIRMED`로 덮어써지지 않는다 (`test_late_timeout_does_not_overwrite_accepted_handoff`).

## 실행 (MOCK) — 설정

```bash
cd web_control
python3 -m venv .venv && source .venv/bin/activate   # Python 3.12 권장
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
# 브라우저 http://localhost:8000
```

| 환경변수 | 기본값 | 설명 |
|---|---|---|
| `RUN_MODE` | `MOCK` | 현재 MOCK만 지원. SIM/REAL은 E↔A 전송 방식(P-003) 확정 후 |
| `DB_PATH` | `web_control/data/web_control.db` | SQLite 경로 (`*.db`는 git 제외) |
| `MOCK_SPEED` | `4` | MOCK 로봇 이동 배속 |
| `START_WAYPOINT` | `HOME` | A `register_jobs`의 `start_waypoint_id` [A와 합의 전] |
| `AUTO_SYNC` | `1` | A 상태와 어긋난 발신 기록을 같은 ID로 자동 재전송 (`0`이면 끔) |
| `COST_REVISION` | `mock-table` | A `register_jobs`의 `cost_revision` [A와 합의 전] |

MOCK에서는 [A 테스트 대역](app/mock_mission.py)이 A 역할을 한다. A 코드(main `92ce3eb`의 `mission_manager`)에 맞춘 범위:

- 문서 기준(A 코드 미구현): 미션 실행 요청이 허용되기 전에는 `claim_next_goal()`이 목표를 넘기지 않음
- 주문 등록 `register_jobs(steps, start_waypoint_id, cost_revision)`, 입력 오류(중복 job/step, 수거 없는 전달, 시작·비용표 없음, 완료 후 등록) 메시지
- 운행 중 등록 → 계획 `deferred` → 현재 단계 인계 수락 후 적용, 목표 선점 `claim_next_goal()`, HOME 복귀 목표(`step_id` 없음)
- 인계 `handoff(event_id, mission_id, step_id)`의 accepted/rejected/conflict와 같은 키 재전송 시 같은 결과, 빈 ID 거절
- 상태 값: `mission_status`(idle/active/returning_home/completed/recovery_required), `step_status`(pending/executing/waiting_confirmation/completed),
  `job_status`(pending/active/completed), `plan_status`(idle/applied/deferred/…)

방문 순서는 등록 순서 그대로이며(C 미연동, A 테스트의 mock optimizer와 같음) 로봇 이동은 시간 경과로 흉내만 낸다. **운영 코드가 아니다.**
`tests/test_a_parity.py`가 같은 시나리오를 실제 A 코드와 이 대역에 실행해 매 단계 결과·상태가 같은지 비교한다.
(A 코드에 아직 없는 실행 허용 단계는 대조할 때 허용된 상태로 둔다.)

화면 오른쪽 아래 **MOCK 결함 주입** 버튼으로 "다음 응답 유실", "다음 요청 타임아웃", "A 재시작(복구 대기)"을 걸어 재전송 흐름을 직접 볼 수 있다.

## 테스트

```bash
cd web_control
env -u PYTHONPATH python3 -m pytest -q tests
```

> **주의:** ROS2를 `source`한 터미널(`~/.bashrc`에서 source 포함)에서는 `/opt/ros/jazzy`의 `PYTHONPATH`가 venv 안에도 남아
> `launch_testing` pytest 플러그인이 자동으로 로드되고 `ModuleNotFoundError: No module named 'lark'`로 실패한다.
> `env -u PYTHONPATH`를 붙이거나 `unset PYTHONPATH` 후 실행한다.

이 경우 `test_a_parity.py`는 A 코드를 찾지 못해 **건너뛴다(skipped)**. A 대조까지 돌리려면 워크스페이스를 빌드·source하고 venv를 켠 뒤 실행한다:

```bash
source /opt/ros/jazzy/setup.bash && source ../ros2_ws/install/setup.bash && source .venv/bin/activate
python3 -m pytest -q tests      # lark 오류가 나면: pip install lark
```

| 테스트 | 검증 내용 |
|---|---|
| `test_ack_lost_then_retry_with_same_event_applies_once` | 응답 유실 → 같은 event_id 재전송 → 완료 전이 1회 |
| `test_timeout_without_processing_then_retry` | A 미처리 타임아웃 → 재전송 → 수락 |
| `test_same_event_id_on_different_step_is_conflict` | 동일 키·다른 단계 → CONFLICT |
| `test_restart_recovers_unconfirmed_handoff` | E 재시작 후 미확인 인계 자동 재전송 |
| `test_restart_recovers_record_saved_before_send` | 전송 직전 종료(PENDING_SEND) 복구 |
| `test_new_press_while_unconfirmed_is_blocked` / `..._after_accept_is_blocked` | 결과 모름·수락 후 새 이벤트 발급 차단 (중복 반영 방지) |
| `test_handoff_for_step_not_waiting_is_rejected_and_new_event_allowed` | 도착 전 거부 → 실제 인계 후 새 이벤트 |
| `test_a_unreachable_is_unconfirmed_not_failed` | 통신 오류는 실패가 아니라 미확인 |
| `test_job_ack_lost_then_retry_registers_once` | Job 요청 응답 유실 → 같은 request_id 재전송 → A가 `duplicate`로 거절 → 이미 등록된 것으로 `ACCEPTED`, 등록 1회 |
| `test_job_request_is_recorded_and_registered_as_step_pair` | 주문 1건 = A `TaskStep` 수거·전달 2개(전달의 선행 조건 = 수거) |
| `test_new_job_during_mission_is_deferred_until_handoff` | 운행 중 주문 → A 계획 `deferred` → 현재 단계 인계 후 적용 |
| `test_a_input_error_is_rejected_not_retried` | A 입력 오류(ValueError)는 `REJECTED`로 기록, 재전송 대상 아님 |
| `test_empty_ids_are_refused_by_e_before_sending` | 빈 mission_id·step_id 인계는 E가 422로 막음 (A도 거절) |
| `test_last_dropoff_away_from_home_shows_home_return_then_completed` | 마지막 하역이 HOME이 아니면 `returning_home` → HOME 복귀 목표 → `completed`, 완료 후 주문은 거절 |
| `test_registered_jobs_do_not_move_until_run_is_allowed` | 작업 등록만으로는 출발하지 않고, 미션 실행 허용 후 첫 목표가 나감 |
| `test_run_without_jobs_is_rejected` / `test_run_is_rejected_when_not_ready_then_allowed` | 작업 없음·준비 상태 이상이면 실행 거부 |
| `test_repeated_run_request_returns_same_result` | 실행 요청 응답 유실(504) 후 다시 요청 → 같은 결과 |
| `test_run_for_other_mission_or_empty_id` | 다른 미션은 거부, 빈 ID는 E가 422 |
| `test_jobs_added_while_running_continue_without_new_run` | 실행 중 추가 주문은 다시 실행 요청 없이 이어짐 |
| `test_sync_resends_unconfirmed_when_a_already_completed_the_step` | E 미확인·A 완료 → 같은 event_id 자동 재전송으로 `ACCEPTED`, 반영 1회, 맞춘 뒤 재전송 없음 |
| `test_sync_leaves_unprocessed_unconfirmed_alone` | A가 처리하지 않은 미확인 기록은 자동으로 보내지 않음 |
| `test_sync_resends_recovery_pending_after_a_recovers` | A 복구가 끝나면 `RECOVERY_PENDING` 기록을 같은 ID로 자동 재전송 |
| `test_sync_settles_unconfirmed_job_already_registered_in_a` | Job 요청 미확인·A에는 등록됨 → 자동 재전송으로 `ACCEPTED`, 등록 1회 |
| `test_mock_matches_real_a` (3개 시나리오) | **실제 A 코드**(`PersistentMission` + `node.py` 상태 함수)와 MOCK 대역에 같은 순서를 실행해 결과·상태 비교. ROS2 source 시에만 실행 |
| `test_late_timeout_does_not_overwrite_accepted_handoff` | 겹친 전송에서 늦게 끝난 타임아웃·통신 오류가 저장된 `ACCEPTED` 인계를 덮어쓰지 않음 |
| `test_late_timeout_does_not_overwrite_accepted_job` | 같은 경우 Job 요청의 `ACCEPTED`·`job_id`를 덮어쓰지 않음 |
| `test_open_record_still_becomes_unconfirmed` | 아직 결과가 없는 기록은 정상적으로 `UNCONFIRMED`가 됨 |
| `test_mock_restart_with_same_db_gets_new_ids_and_new_handoff_is_not_blocked` | 같은 DB로 MOCK 재시작 시 새 미션·단계 ID가 이전 기록과 겹치지 않고, 새 인계가 409 없이 수락됨 |
| `test_restart_in_same_second_with_same_uuid_prefix_is_not_blocked` | 같은 DB로 같은 초에 재시작하고 UUID 앞 두 자리가 겹쳐도 미션 ID가 달라 새 인계가 409 없이 수락됨 (실행 ID = 전체 UUID) |
| `test_restart_recovers_unconfirmed_job_request` | 재시작 후 미확인 Job 요청 복구 |
| `test_state_view_comes_from_a` | 화면 상태는 A 값 |
| `test_new_handoff_during_a_recovery_is_recovery_pending` | A 복구 확인 중 새 인계 → `RECOVERY_PENDING`, 복구 후 같은 ID 재전송 → 1회 수락 |
| `test_known_event_replays_result_during_a_recovery` | 이미 처리된 이벤트는 A 복구 중에도 저장된 결과 재사용 |
| `test_e_restart_resends_recovery_pending` | E 재시작 시 `RECOVERY_PENDING` 기록도 같은 ID로 복구 |

## API [초안-팀합의필요]

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/config` | 실행 모드, 지점, 지점 출처, 시작 지점·비용표 |
| GET | `/api/state` | A 상태(`a_state`: A 상태 값 그대로) + E 발신 기록 |
| WS | `/ws` | 위 상태를 변경 시 전송 |
| POST | `/api/job-requests` `{pickup, dropoff}` | Job 요청 (새 request_id) |
| POST | `/api/job-requests/{request_id}/retry` | 같은 ID 재전송 |
| POST | `/api/mission/run` `{mission_id}` | 미션 실행 요청 (INTERFACES §11.1). A가 허용 여부 결정. [A 미구현·초안] |
| POST | `/api/handoffs` `{mission_id, step_id}` | 인계 요청 (새 event_id). 같은 단계에 결과 미확인·수락 기록이 있으면 409 |
| POST | `/api/handoffs/{event_id}/retry` | 같은 event_id 재전송 (최종 결과가 있으면 재전송하지 않음) |
| GET | `/api/recovery` | 시작 시 복구한 건수 |
| POST | `/api/mock/faults` | MOCK 전용 결함 주입 (`drop_next_ack`, `timeout_next`, `not_ready`) |

## 구조

```
web_control/
├── app/
│   ├── main.py             # FastAPI, create_app() 팩토리 (테스트에서 임시 DB·대역 주입)
│   ├── db.py               # SQLite: job_requests, handoff_events
│   ├── requests_service.py # 기록 → 전송 → 결과 / 재전송 / 재시작 복구
│   ├── mission_client.py   # E→A 계약 (A register_jobs·handoff 모양의 Protocol). 실제 전송 구현은 P-003 이후
│   └── mock_mission.py     # A 테스트 대역 (MOCK 전용)
├── static/                 # 관제 화면
├── config/waypoints.json   # D 공용 지점 사본 (화면 표시용, P-014)
└── tests/
```

## 미구현·미검증

- **SIM / REAL 미검증.** 실제 A(`mission_manager`)와의 연동은 전송 방식(P-003) 결정 후. 현재 결과는 MOCK뿐.
- Job 요청 재전송: A `register_jobs`에는 요청 ID가 없어서, 같은 주문 재전송 시 A의 `duplicate job or step` 거절을 "이미 등록됨"으로
  해석한다(재전송일 때만). **A와 합의되지 않은 해석**이다 (인계 `event_id`는 ADR-015 확정).
- 미션 실행 요청은 문서(INTERFACES §11.1·§4.6)에만 있고 A 코드에는 아직 없다. 호출 이름·필드·결과 값, 준비 상태 확인 범위는 A와 합의 필요.
- A 상태를 E에 넘기는 묶음(`a_state` 모양: 현재 목표, 남은 순서, 단계·Job 상태)은 E 초안이다. A 노드에는 아직 이런 상태 발행이 없다 (INTERFACES §11.2, P-003).
- A의 인계 원장 영속화는 A가 SQLite 파일 모드로 MOCK 검증(P-012). **A·E 양쪽 프로세스 재시작 후 DB·미션 상태 대조(가이드의 복구 통합)는 전송 방식 결정 후** 진행.
- 카메라·실제 지도·로봇 위치 표시는 토픽·연결 방식(P-003, P-005) 결정 후.
- 가이드 3번(운영 화면의 인계 대기/도착 구분, B·C·D 실험 레코드의 환경·설정 버전별 저장·조회)은 다음 PR.
