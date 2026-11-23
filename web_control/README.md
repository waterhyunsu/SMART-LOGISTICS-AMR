# web_control — E 웹 관제 (FastAPI·SQLite)

E 가이드([E_WEB_DASHBOARD.md](../docs/TEAM_GUIDES/E_WEB_DASHBOARD.md)) 1·2번 범위와 최소 관제 화면.

- **Job 요청과 인계 요청을 분리**해 각각 E가 ID(`request_id`, `event_id`)를 발급하고 **전송 전에 SQLite에 기록**한다.
- A의 결과를 `ACCEPTED` / `REJECTED` / `CONFLICT` / `UNCONFIRMED`(응답 미확인) / `RECOVERY_PENDING`(A 복구 확인 대기)로 구분해 기록한다.
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

화면: 왼쪽 아래 작업 요청에서 수거·전달 지점을 고르고 등록 → **미션 시작**.

curl:

```bash
curl -s -X POST localhost:8000/api/job-requests -H 'Content-Type: application/json' -d '{"pickup":"HOME","dropoff":"A"}'
```

```json
{"request_id":"req-68054ad640c8","pickup":"HOME","dropoff":"A","status":"ACCEPTED","job_id":"JOB-001", ...}
```

```bash
curl -s localhost:8000/api/job-requests
curl -s -X POST localhost:8000/api/mission/start
```

```json
{"outcome":"accepted","mission_id":"MISSION-MOCK-001","reason":""}
```

지점 이름은 `config/waypoints.json`의 `HOME`, `A`~`D` (좌표는 임시, P-014).

### 3. 인계 요청 1건 보내고 확인

로봇이 수거 지점에 도착해 A가 인계 대기가 될 때까지 기다린다 (MOCK에서 수 초).

```bash
curl -s localhost:8000/api/state | python3 -c 'import sys,json;a=json.load(sys.stdin)["a_state"];print(a["mission_id"],a["current_step_id"],a["waiting_confirmation"])'
```

```text
MISSION-MOCK-001 JOB-001-PICKUP True
```

`waiting_confirmation`이 `True`가 되면 그 `mission_id`, `current_step_id`로 인계를 보낸다.

```bash
curl -s -X POST localhost:8000/api/handoffs -H 'Content-Type: application/json' -d '{"mission_id":"MISSION-MOCK-001","step_id":"JOB-001-PICKUP"}'
```

```json
{"event_id":"evt-0e9949f3eb2d","mission_id":"MISSION-MOCK-001","step_id":"JOB-001-PICKUP","status":"ACCEPTED","attempts":1, ...}
```

`curl -s localhost:8000/api/handoffs`로 E가 기록한 인계 이력을 확인한다. 화면에서는 가운데 "현재 미션"의 **인계 완료** 버튼과 오른쪽 아래 발신 기록 표로 같은 흐름을 볼 수 있다.

### 4. 응답 유실 → 같은 event_id 재전송 → 1회만 반영

전달 지점(`JOB-001-DROPOFF`)에서 인계 대기가 되면:

```bash
curl -s -X POST localhost:8000/api/mock/faults -H 'Content-Type: application/json' -d '{"drop_next_ack":true}'
curl -s -X POST localhost:8000/api/handoffs -H 'Content-Type: application/json' -d '{"mission_id":"MISSION-MOCK-001","step_id":"JOB-001-DROPOFF"}'
```

```json
{"event_id":"evt-2b1f30c67c01", ..., "status":"UNCONFIRMED","attempts":1,"last_error":"[MOCK] 응답 유실 (A는 이미 처리함)"}
```

A 테스트 대역은 인계를 처리했지만 응답이 사라진 상태다. 이때:

- 같은 단계로 **새 인계를 누르면 409**로 막힌다 (새 event_id로 중복 반영되는 것을 방지)
- **같은 event_id로 재전송**한다

```bash
curl -s -X POST localhost:8000/api/handoffs/evt-2b1f30c67c01/retry
```

```json
{"event_id":"evt-2b1f30c67c01", ..., "status":"ACCEPTED","attempts":2,"last_error":null}
```

A 테스트 대역은 같은 event_id의 첫 결과를 다시 돌려줄 뿐 단계 완료를 다시 반영하지 않는다. 반영 횟수는 자동 테스트
`test_ack_lost_then_retry_with_same_event_applies_once`에서 `handoff_accept_count == 1`로 확인한다.

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

MOCK에서는 [A 테스트 대역](app/mock_mission.py)이 A 역할을 한다. 인계 처리 의미는
`mission_manager/contract.py`의 `MissionState.handoff()`와 같게 맞췄다. 방문 순서는 등록 순서 그대로이며(C 미연동),
로봇 이동은 시간 경과로 흉내만 낸다. **운영 코드가 아니다.**

화면 오른쪽 아래 **MOCK 결함 주입** 버튼으로 "다음 응답 유실", "다음 요청 타임아웃", "A 재시작(복구 대기)"을 걸어 재전송 흐름을 직접 볼 수 있다.

## 테스트

```bash
cd web_control
env -u PYTHONPATH python3 -m pytest -q tests
```

> **주의:** ROS2를 `source`한 터미널(`~/.bashrc`에서 source 포함)에서는 `/opt/ros/jazzy`의 `PYTHONPATH`가 venv 안에도 남아
> `launch_testing` pytest 플러그인이 자동으로 로드되고 `ModuleNotFoundError: No module named 'lark'`로 실패한다.
> `env -u PYTHONPATH`를 붙이거나 `unset PYTHONPATH` 후 실행한다.

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
| `test_job_ack_lost_then_retry_registers_once` | Job 요청 응답 유실 → 같은 request_id 재전송 → 1회 등록 |
| `test_restart_recovers_unconfirmed_job_request` | 재시작 후 미확인 Job 요청 복구 |
| `test_state_view_comes_from_a` | 화면 상태는 A 값 |
| `test_new_handoff_during_a_recovery_is_recovery_pending` | A 복구 확인 중 새 인계 → `RECOVERY_PENDING`, 복구 후 같은 ID 재전송 → 1회 수락 |
| `test_known_event_replays_result_during_a_recovery` | 이미 처리된 이벤트는 A 복구 중에도 저장된 결과 재사용 |
| `test_e_restart_resends_recovery_pending` | E 재시작 시 `RECOVERY_PENDING` 기록도 같은 ID로 복구 |

## API [초안-팀합의필요]

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/config` | 실행 모드, 지점 |
| GET | `/api/state` | A 상태 + E 발신 기록 |
| WS | `/ws` | 위 상태를 변경 시 전송 |
| POST | `/api/job-requests` `{pickup, dropoff}` | Job 요청 (새 request_id) |
| POST | `/api/job-requests/{request_id}/retry` | 같은 ID 재전송 |
| POST | `/api/mission/start` | 미션 실행 요청 |
| POST | `/api/handoffs` `{mission_id, step_id}` | 인계 요청 (새 event_id). 같은 단계에 결과 미확인·수락 기록이 있으면 409 |
| POST | `/api/handoffs/{event_id}/retry` | 같은 event_id 재전송 (최종 결과가 있으면 재전송하지 않음) |
| GET | `/api/recovery` | 시작 시 복구한 건수 |
| POST | `/api/mock/faults` | MOCK 전용 결함 주입 |

## 구조

```
web_control/
├── app/
│   ├── main.py             # FastAPI, create_app() 팩토리 (테스트에서 임시 DB·대역 주입)
│   ├── db.py               # SQLite: job_requests, handoff_events
│   ├── requests_service.py # 기록 → 전송 → 결과 / 재전송 / 재시작 복구
│   ├── mission_client.py   # E→A 계약 (Protocol). 실제 전송 구현은 P-003 이후
│   └── mock_mission.py     # A 테스트 대역 (MOCK 전용)
├── static/                 # 관제 화면
├── config/waypoints.json   # [미확인] 임시 좌표 (P-014)
└── tests/
```

## 미구현·미검증

- **SIM / REAL 미검증.** 실제 A(`mission_manager`)와의 연동은 전송 방식(P-003) 결정 후. 현재 결과는 MOCK뿐.
- Job 요청의 `request_id` 중복 방지는 A와 **합의되지 않은 초안** (인계 `event_id`는 ADR-015 확정).
- A의 인계 원장 영속화는 A가 SQLite 파일 모드로 MOCK 검증(P-012). **A·E 양쪽 프로세스 재시작 후 DB·미션 상태 대조(가이드의 복구 통합)는 전송 방식 결정 후** 진행.
- 카메라·실제 지도·로봇 위치 표시는 토픽·연결 방식(P-003, P-005) 결정 후.
- 가이드 3번(운영 화면의 인계 대기/도착 구분, B·C·D 실험 레코드의 환경·설정 버전별 저장·조회)은 다음 PR.
