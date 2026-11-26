# A↔E 공항 SIM 연결 계약

첫 연동의 단위는 한 미션, 주문 한 건이다. E의 FastAPI 서버가 HTTP/JSON으로 A에 요청한다. 브라우저는 기존 E 주소와 WebSocket을 사용한다. A는 C에서 방문 순서를 받아 승인하고 D에 목표를 보낸다. 작업자 인계가 도착하면 A의 기존 SQLite 원장에서 결과를 확정한다.

## 실행

공항 SIM 지도와 초기 자세를 준비하고 C와 D를 먼저 시작한다. C와 A에 같은 비용표 파일을 지정하고, D에는 그 지점 파일과 `wait_for_nav2_before_dispatch:=true`를 사용한다. 첫 시연은 HOME에서 시작한다. [공항 SIM 환경과 초기 자세 확인](https://github.com/waterhyunsu/SMART-LOGISTICS-AMR/tree/9751721d79a9fc3baafb358c8534e79e4a068f6f/experiments/airport_sim)

A 게이트웨이의 추가 Ubuntu 패키지는 `python3-aiohttp`다. Jazzy 환경에서 처음 준비하는 PC는 `ros2_ws`에서 `rosdep install --from-paths src --ignore-src -y`를 실행한 뒤 `colcon build --symlink-install`로 빌드한다.

아래 명령은 빌드한 워크스페이스의 `install/setup.bash`를 적용하고, C·D와 같은 격리된 SIM 도메인에서 실행한다. `COST`는 C가 사용 중인 공항 비용표 JSON, `MISSION_DB`는 쓰기 가능한 SQLite 파일 경로다. 새 미션은 새 파일, 재시작은 동일 파일을 지정한다.

```bash
: "${COST:?C가 사용하는 비용표 JSON 경로를 설정하세요}"
: "${MISSION_DB:?이번 미션의 SQLite 파일 경로를 설정하세요}"
ros2 run mission_manager mission_web_gateway --cost-snapshot "$COST" \
  --ros-args -p use_sim_time:=true -p "database_path:=$MISSION_DB"
```

A 게이트웨이가 Mission Manager 인스턴스 하나를 실행한다. 같은 네임스페이스에 기존 A 실험 실행기를 함께 띄우면 두 인스턴스가 D의 이벤트를 나눠 받을 수 있으므로 A는 이 실행기 하나만 사용한다. HTTP 기본 주소는 `http://127.0.0.1:8765`이고 `--port`로 포트를 바꿀 수 있다. E 서버도 같은 PC에서 실행하거나, E 서버 PC에서 A의 로컬 포트로 SSH 터널을 연결한다. 브라우저 PC의 localhost와 구분한다.

## E의 MissionClient가 호출할 API

POST 본문은 `application/json`으로 보낸다. E는 최초 전송 전에 A에서 읽은 `mission_id`를 자신의 요청 기록에 저장하고 재전송에도 동일한 값을 쓴다. E가 좌표·실행 승인·비용표 버전을 추정할 필요는 없다.

| E 동작 | 요청 | 결과 |
|---|---|---|
| 화면 갱신 | `GET /v1/state` | A의 상태 객체를 E의 `a_state`에 넣는다. `mission_id`, `steps`, `jobs`, `order`, `current_goal`, `waiting_confirmation`, `run_authorized`, `plan_status`, `cost_revision`, `recovery_required`를 제공한다. |
| 주문 등록 | `POST /v1/jobs` · `{"mission_id":"…","job_id":"JOB-1","pickup":"S4","dropoff":"G8"}` | `{"outcome":"accepted","mission_id":"…","job_id":"JOB-1"}`. A가 기존 E 형식의 `JOB-1-PICKUP`, `JOB-1-DROPOFF`를 생성한다. 같은 주문 재전송은 같은 응답을 돌려준다. 접수 직후 C의 계산은 비동기로 진행한다. |
| 미션 실행 | `POST /v1/run` · `{"mission_id":"…"}` | `{"outcome":"accepted" 또는 "rejected","reason":"…"}`. C의 계획 승인과 C·D 서비스 준비를 확인한다. `ready`는 서비스 탐색 결과이며 실제 송신 시 D가 Nav2 상태를 다시 검사한다. 최초 계획 계산 중이면 계획 적용 후 다시 실행을 요청한다. |
| 적재·하역 확인 | `POST /v1/handoffs` · `{"mission_id":"…","step_id":"JOB-1-PICKUP","event_id":"…"}` | `{"outcome":"accepted" 또는 "rejected" 또는 "conflict"}`. 도착 전 인계는 거부된다. 재전송에는 같은 세 ID를 사용한다. |

다른 미션 ID 또는 등록된 주문과 다른 내용은 HTTP 409 `error=conflict`, 잘못된 입력은 422다. 중복이라는 문구만으로 접수를 성공 처리하지 않는다. 복구 제한에 걸린 새 요청은 409 `error=recovery_pending`이며, 이미 처리한 인계는 복구 중에도 기존 결과를 받는다. 시간 초과·연결 유실·5xx는 결과 미확인으로 기록하고 같은 요청을 재전송한다. A 재시작 뒤에는 실행 승인을 다시 받아야 하고, 외부 상태 확인이 필요한 목표는 기존 복구 정책을 따른다.

## 첫 주문의 완료 기준

1. E의 SIM 설정에 공항 Waypoint 파일의 ID와 좌표를 읽어 `S4 → G8`을 선택할 수 있게 한다.
2. 등록 후 계획이 적용되어도 `현재 목표 없음`으로 대기한다. 웹에서 미션 실행을 요청한 뒤 S4로 출발한다.
3. `waiting_confirmation=true`이고 현재 단계가 PICKUP이면 적재 확인, DROPOFF이면 하역 확인을 표시한다. 해당 확인 전까지 다음 목표는 발행되지 않는다.
4. G8에서 하역을 확인하면 HOME으로 복귀한다. 별도 HOME 복귀의 `current_goal.step_id`는 null이며 도착 시 미션은 `completed`가 된다. 주문 한 건의 송신 목표는 3개, 인계는 2개다.

E의 운영 어댑터와 UI 설정 반영은 E 담당이다. A 계약 시험은 `python3 -m pytest -q src/mission_manager/test/test_web_gateway.py`로 실행한다(빌드한 `ros2_ws`에서 실행).


## 2026-11-26 검증 범위

Ubuntu 24.04 ARM64 UTM에서 기존 E FastAPI 앱에 시험용 HTTP 어댑터와 공항 지점 설정을 주입했다. 시험 클라이언트가 웹 페이지에서 사용하는 API로 주문·실행·적재·하역을 요청했다. 실제 A·C·D와 Gazebo Nav2가 S4, G8, HOME 목표 각각을 한 번씩 송신·완료했다. S4와 G8에서는 각 2초간 인계 대기를 확인했고, 같은 이벤트 재전송 후에도 A 파일 DB의 수락 기록은 2건이었다. 마지막 상태는 `completed`였다. 연주님이 운영 중인 웹 서버 배포와 사람이 브라우저 버튼으로 누르는 시연은 후속 연동 범위다.
