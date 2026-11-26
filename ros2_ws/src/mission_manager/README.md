# Mission Manager 첫 실행

웹에서 공항 SIM 주문을 등록하고 작업자의 적재·하역 확인으로 HOME까지 진행하는 실행기는 [A↔E SIM 연결 계약](WEB_SIM.md)을 참조한다.

두 터미널 모두 빌드한 `ros2_ws`에서 아래 환경을 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

첫 터미널에서 테스트 전용 C 서버를 실행한다. 받은 단계 순서를 그대로 반환하는 대역이며 운영 패키지 실행 파일로 설치하지 않는다.

```bash
python3 src/mission_manager/test/mock_optimizer.py --ros-args -r __ns:=/a_contract_test
```

둘째 터미널에서 Job 한 건을 등록한다. 여러 주문은 `--job`을 반복한다.

```bash
ros2 run mission_manager mission_manager --job J1 A HOME --start HOME --cost-revision r1 \
  --ros-args -r __ns:=/a_contract_test
```

성공하면 A 노드가 `Plan applied, version 1`을 기록한다. Job 인자를 생략하면 빈 미션으로 대기한다. 첫 계획 승인까지가 CLI 실행 범위다. 종료는 Ctrl+C를 사용한다.

- A는 상대 서비스명 `optimize_route`를 사용한다. 서버 탐색과 응답을 합한 제한 시간은 단조 시계로 측정하며 `optimizer_timeout_sec`로 설정한다. 기본 5초는 개발용 설정이다. 실패·시간 초과 시 기존 활성 계획을 보존하며 재계산은 새 요청으로 시작한다.
- 단일 executor에서 `register_jobs`로 신규 주문을 받아 재요청한다. `state.start_next()`로 현재 단계를 선택하고 `arrive(step_id)`, `handoff(event_id, mission_id, step_id)`로 D·E 결과를 주입할 수 있다. 이 Python 경계는 테스트·후속 어댑터용이다.
- 현재 단계가 있으면 유효한 응답을 하나만 보류한다. 인계 후 요청 ID·활성 버전·비용표·남은 단계를 다시 검사해 승인한다. 인계 원장은 기본 `:memory:` 실행에서는 메모리에, `database_path`에 파일을 지정한 실행에서는 SQLite에 저장된다.

워크스페이스에서 `python3 -m pytest -q src/mission_manager/test/test_dds.py`로 DDS 경계 테스트를 실행한다. Nav2 주행과 실제 웹 이벤트 전송은 후속 연동 범위다.

## A 전용 상태 저장과 복구

파일 저장은 노드 파라미터로 활성화한다. 부모 디렉터리는 실행 전에 생성한다.

```bash
ros2 run mission_manager mission_manager --ros-args -p database_path:=/tmp/a-mission.sqlite
```

기본값 `:memory:`는 기존 테스트·일회성 실행용이다. 재시작 복구 검증에는 파일 경로를 지정한다. 하나의 DB는 하나의 미션을 보관하며, 기존 DB가 있으면 저장된 mission_id를 사용한다. 복구 실행에는 최초 등록용 `--job` 인자를 다시 전달하지 않는다.

SQLite의 mission 테이블 한 행에 schema 1 JSON 스냅샷과 revision을 저장한다. 단계의 job_id·waypoint_id·선행 조건, 현재 단계·인계 대기, 완료 집합, 남은 순서, 활성 버전, 비용표 버전, 이벤트 결과 및 요청 발급 번호를 함께 보관한다. 각 전이는 상태 사본에 적용하고 트랜잭션 COMMIT 성공 후 메모리를 교체한다. 실패는 예외로 전달되며 완료 ACK가 반환되지 않는다. 다른 writer가 revision을 변경했다면 재개방 전까지 쓰기를 거부한다. E의 저장소와 테이블을 공유하지 않는다.

진행 단계 복구 시 `state.recovery_required`와 초기 `plan_status=recovery_required`가 설정된다. 저장된 인계 대기 여부는 보존하지만 외부 확인 전 새 전이를 허용하지 않는다. A는 외부 관측 결과를 아래 복구 확인 경계로 받아 반영한다. 기존 이벤트 재전송은 저장된 결과를 반환할 수 있다. 새 노드는 진행 중이던 최적화 요청과 후보를 폐기하고 다음 요청 번호를 계속 발급한다.

`test/test_persistence.py`는 파일 DB 복구, 이벤트 재전송·충돌, 트랜잭션 실패, 중복 writer 및 별도 ROS 노드 프로세스의 강제 종료 후 재실행을 검증한다. 메모리·DB의 원자성은 A 프로세스에 한정하며 실제 웹 ACK 전송 및 하드웨어 재조정은 후속 연동 대상이다.

## D·E 복구 연동 경계

A 노드의 단일 executor에서 아래 Python 메서드를 호출한다. 실제 ROS·웹 전송 방식은 후속 연동에서 결정한다.

1. 복구된 현재 단계가 있으면 A는 `begin_recovery_check()`로 확인 컨텍스트를 만든다. D 어댑터는 `mission_id`, `step_id`, `plan_version`, `waypoint_id`, `attempt_id`가 가리키는 기존 실행을 확인하고 `check_id`를 포함한 같은 컨텍스트로 결과를 돌려준다. `check_id`는 새 확인마다 바뀌며 이전 응답을 구분하는 값이다. 실제 Nav2 Goal ID와의 연결은 D 연동 시 필요하다.
2. D 어댑터는 `handle_recovery_observation(check, observation)`을 호출한다. `ARRIVED`는 작업 단계의 인계 대기를 확인한다. `NOT_ARRIVED`는 인계 대기를 해제하고 기존 목표와 복구 제한을 유지한다. 새 확인에는 새 `check_id`를 사용한다. `UNKNOWN`, `DISCONNECTED`, `TIMEOUT`은 해당 확인을 종료하고 복구 제한을 유지한다. 어댑터는 확인 기한이 만료되면 늦은 결과를 전달하기 전에 `TIMEOUT`을 보고해야 한다. 이전 확인 ID나 다른 대상의 응답은 무시한다.
3. E 어댑터는 같은 조작을 재전송할 때 동일한 세 문자열로 `handoff(event_id, mission_id, step_id)`를 호출한다. 빈 식별자는 입력 오류다. `accepted`, `rejected`, `conflict`가 처리 결과이며, 확정 결과는 DB 재기록 없이 재사용한다. `RecoveryPending`은 아직 처리되지 않은 이벤트가 복구 확인을 기다리는 상태다. 저장 예외나 ACK 유실 시에는 결과를 확정하지 않고 동일한 식별자로 재조회한다.

파일 SQLite와 D·E 대역으로 확인하려면 빌드한 워크스페이스에서 overlay를 source한 뒤 실행한다.

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q src/mission_manager/test/test_recovery_integration.py
```

실제 통합에서는 D의 Goal ID·위치 확인 근거와 기한 처리, E의 HTTP 결과 매핑·ACK 전달과 동일 이벤트 재전송을 연결해 검증해야 한다.

## 미션 실행 오케스트레이션

MOCK 또는 A의 후속 어댑터는 단일 executor에서 다음 순서로 호출한다.

1. `register_jobs`로 작업을 등록한다. A가 기존 `OptimizeRoute` 응답을 검증·승인한다. 주행 중에 수신한 유효 후보는 현재 단계의 인계 완료 경계에서 다시 검증한다.
2. `claim_next_goal()`은 실행할 목표 하나를 SQLite에 기록한 뒤 미션·단계·계획 버전·논리적 Waypoint를 반환한다. 호출자는 받은 목표를 D에 전달한다. `current_goal()`은 이미 선점된 목표의 조회용이다. 복구 제한 중, 목표 진행 중, 재계획 응답 대기 중에는 새 목표가 나오지 않는다.
3. D 대역은 `handle_navigation_observation(goal, NavigationObservation.ARRIVED)`로 일치하는 목표의 도착을 알린다. 중복 도착과 다른 목표의 결과는 상태를 바꾸지 않는다. 불확실한 결과에서는 현재 목표가 유지된다. 복구 제한 중에는 기존 복구 확인 흐름을 사용한다.
4. E 대역은 `handoff(event_id, mission_id, step_id)`로 별도 인계를 확인한다. 동일 ACK 재전송은 기존 결과를 반환한다. 인계가 커밋되면 다음 선점 호출에서 후속 목표를 받을 수 있다.
5. 모든 TaskStep의 인계가 완료되고 HOME이 확인되면 미션 상태가 `completed`가 된다. 마지막 인계가 HOME 하역이면 해당 인계까지 기다린다. 마지막 인계가 다른 지점이면 A가 `step_id=None`인 논리적 HOME 복귀 목표를 별도로 선점한다. 이는 물품 인계 단계가 아니다. 재시작 시 진행 중인 복귀 목표도 확인 전까지 다시 발행하지 않는다.

`mission_status`, `job_status(job_id)`, `step_status(step_id)`는 상태 조회용이다. C의 순서는 물류 TaskStep 목록이므로 별도 HOME 복귀를 표현할 수 없다. 복귀 목표는 A 내부에서 처리하며 공통 서비스는 유지한다. 비용·최적화 관점에서 최종 HOME 구간을 포함하는 방법은 C와 연동 시 확인한다.

SQLite 스냅샷에 HOME 복귀/확인 상태를 추가했다. 이전 schema 1에 이 필드가 없으면 위치를 추정하지 않는다. 이전 데이터에서 모든 인계가 끝났더라도 A는 HOME 확인용 목표를 반환할 수 있다. 재시작 후 현재 목표는 다시 선점할 수 없으며 실제 Nav2 Goal ID와 대조하는 작업이 필요하다. SQLite 선점은 A 내부에서만 중복 발행을 막는다. DB 커밋과 외부 목표 전송 사이의 장애 및 실제 E의 ACK 전달은 D·E 통합 시 검증한다.

Jazzy overlay를 설정한 뒤 워크스페이스에서 MOCK E2E를 실행한다.

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q src/mission_manager/test/test_orchestration.py
```

### 명시적 목표 재시도

D 어댑터는 이전 Goal ID와 실행 컨텍스트를 대조해 기존 실행이 없고 뒤늦게 시작될 가능성도 배제한 후에만 같은 확인 컨텍스트로 `NO_ACTIVE_GOAL`을 보낸다. 미도착이나 확인 시간 초과는 재시도의 근거가 될 수 없다. A가 `approve_goal_retry(check)`를 명시적으로 호출하면 새 실행 시도 ID를 SQLite에 기록한 후 목표를 반환한다. 저장된 인계 대기가 남아 있으면 도착 여부를 먼저 다시 확인해야 한다.

실행 시도 ID는 최초 선점부터 SQLite에 저장된다. 새 시도를 승인하면 이전 시도의 주행 결과와 중복 승인은 반영되지 않는다. 새 확인을 시작하거나 노드를 재시작하면 이전 실행 부재 확인은 다시 받아야 한다. COMMIT 실패 시 기존 시도와 복구 제한이 유지된다. COMMIT 직후 종료하면 다음 노드는 새로 저장된 시도를 다시 확인한다. 시도 ID가 없는 이전 스냅샷은 기존 미션·단계·계획·Waypoint를 바탕으로 D가 이전 실행을 대조해야 하며, ID 누락만으로 실행 부재를 판정할 수 없다.

실제 D 연결에서는 Nav2 Goal ID 매핑, 늦은 외부 전송의 배제, 명시적 승인 후 전송을 검증해야 한다. 위 MOCK 명령에 일반 TaskStep·HOME의 별도 프로세스 종료, 승인 직후 종료, DB 쓰기 실패, 중복 승인과 늦은 응답 검증이 포함된다.

## A ↔ D Nav2 Goal UUID 경계 (MOCK)

새로운 `mission_manager.navigation_boundary.NavigationDispatchBoundary`는 A가 선점한
논리 목표의 `attempt_id`와 Nav2 Goal UUID(소문자 32자리 hex)를 연결한다.
**실제 Nav2 Action Client/서버는 구현하지 않았다.**

1. A에서 `claim_next_goal()`을 호출해 논리 목표를 선점한다.
   `prepare_dispatch(goal)`는 시도 ID와 Nav2 Goal UUID를 기존 SQLite 스냅샷에
   COMMIT한 뒤 `DispatchTicket(goal, nav_goal_id)`을 반환한다. COMMIT에 실패하면
   티켓과 외부 전송은 발생하지 않는다.
2. D 송신기는 `submit_once(ticket, sender)`로 **한 번만 호출**한다.
   콜백 호출은 Nav2 수락·실행 성공의 증거가 아니다. 송신 불명확·응답 유실 시
   동일 목표를 무조건 재발행하지 않고 복구 확인 절차를 사용한다.
3. D는 `observe(ticket, observation)`으로 결과를 전달한다.
   현재 Mission·Step·Plan Version·Attempt ID와 저장된 Nav2 Goal UUID가 모두
   일치해야 도착 결과를 반영한다. A 내부의 기존 테스트용
   `handle_navigation_observation`을 외부 D가 직접 호출해서는 안 된다.
4. 송신 콜백이 예외를 던지면 A는 현재 선점·시도 ID·UUID를 유지하고 복구 제한에 들어간 뒤 원래 예외를 호출자에게 전달한다. 비동기 콜백이 반환된 뒤 수락 확인이 시간 초과되거나 결과가 불확실하면 D 어댑터는 `report_dispatch_uncertain(ticket)`을 호출한다. 정상 콜백 반환은 수락 확인과 별개이며 복구를 시작하지 않는다.
5. 같은 프로세스에서 복구 제한에 들어갔거나 A가 재시작한 경우 `begin_recovery_lookup()`으로 저장된 시도 컨텍스트와
   Nav2 UUID를 D에 제공한다. D는 Nav2 활성 Goal뿐 아니라 아직 송신 가능한
   **지연된 큐·전송 중 요청**도 확인해야 한다. 실행 중인 Goal도 없고 뒤늦은
   송신 가능성도 배제한 경우에만 `RecoveryReport(lookup,
   NavigationObservation.NO_ACTIVE_GOAL, no_pending_dispatch=True)`를 반환한다.
   `NOT_ARRIVED`, `UNKNOWN`, `TIMEOUT`만으로 실행 부재를 확정하지 않는다.
6. A는 해당 보고를 수용한 뒤에도 `approve_retry(lookup,
   explicitly_approved=True)`를 **별도로 호출**해야 한다. 승인 시 새로운 시도
   ID가 SQLite에 저장되고 이전 Nav2 UUID 매핑은 해제된다. 이후 새 시도에 대해
   `prepare_dispatch` → `submit_once` 순서로 D에 전송한다.
   지난 시도의 Goal UUID 또는 실행 시도 ID로 온 늦은 응답은 거부한다.

이전 schema 1 스냅샷에 `nav_goal_id` 필드가 없으면 `None`으로 복구한다.
UUID 누락을 미전송/실행 부재의 근거로 간주하지 않는다. D 어댑터가 모든
가능한 이전 실행·지연 전송을 배제해야 한다. 이 코드는 D가 신뢰 가능한
확인 결과를 제공한다는 계약을 전제로 하며, 자체적으로 Nav2 목표 목록이나
외부 프로세스 송신 큐를 조사하지 않는다.

MOCK 테스트 (Jazzy overlay 적용 후):

```bash
python3 -m pytest -q src/mission_manager/test/test_navigation_boundary.py
```

UUID 저장은 외부 송신 전에 완료된다. 송신 실패 시 기존 저장값을 유지하며, 재시작 시 선점된 목표에서 복구 제한을 복원한다. DB COMMIT과 외부 송신은 별도 단계이므로 재시도 전에 D가 실행 및 지연 송신 부재를 확인해야 한다.

실제 Nav2 UUID/Action API, D의 실행 부재 증빙 및 지연 송신 배제, D 프로세스와 로봇 통신, 실차 주행은 후속 연동 검증 범위다.

## Gate 1 A↔D ROS 2 서비스 검증

`NavigationServices(manager)`는 Mission Manager의 단일 executor에서 기존
`NavigationDispatchBoundary`를 ROS 2의 `dispatch_goal`, `report_goal_event`,
`lookup_goal`에 연결한다. Gate 1 테스트가 이 어댑터를 등록하며 기본 CLI는
계획 승인까지 실행한다.

- 호출자가 `claim_next_goal()`로 선점한 목표를 `dispatch(goal)`에 전달한다.
  A가 SQLite에 UUID를 커밋한 뒤 D에 한 번 송신한다. D의 접수 응답과
  Nav2의 수락 이벤트는 각각 확인한다. 일반 TaskStep은 일치하는 도착
  이벤트를 적용한 뒤 인계 대기가 된다.
- 접수·수락 시간 초과, 실패, 취소, 불확실 응답은 복구 조회 대상으로 둔다.
  `lookup()`은 매번 새 확인 ID를 생성한다. 결과가 불확실하거나 조회가
  실패하면 같은 시도를 유지한다. DDS 탐색에서 D의 두 서비스가 모두
  사라졌을 때도 진행 중인 시도를 복구 제한에 둔다. `NO_ACTIVE_GOAL`과 지연 송신 부재가
  함께 확인되면 `approve_retry(lookup, explicitly_approved=True)`로
  새 시도를 승인할 수 있다. 승인을 받아도 송신은 별도 호출이다.
- D의 접수와 Nav2의 수락을 모두 확인한 목표는 같은 시도·UUID로 `lookup_goal`을
  조회한다. 정상 응답 사이에 1초를 두고, 동시에 한 건만 조회한다. D가 빠르게
  재시작해 DDS 서비스가 계속 보여도 `UNKNOWN`, 응답 불일치 또는 시간 초과가
  발생하면 기존 목표를 복구 제한에 둔다. 이 조회에서 도착이나 재시도 승인을
  적용하지 않는다. 복구에서는 새 확인 ID로 조회하고 기존 승인 절차를 적용한다.
- 테스트용 접수·수락 및 조회 기한은 `timeout_sec`으로 설정한다. 기본값은
  5초다. 실제 실행 상태를 판단할 운영 기한과 프로세스 재시작 후 D의
  Nav2 UUID 조회 방식은 SIM 통합 대상이다.

PR #12의 D 노드와 가짜 Nav2 Action 서버가 포함된 워크스페이스를 빌드하고
`ros2_ws`에서 다음을 실행한다. 테스트가 실제 A 노드와 별도 D·가짜 Nav2
프로세스를 격리된 ROS 네임스페이스에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/mission_manager/test/test_navigation_services.py
```

### Nav2가 늦게 시작되는 경우

D는 송신 전 `NOT_READY`로 거절한 시도와 UUID를 기록한다. 같은 요청이
늦게 도착해도 `DUPLICATE`로 응답한다. Nav2가 준비되고 활성 실행이나
대기 중인 송신이 없으면, D는 일치하는 실행 문맥과 최신 Check ID의
`lookup_goal`에 `REFUSED_BEFORE_SEND`와 `no_pending_dispatch=True`를
반환한다.

| A가 받은 근거 | 복구 처리 |
| --- | --- |
| 현재 시도 ID·UUID에 일치하는 `NOT_READY`와 `STATE_NONE`을 기한 내 받았고, 최신 조회에서 D의 거절 기록과 준비 상태를 확인함 | A가 명시적으로 승인하면 새 시도 ID가 생성된다. 별도로 목표를 제출할 때 새 UUID를 사용한다. |
| 응답 유실·시간 초과, 문맥 불일치 또는 A·D 재시작으로 송신 전 거절 근거를 잃음 | 이 거절 경로에서는 기존 시도와 복구 제한을 유지한다. Nav2가 준비됐다는 사실이나 거절 이유 문자열로 실행 부재를 판정하지 않는다. |

별도 프로세스 테스트는 두 경우와 기존 UUID 재전송 차단을 확인한다.

```bash
python3 -m pytest -q src/mission_manager/test/test_navigation_services.py   -k retry_after_not_ready_then_nav2_start
```

### 공용 PC에서 A↔D 단일 목표 SIM

PR #14의 코드가 포함된 체크아웃에서 Jazzy와 작업 공간을 빌드하고 source한
뒤, 저장소 루트에서 [예제 설정](../../../scripts/a_d_sim.example.json)을 복사해
D와 합의한 값으로 채운다. [실행 스크립트](../../../scripts/verify_a_d_sim.py)는
공용 PC의 SIM 프로세스를 실행하고 결과를 파일에 기록한다.

- `sim_command`에는 공용 PC에서 사용할 Gazebo·Nav2 실행 명령을 인자 배열로
  적는다. `{map_yaml}`은 지정한 지도 YAML 절대 경로로, `{config_dir}`는 설정
  파일의 상위 경로로 치환된다. 명령의 작업 디렉터리도 설정 파일의 상위 경로다.
- `initial_pose`와 `waypoint.pose`는 지도 좌표계의 `[x, y, yaw]`이며 yaw는
  라디안이다. D가 확인한 Waypoint ID를 넣는다. Nav2의 최종 위치 판정에 사용할
  거리 허용치는 `tolerance_m`으로 지정한다.
- `d_command: null`은 PR #14의 D 어댑터를 사용하고 입력 좌표를 해당 프로세스에만
  주입한다. 내장 어댑터는 자신의 Nav2 Action 클라이언트가 서버를 발견한 후
  `dispatch_goal`을 공개한다. D가 별도 실행 명령을 공유하면 인자 배열로 지정할 수 있다. 별도 D
  실행은 같은 Waypoint·SIM 시간·루트 네임스페이스를 사용하고, PR #14의
  `[SEND] <UUID> → waypoint <ID> (x, y, yaw)` 로그 형식을 제공해야 한다.
- 격리된 미사용 `ros_domain_id`를 선택한다. 실행기는 localhost DDS, 빈 정적 피어,
  실행별 Gazebo 파티션을 설정한다. SIM 구성은 루트 네임스페이스의 표준 Nav2
  Action·lifecycle 서비스·센서 토픽을 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/verify_a_d_sim.py /path/to/team-sim.json
python3 scripts/verify_a_d_sim.py /path/to/team-sim.json --run --log-dir /path/to/sim-logs
```

첫 명령은 설정 검사다. `--run` 실행은 센서·AMCL·TF·Nav2 활성 상태를 기다린
뒤, 고정된 수거·전달 계획의 첫 목표를 송신한다. 검증 대상은 D 송신 로그 1건,
A·D·Nav2 UUID 일치, Nav2 `SUCCEEDED`, 지도 좌표의 도착 위치, A의 인계 대기다.
외부 C 서비스 없이 고정 계획을 사용하며 인계 확정이나 다음 목표 송신은 수행하지
않는다. 성공과 실패 모두 `summary.json`, `sim.log`, `d.log` 등에 기록한다.
준비 단계에서 실패하면 시작된 프로세스의 로그만 생긴다.
