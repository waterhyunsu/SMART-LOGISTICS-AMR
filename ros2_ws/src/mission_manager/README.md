# Mission Manager 첫 실행

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

진행 단계 복구 시 `state.recovery_required`와 초기 `plan_status=recovery_required`가 설정된다. 저장된 인계 대기 여부는 보존하지만 외부 확인 전 새 전이를 허용하지 않는다. A 어댑터는 실제/작업자 상태 확인 후 `state.reconcile(step_id, arrived=True 또는 False)`를 호출한다. 이는 물리 상태 확인 자체를 수행하지 않는다. 기존 이벤트 재전송은 저장된 결과를 반환할 수 있다. 새 노드는 진행 중이던 최적화 요청과 후보를 폐기하고 다음 요청 번호를 계속 발급한다.

`test/test_persistence.py`는 파일 DB 복구, 이벤트 재전송·충돌, 트랜잭션 실패, 중복 writer 및 별도 ROS 노드 프로세스의 강제 종료 후 재실행을 검증한다. 메모리·DB의 원자성은 A 프로세스에 한정하며 실제 웹 ACK 전송 및 하드웨어 재조정은 후속 연동 대상이다.

## D·E 복구 연동 경계

A 노드의 단일 executor에서 아래 Python 메서드를 호출한다. 실제 ROS·웹 전송 방식은 후속 연동에서 결정한다.

1. 복구된 현재 단계가 있으면 A는 `begin_recovery_check()`로 확인 컨텍스트를 만든다. D 어댑터는 `mission_id`, `step_id`, `plan_version`, `waypoint_id`가 가리키는 기존 실행을 확인하고 `check_id`를 포함한 같은 컨텍스트로 결과를 돌려준다. `check_id`는 새 확인마다 바뀌며 이전 응답을 구분하는 값이다. 실제 Nav2 Goal ID와의 연결은 D 연동 시 필요하다.
2. D 어댑터는 `handle_recovery_observation(check, observation)`을 호출한다. `ARRIVED`는 인계 대기, `NOT_ARRIVED`는 현재 단계를 유지하면서 인계 대기를 해제한다. 두 결과 모두 도착 여부만 반영하며 인계 완료나 목표 발행은 별도다. `UNKNOWN`, `DISCONNECTED`, `TIMEOUT`은 해당 확인을 종료하고 복구 제한을 유지한다. 어댑터는 확인 기한이 만료되면 늦은 결과를 전달하기 전에 `TIMEOUT`을 보고해야 한다. 이전 확인 ID나 다른 대상의 응답은 무시한다.
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
