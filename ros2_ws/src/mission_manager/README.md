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
