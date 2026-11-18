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
- 현재 단계가 있으면 유효한 응답을 하나만 보류한다. 인계 후 요청 ID·활성 버전·비용표·남은 단계를 다시 검사해 승인한다. 인계 원장은 메모리에 있다.

워크스페이스에서 `python3 -m pytest -q src/mission_manager/test/test_dds.py`로 DDS 경계 테스트를 실행한다. Nav2 주행, 실제 웹 이벤트 전송, 재시작 복구는 후속 범위다.
