# 팀원별 병렬 개발

P1 준비 PR을 팀장이 병합한 뒤 아래 명령을 저장소 루트에서 실행한다. 개인 변경이 있다면 먼저 현재 브랜치에서 보존한다. 예시의 마지막 브랜치명은 담당 가이드의 이름으로 바꾼다.

```bash
git fetch origin
git switch main
git pull --ff-only origin main
git switch -c feat/a-mission-manager
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep check --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

| 담당 | 첫 산출물 |
|---|---|
| [윤현수 · A](A_MISSION_MANAGER.md) | 현재 단계 인계 후 계획을 적용하는 Mission Manager |
| [정지호 · B](B_PATH_PLANNER.md) | 동일 출발·목표에서 두 플래너를 비교할 재현 조건 |
| [이현수 · C](C_ROUTE_OPTIMIZER.md) | 방문 순서 서비스와 독립적인 Bringup 점검 기록 |
| [홍근태 · D](D_NAVIGATION.md) | AMCL 준비 판정과 단일 목표 결과 |
| [강연주 · E](E_WEB_DASHBOARD.md) | Job 요청 및 인계 이벤트의 재시도 가능한 저장 |

P0는 패키지 설치·빌드와 메모리 내 계약을 제공한다. 후속 기능 완료 여부는 MOCK, SIM, REAL로 구분해 보고한다. 기본 동적 재계획의 제품 우선순위는 P0로 유지한다. 이번 P1 준비는 팀별 구현을 위한 작업 단계다.

공통 타입·정책 변경은 A가 승인한다. 다른 모듈 변경은 영향받는 담당자와 협의하고 PR에 사유와 테스트를 적는다. main 직접 Push 금지는 팀 운영 규칙이다. [공통 AI 프롬프트](COMMON_AI_PROMPT.md) · [Git 협업 규칙](../../CONTRIBUTING.md) · [인터페이스 계약](../INTERFACES.md)
