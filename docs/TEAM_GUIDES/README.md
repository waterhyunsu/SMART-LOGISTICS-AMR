# 팀원별 병렬 개발

처음에는 [루트 README](../../README.md)의 클론·개발환경·공통 테스트 절차를 따른다. 작업 범위는 아래 담당 가이드와 [공통 AI 프롬프트](COMMON_AI_PROMPT.md)를 함께 확인한다.

새 작업은 최신 `main`에서 분기한다. 개인 변경과 진행 중인 커밋은 현재 브랜치에 보존한다. 아래는 C의 새 작업 예시다.

```bash
git status
git switch main
git pull --ff-only origin main
git switch -c feat/c-route-optimizer
```

이미 원격에 담당 브랜치가 있다면 `git fetch origin` 후 `git switch --track origin/브랜치명`으로 받는다. 로컬 브랜치가 이미 있다면 `git switch 브랜치명`으로 이어간다. 예를 들어 B가 처음 클론했다면 `git switch --track origin/feat/b-path-planner`로 기존 실험 작업을 받는다.

공항 SIM은 13개 지점의 다중 주문을, 강의실 REAL은 5개 지점의 실차·인계를 검증한다. 모두 [MOVEIT 공항 시나리오와 기준점](../MOVEIT_AIRPORT.md)을 읽고 담당 가이드를 적용한다.

| 담당 | 전환 후 확인할 산출물 |
|---|---|
| [윤현수 · A](A_MISSION_MANAGER.md) | 현재 단계 인계 후 새 계획 적용, 반복·복구 시 중복 송신 방지, 전체 SIM 미션 |
| [정지호 · B](B_PATH_PLANNER.md) | 같은 공항 지도·설정에서 A*·Dijkstra 비교 조건과 경로 근거 |
| [이현수 · C](C_ROUTE_OPTIMIZER.md) | 공항과 강의실별 방향별 비용 버전, FIFO·최적화 비교, 독립적인 실차 Bringup |
| [홍근태 · D](D_NAVIGATION.md) | 공항 SIM·강의실 REAL의 Pose·Nav2 준비 상태, [A↔D 실행·복구 계약](../INTEGRATION_A_D_HANDOFF.md) |
| [강연주 · E](E_WEB_DASHBOARD.md) | 웹 MOCK과 실제 A 연결 구분, 접수·인계 상태 및 재전송 결과 |

공통 빌드·테스트에는 Jazzy 환경이 필요하다. 지도·실차가 필요한 검증은 담당 가이드의 통합 단계에서 진행한다. 후속 기능 완료 여부는 MOCK, SIM, REAL로 구분해 보고한다. 기본 동적 재계획의 제품 우선순위는 Priority P0로 유지한다. 팀원별 개발은 Phase 1에서 시작한다.

공통 타입·정책 변경은 A가 승인한다. 다른 모듈 변경은 영향받는 담당자와 협의하고 PR에 사유와 테스트를 적는다. main 직접 Push 금지는 팀 운영 규칙이다. [공통 AI 프롬프트](COMMON_AI_PROMPT.md) · [Git 협업 규칙](../../CONTRIBUTING.md) · [인터페이스 계약](../INTERFACES.md)
