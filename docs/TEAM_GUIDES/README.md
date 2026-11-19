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

| 담당 | 첫 산출물 |
|---|---|
| [윤현수 · A](A_MISSION_MANAGER.md) | 독립 MOCK 완료. D 구현이 올라오면 전송·복구 계약과 별도 프로세스 MOCK 검증 |
| [정지호 · B](B_PATH_PLANNER.md) | 동일 출발·목표에서 두 플래너를 비교할 재현 조건 |
| [이현수 · C](C_ROUTE_OPTIMIZER.md) | 방문 순서 서비스와 독립적인 Bringup 점검 기록 |
| [홍근태 · D](D_NAVIGATION.md) | [A↔D 인수인계](../INTEGRATION_A_D_HANDOFF.md)에 맞춘 지도 없는 별도 프로세스 MOCK |
| [강연주 · E](E_WEB_DASHBOARD.md) | Job 요청 및 인계 이벤트의 재시도 가능한 저장 |

공통 빌드·테스트에는 Jazzy 환경이 필요하다. 지도·실차가 필요한 검증은 담당 가이드의 통합 단계에서 진행한다. 후속 기능 완료 여부는 MOCK, SIM, REAL로 구분해 보고한다. 기본 동적 재계획의 제품 우선순위는 Priority P0로 유지한다. 팀원별 개발은 Phase 1에서 시작한다.

공통 타입·정책 변경은 A가 승인한다. 다른 모듈 변경은 영향받는 담당자와 협의하고 PR에 사유와 테스트를 적는다. main 직접 Push 금지는 팀 운영 규칙이다. [공통 AI 프롬프트](COMMON_AI_PROMPT.md) · [Git 협업 규칙](../../CONTRIBUTING.md) · [인터페이스 계약](../INTERFACES.md)
