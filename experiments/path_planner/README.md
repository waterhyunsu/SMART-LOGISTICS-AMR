# experiments/path_planner — 전역 경로 탐색 비교 (B · 정지호)

> 연구 1: 동일한 지도·Costmap·출발/목표에서 Nav2 전역 플래너(NavFn A*·Dijkstra)를 비교해 플래너 선정 근거를 제공한다.
> 위치 근거: ADR-015 ("B 실험은 `experiments/path_planner/`"). B 전용 ROS2 패키지는 만들지 않는다 (ADR-003, `docs/ARCHITECTURE.md` §5).

## 현재 상태

| 항목 | 상태 |
| --- | --- |
| 이전 연구(Humble) 코드·결과 | [`legacy_r1/`](legacy_r1/README.md)에 이관함. 참고 자료이며 Jazzy에서는 [미확인] |
| Jazzy 비교 조건·설정·실행 스크립트 | **미착수**: 아래 "다음 작업" |
| 측정 결과 (SIM / REAL) | 없음. Jazzy 환경 측정 전 |
| 전역 플래너 최종 선정 (P-009) | VERIFY: B·D 실험 후 A 승인 |

## 다음 작업 — B 가이드 첫 번째 기능

기준: [`docs/TEAM_GUIDES/B_PATH_PLANNER.md`](../../docs/TEAM_GUIDES/B_PATH_PLANNER.md), [`COMMON_AI_PROMPT.md`](../../docs/TEAM_GUIDES/COMMON_AI_PROMPT.md)

- [ ] D와 지도·프레임(`map`)·플래너 실행 환경을 맞춘다. D의 공식 지도가 나오기 전에 쓰는 임시 지도는 `[초안-팀합의필요]`로 표시한다.
- [ ] 고정된 출발/목표 쌍을 정의한다. 방향별로 따로 정의하고(`A → B`, `B → A`, INTERFACES §8.4), 도달 불가 실패 사례를 포함한다.
- [ ] NavFn 설정을 `use_astar: true` / `false` 두 개로 분리한다. 나머지 설정은 같게 두고, 각 설정에 플래너 ID와 설정 버전을 붙인다.
- [ ] `ComputePathToPose`를 `start` + `use_start: true`로 호출해 같은 쌍을 두 설정에서 다시 실행하는 스크립트를 만든다.
- [ ] 결과는 `docs/INTERFACES.md` §4.7 ExperimentResult 항목에 맞춰 CSV·JSON으로 기록한다. 경로 길이(m), 계획 시간, 성공 여부, 실패 사유, `planner_id`, `config_version`, `map_version`, `run_mode`를 포함한다.
- [ ] 완료 확인: 도달 불가 구간이 실패로 남는지, 설정·지도 버전별로 결과를 분리할 수 있는지 본다. 측정값에는 SIM 또는 REAL을 표시하고, 계획 경로 길이와 실제 주행 거리를 구분한다.
- [ ] 실행 환경이 없으면 준비한 설정과 미측정 항목을 구분하고, PR에 재현 명령을 남긴다.

`legacy_r1/`의 시나리오(R1-A~R1-D2), 측정 방법(`r1_bench.py nav2plan`), 결과 형식은 위 작업의 출발점으로 참고할 수 있다.
다만 Humble 전용 부분(Gazebo Classic `gazebo_msgs`, 이전 프로젝트 `scripts/`)은 Jazzy 기준으로 다시 작성한다.

## 작업 규칙 요약

- 브랜치는 최신 `main`에서 만든 `feat/b-path-planner`를 쓴다. PR은 main 대상으로 생성까지만 하고, 병합은 팀장 A가 한다 (`CONTRIBUTING.md`).
- 공통 `.msg`·`.srv`·Topic·Service·Action 변경은 A의 승인이 필요하다. ExperimentResult는 ROS 메시지가 아니라 JSON·CSV·SQLite로 관리한다 (INTERFACES §4.7).
- D의 Nav2 배포 설정(`amr_navigation`)은 D와 협의해서 바꾼다. 비용표 기준은 C에게 플래너 ID와 설정 버전으로 전달한다 (INTERFACES §9).
- 측정하지 않은 수치는 쓰지 않는다. 실패 결과도 기록한다 (ADR-012).
