# 연구 1 기술 선정 보고서 — 전역 경로 탐색 (A* vs Dijkstra)

> **상태:** 작성 중 (초안). 항목 순서는 `docs/reference/ORIGINAL_PROPOSAL.md` §12.4를 따른다.
> **담당:** B — 정지호. 측정한 내용만 쓴다. 측정하지 않은 항목은 "미정" 또는 "미측정"으로 둔다.
> **최종 선정(P-009):** 미정 (VERIFY, B·D 실험 후 A 승인)

## 1. 문제 상황

물류 미션에서 전역 플래너는 두 곳에 쓰인다. D의 주행이 따라갈 경로를 만들고, C가 방문 순서를 정할 때 쓰는 계획 경로 비용표를 만든다.
같은 지도에서 A*와 Dijkstra가 어떤 차이를 보이는지, 실제 물류 운행에는 어느 쪽이 적합한지 실험으로 정해야 한다 (기획서 §5 연구 질문).

## 2. 비교 후보 및 선정 이유

| 후보 | 구현 | 설정 |
| --- | --- | --- |
| A* | Nav2 Jazzy NavFn (`nav2_navfn_planner::NavfnPlanner`) | `use_astar: true` (`NavFnAStar`) |
| Dijkstra | 같은 플러그인 | `use_astar: false` (`NavFnDijkstra`) |

두 후보는 `use_astar` 외 설정이 모두 같다 ([`config/navfn_compare.yaml`](config/navfn_compare.yaml)). 선정 이유: **미정**.

**비교 범위:** 이 비교는 A*·Dijkstra 알고리즘 일반이 아니라 **Nav2 NavFn 플래너의 두 모드**를 비교한다.
NavFn은 정석 A*와 계산 방식이 다르므로, 결과를 "A* 알고리즘의 성능"으로 일반화하지 않는다.

### 2.1. 정석 A*와 NavFn의 차이

NavFn 내용은 Nav2 Jazzy 소스 기준이다:
[`navfn_planner.cpp`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_navfn_planner/src/navfn_planner.cpp),
[`navfn.cpp`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_navfn_planner/src/navfn.cpp).

| 항목 | 정석 A* | NavFn `use_astar: true` | NavFn `use_astar: false` |
| --- | --- | --- | --- |
| 계산하는 것 | 경로 하나 (칸마다 "어디서 왔는지"를 기록) | **포텐셜**: 로봇 출발 칸을 0으로 두고 퍼뜨린 각 칸까지의 누적 비용. Nav2가 출발점과 목표를 바꿔 넘기므로(`setStart(map_goal)`, `setGoal(map_start)`) 로봇 출발 칸이 기준이 된다 | 같음 |
| 칸 비용 | 이동 거리(또는 정한 간선 비용) | `COST_NEUTRAL + COST_FACTOR × costmap 값`. 장애물 근처(inflation)일수록 비싸다 | 같음 |
| 칸 값 갱신 | 이웃 칸 비용 + 간선 비용 | 이웃 두 칸의 값을 이차식으로 보간한다. 격자 방향에 덜 묶인 거리 값이 나온다 | 같음 |
| 탐색 순서 | 정확한 우선순위 큐. `f = g + h`가 가장 작은 칸부터 꺼낸다 | 임계값 구간 3개(`curP`·`nextP`·`overP`)를 쓰는 근사 순서. 같은 구간 안에서는 순서가 없고, 한 칸이 여러 번 갱신될 수 있다 | 같은 구간 방식. 포텐셜 값만으로 순서를 정한다 |
| 휴리스틱 | `h` = 목표까지 추정 거리 | 목표까지 직선거리 × `COST_NEUTRAL`. **구간 배치에만** 쓰고 저장하는 포텐셜에는 넣지 않는다 | 없음 |
| 멈추는 시점 | 목표 칸을 큐에서 **꺼낼 때**. 이 순간 비용이 최소로 확정된다 | 목표 칸에 **처음 값이 생길 때** | 목표 칸에 처음 값이 생길 때 (`calcNavFnDijkstra(..., true)`) |
| 경로 추출 | 부모 기록을 거꾸로 따라간다. 격자 칸을 잇는 지그재그 경로 | 목표에서 포텐셜 기울기를 따라 0.5칸씩 내려간다. 값이 없는 칸이 근처면 주변 8칸 중 가장 낮은 칸으로 이동한다 | 같음 |
| 최적성 | 휴리스틱이 실제 비용을 넘지 않으면 최소 비용 경로가 보장된다 | **보장되지 않는다**. 목표에 처음 닿은 순간의 포텐셜은 최소로 확정된 값이 아닐 수 있고, 주변에 값이 없는 칸이 많다 | 보장되지 않지만, 비용 순서로 고르게 퍼져서 목표에 닿을 때 더 싼 칸들의 값이 대부분 확정되어 있다 |
| 목표에 못 닿을 때 | 실패 | `tolerance`(이 설정에서 0.5 m) 사각형 안에서 값이 있는 칸 중 목표에 가장 가까운 칸으로 대신 계획한다 | 같음 |

- NavFn이 줄이는 것은 경로 **길이가 아니라 비용**이다(장애물 근처 비용 포함). 그래서 측정한 `planned_distance`가 짧다고 해서 더 최적인 것은 아니다.
- Nav2 안에서 정석에 가까운 A*는 Smac Planner 2D(정확한 우선순위 큐, 8방향 격자)다.
  이전 Humble 연구([`legacy_r1/experiments/r1/README.md`](legacy_r1/experiments/r1/README.md) §7.3)에서는 Smac 2D가 NavFn Dijkstra보다 먼 길로 돌지 않았다. Jazzy에서는 **[미확인]**이다.

## 3. 실험 환경과 통제 조건

통제 조건과 재현 명령은 [README "비교 조건"·"실행 방법"](README.md#비교-조건)에 있다.

| 항목 | 값 |
| --- | --- |
| 실행 모드 | SIM (planner-only): map_server + planner_server. Gazebo·AMCL·LiDAR는 쓰지 않았다 |
| 환경 | Docker `amr-path-planner-bench:jazzy` (ROS 2 Jazzy, Nav2 1.3.14, Python 3.12.3), Windows 11 + WSL2 |
| 지도 | `lecture_hall_real@4995e81c1fb1` — **[초안-팀합의필요]** 임시 지도 (legacy 강의실 복제 맵, D 공식 지도 전) |
| 시나리오 | `lecture_hall_v1@81133c5442f3`: R1-A·R1-B·R1-C 정·역방향, FAIL-WALLED, FAIL-OFFMAP |
| 설정 | `navfn_compare@b6b5e3a1039a` |

## 4. 측정 결과 및 그래프

실험마다의 목적·조건·관찰은 [`EXPERIMENT_LOG.md`](EXPERIMENT_LOG.md)에 있다. 이 절에는 보고에 쓰는 측정만 모은다.

| 실험 ID | 날짜 | 모드 | 반복 | 결과 |
| --- | --- | --- | --- | --- |
| `20261118T061956Z_SIM_lecture_hall_v1` | 2026-11-18 | SIM (planner-only) | 예열 1 + 30 | [summary.md](results/20261118T061956Z_SIM_lecture_hall_v1/summary.md), 요약표는 [README "측정 결과"](README.md#측정-결과--sim-planner-only) |

- 도달 가능한 6개 실행: 두 설정 모두 30/30 성공. 도달 불가 2개 실행: 두 설정 모두 0/30이고 Nav2 오류 코드가 기대와 일치했다.
- 경로 길이: R1-A 같음. R1-B는 A*가 정방향 0.10 m, 역방향 0.003 m 길었다. R1-C는 정방향에서 A*가 8.33 m(약 1.9배) 길었고, 역방향에서는 0.17 m 짧았다.
- 계획 시간 중앙값: 0.33~1.04 ms. 실행마다 어느 쪽이 빠른지 달랐다.
- 그래프: **미작성**.

## 5. 결과 차이와 실패 원인 분석

- R1-C 정방향에서 NavFn A* 경로가 길어진 원인: Jazzy에서는 **미분석**이다.
  - 원인 후보(§2.1): A* 모드는 목표 쪽으로 치우쳐 퍼지다가 목표에 처음 닿으면 멈춘다. 그때의 포텐셜은 덜 완성되어 있고, 경로는 그 기울기를 따라 그려진다. 그래서 처음 닿은 우회로를 따라갈 수 있다.
  - 이전 Humble 연구(legacy §7.3)도 같은 원인(부분 포텐셜 + 기울기 따라가기)으로 보았다. Jazzy 결과에서 이 원인을 직접 확인하지는 않았다.
  - 확인 방법 후보: 두 모드의 포텐셜 배열(탐색된 칸과 그 값)과 경로를 함께 그려 비교한다.
- R1-C 역방향에서는 A*가 0.17 m 짧았다. §2.1대로 NavFn은 길이가 아니라 비용을 줄이므로, 이것이 A*가 더 좋다는 뜻은 아니다.
  장애물에 더 붙은 경로일 수 있으나 **[미확인]**이다. 장애물까지의 거리 같은 경로 품질 지표는 아직 기록하지 않았다.
- 도달 불가: 벽으로 막힌 목표는 `NO_VALID_PATH`, 지도 밖 목표는 `GOAL_OUTSIDE_MAP`으로 두 설정 모두 실패했다. 실패 응답에는 `planning_time`이 보고되지 않았다.

## 6. 최종 선정 기술

**미정.**

## 7. 선택 근거

**미정.** 선정에 더 필요한 근거:

- D의 공식 지도와 HOME·A~D 지점에서 방향별 쌍 측정
- 경로 차단(R1-D) 조건
- Gazebo Harmonic 주행 비교(주행 시간·실제 거리·복구 동작, D와 함께)
- 실차(REAL) 검증

## 8. 실제 시스템 적용 결과

**미적용.** D의 Nav2 배포 설정과 C의 비용표에는 아직 반영하지 않았다.

## 9. 한계 및 후속 개선 사항

| 한계 | 후속 |
| --- | --- |
| 임시 지도 한 곳, 정적 지도에서 계획만 비교 | 공식 지도 시나리오 추가, 장애물 조건(R1-D) 추가 |
| 계획 시간은 WSL2 + Docker 환경의 값이다 | 같은 세션 안에서만 비교한다. 실차 PC에서 다시 잰다 |
| 다른 환경(네이티브 Ubuntu, Mac VM)에서 교차 실행하지 않았다 | 팀원 환경에서 같은 명령으로 경로 길이 일치를 확인한다 |
| 주행 결과 없음 | Gazebo Harmonic 주행, 실차 검증 |
| NavFn 두 모드만 비교했다. 정석 A*와의 차이는 소스 분석뿐이다 (§2.1) | 정석에 가까운 A*인 Smac Planner 2D를 같은 조건으로 추가 비교할지 검토한다 |
| 경로 품질은 길이만 기록했다 | 장애물까지 최소 거리와 경로 매끄러움 지표를 추가한다 (legacy 연구가 쓴 지표) |
