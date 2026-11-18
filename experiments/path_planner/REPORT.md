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

| 실험 ID | 날짜 | 모드 | 반복 | 결과 |
| --- | --- | --- | --- | --- |
| `20261118T061956Z_SIM_lecture_hall_v1` | 2026-11-18 | SIM (planner-only) | 예열 1 + 30 | [summary.md](results/20261118T061956Z_SIM_lecture_hall_v1/summary.md), 요약표는 [README "측정 결과"](README.md#측정-결과--sim-planner-only) |

- 도달 가능한 6개 실행: 두 설정 모두 30/30 성공. 도달 불가 2개 실행: 두 설정 모두 0/30이고 Nav2 오류 코드가 기대와 일치했다.
- 경로 길이: R1-A 같음. R1-B는 A*가 정방향 0.10 m, 역방향 0.003 m 길었다. R1-C는 정방향에서 A*가 8.33 m(약 1.9배) 길었고, 역방향에서는 0.17 m 짧았다.
- 계획 시간 중앙값: 0.33~1.04 ms. 실행마다 어느 쪽이 빠른지 달랐다.
- 그래프: **미작성**.

## 5. 결과 차이와 실패 원인 분석

- R1-C 정방향에서 NavFn A* 경로가 길어진 원인: Jazzy에서는 **미분석**이다.
  이전 Humble 연구([`legacy_r1/experiments/r1/README.md`](legacy_r1/experiments/r1/README.md) §7.3)는 NavFn A* 모드의 구현(부분 포텐셜 + 경사 따라가기)을 원인으로 보았다. 이 설명은 Jazzy에서 **[미확인]**이다.
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
