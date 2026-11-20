# legacy_r1 — 이전 연구 1(A*·Dijkstra) 결과·코드 보관본

> **상태:** 참고 자료(보관). ROS2 **Humble** · Gazebo **Classic 11** 기준이며, 이 저장소의 공식 기준(Jazzy · Gazebo Harmonic)에서는 **[미확인]**.
> **담당:** B — 정지호
> **이관일:** 2026-11-18

이 폴더는 이전 팀 저장소에서 수행한 연구 1(A* vs Dijkstra)의 **Nav2 플래너 결과와 그 측정 코드**를 보관한다.
이전 프로젝트에만 쓰이던 파일(자체 플래너, 로봇 스택 실행 스크립트, 확정되지 않은 결론을 전제로 한 안내 문서)은 뺐다.
이 폴더의 코드는 그대로 실행되지 않는다(아래 4절). Phase 1 작업은 상위 [`experiments/path_planner/README.md`](../README.md)를 따른다.

## 1. 출처

| 항목 | 값 |
| --- | --- |
| 원본 저장소 | `team_project2` (이전 팀 프로젝트) |
| 원본 브랜치 | `research1-astar-dijkstra` |
| 스냅샷 커밋 | `2e0ca5f2e516eeeff3acfdbfb309500e01505369` (2026-11-18 14:22 +0900) |
| 추출 방식 | 위 커밋에서 `git archive`로 바이트 그대로 추출 (커밋하지 않은 진행 중 결과는 제외) |
| 원문 수정 | `experiments/r1/README.md` §6의 로컬 경로 한 줄을 `<이전 저장소 경로>`로 바꿈. 나머지 파일은 원본과 같다 |

원본 저장소의 상대 경로를 이 폴더 아래에 그대로 유지했다(`tools/`, `experiments/r1/`, `worlds/`).

## 2. 구성

| 경로 | 내용 |
| --- | --- |
| [`experiments/r1/README.md`](experiments/r1/README.md) | 원본 연구 보고서: 실험 조건, 측정 방법, 결과(§7), 기술 선정(§8), 한계(§9) |
| `experiments/r1/scenarios.yaml` | 강의실·TurtleBot3·창고 월드의 지점과 시나리오 (R1-A~R1-D2, L-A~L-D2) |
| `experiments/r1/maps/` | `lecture_hall_real`(강의실 복제 월드에서 만든 맵, 현재 비교의 임시 지도), `warehouse_large`(40 × 30 m 창고) |
| `experiments/r1/nav2/` | Nav2 설정(`nav2_r1.yaml`: NavFn `use_astar` true/false 등), BT XML, 런치, NavFn C++ 벤치(`navfn_bench.cpp`), 선정 설정 초안(`planner_server_final.yaml`), 초기 위치 설정(`set_initial_pose.py`) |
| `experiments/r1/results/` | Nav2 플래너 결과 21개 폴더: `trials.csv`(원자료), `summary.md`, `meta.json`, 그림(`fig_*.png`, `paths_*.png`) |
| `tools/r1_bench.py` | 실험 도구 (`nav2plan` / `nav2sim` / `navfncore` / `report` 등) |
| `tools/clean_map.py`, `tools/world_to_map.py`, `tools/make_hall_world.py`, `tools/make_warehouse_world.py` | 맵·월드 생성 도구 |
| `worlds/lecture_hall_real.world`, `worlds/warehouse_large.world` | Gazebo Classic 월드 (Harmonic에서는 그대로 쓸 수 없다) |
| `COLCON_IGNORE` | 저장소 루트에서 `colcon build`를 실행해도 이 폴더를 탐색하지 않게 한다 |

결과 폴더 이름은 `<시각>_<모드>[-<A* 쪽 플래너>]_<월드>` 형식이다. 모드: `nav2plan`(ComputePathToPose 계획),
`navfncore`(NavFn 핵심 계산 C++ 직접 호출), `nav2sim`(Gazebo 주행).

## 3. 뺀 파일

| 뺀 것 | 이유 |
| --- | --- |
| 결과 폴더의 원시 데이터 (`costmaps/*.bin`, `maps/*`, 91개) | 팀 저장소 용량 |
| `PATH_COST_GUIDE.md`, `REAL_ROBOT_TEST.md` | "Dijkstra 선정"을 전제로 C·D에게 안내하는 문서. 이 저장소에서 선정(P-009)은 VERIFY이고, 비용표 기준은 INTERFACES §8~9, 실차 절차는 `docs/SIM_TO_REAL.md`를 따른다 |
| `run_sim.sh`, `nav2/run_nav2_sim.sh`, `nav2/run_nav2_real.sh` | 이전 프로젝트의 `scripts/`, Pi 명령 감시, Gazebo Classic이 있어야 실행된다 |
| 자체 격자 플래너 `ros2_ws/src/warehouse_robot/` (코드·설정·테스트)와 그 결과 3개 (`offline_lecture_hall_real`, `offline_turtlebot3_world`, `sim_lecture_hall_real`) | Nav2가 아닌 이전 프로젝트 자체 플래너다 (원본 보고서 부록 A·B) |

뺀 파일은 원본 저장소의 스냅샷 커밋 `2e0ca5f2`에 남아 있다.
원본 보고서와 코드에는 뺀 파일을 가리키는 링크·문구가 남아 있으며, 그 링크는 이 폴더에서 열리지 않는다.

## 4. 실행할 수 없는 이유와 이 저장소 기준과 다른 점

- `tools/r1_bench.py`는 맨 위에서 자체 플래너(`warehouse_robot`)를 import하므로 이 폴더에서는 `ModuleNotFoundError`로 멈춘다.
  Gazebo·Nav2 모드는 이전 프로젝트의 Docker 이미지(`warehouse-sim`, Humble)와 `scripts/`가 필요하다.
- 측정 방법과 결과 형식을 참고하는 용도다. Jazzy 비교는 상위 폴더의 `scripts/run_compare.py`로 한다.

| 항목 | 이 보관본 | 현재 저장소 기준 |
| --- | --- | --- |
| ROS2 / 시뮬레이터 | Humble, Nav2 1.1.20, Gazebo Classic 11 (`gazebo_msgs` 사용) | Jazzy, Gazebo Harmonic (`ros_gz`) — ADR-014 |
| 실행 모드 표기 | 결과는 Gazebo 시뮬레이션 또는 오프라인 계산 | 새 결과는 MOCK / SIM / REAL로 표기 — ADR-010 |
| 플래너 선정 | 원본 보고서 §8은 NavFn `use_astar: false`(Dijkstra)를 선정 | 전역 플래너 최종 선정 P-009는 **VERIFY**. 보관본의 결론은 근거 자료일 뿐 승인된 결정이 아니다 |
| 역할 표기 | 원본 문서에 "B (경로 탐색·물류 최적화)"로 적힌 곳이 있음 | 물류 방문 순서 최적화·비용표는 C 담당 — ADR-002, ADR-006 |
| 지도·지점 | 강의실 복제 월드에서 만든 이상적인 맵, 임시 지점 | HOME·A~D 실제 Pose와 지도 기준은 C·D 담당 P-014 (VERIFY) |
| `planning_time` | Humble `planner_server`가 결과에 채우지 않아 `navfncore`(C++)로 대신 측정 | Jazzy(Nav2 1.3.14)는 결과에 채운다 |

- 결과 수치는 모두 **Humble 환경의 SIM(Gazebo Classic) 또는 오프라인 계산값**이다. REAL(실차) 결과는 없다.
- `planner_server_final.yaml`은 C·D와 합의된 설정이 아니다.
