# legacy_r1 — 이전 연구 1(A*·Dijkstra) 코드·결과 이관본

> **상태:** 참고 자료(이관만 함). ROS2 **Humble** · Gazebo **Classic 11** 기준이며, 이 저장소의 공식 기준(Jazzy · Gazebo Harmonic)에서는 **[미확인]**.
> **담당:** B — 정지호
> **이관일:** 2026-11-18

이 폴더는 이전 팀 저장소에서 수행한 연구 1(A* vs Dijkstra) 코드와 실험 결과를 **내용 수정 없이** 옮긴 것이다.
이후 Phase 1 작업은 상위 [`experiments/path_planner/README.md`](../README.md)와 저장소 공통 규칙(`AGENTS.md`, `docs/TEAM_GUIDES/B_PATH_PLANNER.md`)을 따른다.

## 1. 출처

| 항목 | 값 |
| --- | --- |
| 원본 저장소 | `team_project2` (이전 팀 프로젝트) |
| 원본 브랜치 | `research1-astar-dijkstra` |
| 스냅샷 커밋 | `2e0ca5f2e516eeeff3acfdbfb309500e01505369` (2026-11-18 14:22 +0900) |
| 추출 방식 | 위 커밋에서 `git archive`로 바이트 그대로 추출 (커밋하지 않은 진행 중 결과는 제외) |

원본 저장소의 상대 경로를 이 폴더 아래에 그대로 유지했다. 그래서 `tools/r1_bench.py`의 `REPO` 기준 경로(`REPO/experiments/r1/...`,
`REPO/ros2_ws/src/warehouse_robot/...`)가 **코드 수정 없이** 이 폴더(`legacy_r1/`)를 가리킨다.

## 2. 구성

| 경로 | 내용 |
| --- | --- |
| [`experiments/r1/README.md`](experiments/r1/README.md) | 원본 연구 보고서: 실험 조건, 측정 방법, 결과(§7), 기술 선정(§8), 한계(§9) |
| [`experiments/r1/PATH_COST_GUIDE.md`](experiments/r1/PATH_COST_GUIDE.md) | 계획 경로 비용 기준 초안 (원본 작성 당시 기준) |
| [`experiments/r1/REAL_ROBOT_TEST.md`](experiments/r1/REAL_ROBOT_TEST.md) | 실차 검증 절차 초안 (실행 전) |
| `experiments/r1/scenarios.yaml` | 강의실·TurtleBot3·창고 월드의 지점과 시나리오 (R1-A~R1-D2, L-A~L-D2) |
| `experiments/r1/maps/` | `lecture_hall_real`(강의실 복제 월드에서 만든 맵), `warehouse_large`(40 × 30 m 창고) |
| `experiments/r1/nav2/` | Nav2 설정(`nav2_r1.yaml`: NavFn `use_astar` true/false 등), BT XML, 런치, 실행 스크립트, NavFn C++ 벤치(`navfn_bench.cpp`), 선정 설정 초안(`planner_server_final.yaml`) |
| `experiments/r1/results/` | 실험 결과 24개 폴더: `trials.csv`(원자료), `summary.md`, `meta.json`, 그림(`fig_*.png`, `paths_*.png`), costmap 스냅샷(`costmaps/*.bin`), 시나리오별 맵(`maps/*.pgm`) |
| `tools/r1_bench.py` | 실험 도구 (`offline` / `sim` / `nav2plan` / `nav2sim` / `navfncore` / `report`) |
| `tools/clean_map.py`, `tools/world_to_map.py`, `tools/make_hall_world.py`, `tools/make_warehouse_world.py` | 맵·월드 생성 도구 (`r1_bench.py`와 맵 재생성에 필요) |
| `worlds/lecture_hall_real.world`, `worlds/warehouse_large.world` | Gazebo Classic 월드 |
| `ros2_ws/src/warehouse_robot/warehouse_robot/{astar.py, plan_map.py}` | 이전 프로젝트의 자체 격자 플래너 (`r1_bench.py`가 import). **ROS 패키지가 아니다** (`package.xml`·`setup.py` 미포함) |
| `ros2_ws/src/warehouse_robot/config/params.yaml`, `.../test/test_{astar,plan_map}.py` | 위 플래너의 설정과 단위 테스트 |
| `COLCON_IGNORE` | 저장소 루트에서 `colcon build`를 실행해도 이 폴더를 패키지로 탐색하지 않게 한다 |

결과 폴더 이름은 `<시각>_<모드>[-<A* 쪽 플래너>]_<월드>` 형식이다. 모드: `offline`·`sim`(자체 플래너), `nav2plan`(ComputePathToPose 계획),
`navfncore`(NavFn 핵심 계산 C++ 직접 호출), `nav2sim`(Gazebo 주행).

## 3. 이 저장소 기준과 다른 점 — 사용 전 주의

| 항목 | 이관본 | 현재 저장소 기준 |
| --- | --- | --- |
| ROS2 / 시뮬레이터 | Humble, Nav2 1.1.20, Gazebo Classic 11 (`gazebo_msgs` 사용) | Jazzy, Gazebo Harmonic (`ros_gz`) — ADR-014 |
| 실행 모드 표기 | 결과는 Gazebo 시뮬레이션 또는 오프라인 계산 | 새 결과는 MOCK / SIM / REAL로 표기 — ADR-010 |
| 플래너 선정 | 원본 보고서 §8은 NavFn `use_astar: false`(Dijkstra)를 선정 | 전역 플래너 최종 선정 P-009는 **VERIFY** 상태. 이관본 결론은 근거 자료일 뿐 승인된 결정이 아니다 |
| 역할 표기 | 원본 문서에 "B (경로 탐색·물류 최적화)"로 적힌 곳이 있음 | 물류 방문 순서 최적화·비용표는 C 담당 — ADR-002, ADR-006 |
| 지도·지점 | 강의실 복제 월드에서 만든 이상적인 맵, 임시 지점 | HOME·A~D 실제 Pose와 지도 기준은 C·D 담당 P-014 (VERIFY) |
| `planning_time` | Humble `planner_server`가 결과에 채우지 않아 `navfncore`(C++)로 대신 측정 | Jazzy `ComputePathToPose` 결과 필드는 Jazzy 환경에서 다시 확인 필요 |

- 결과 수치는 모두 **Humble 환경의 SIM(Gazebo Classic) 또는 오프라인 계산값**이다. REAL(실차) 결과는 없다.
- `PATH_COST_GUIDE.md`, `planner_server_final.yaml`은 C·D와 합의된 문서·설정이 아니다. 비용표 기준과 Nav2 배포 설정은
  Jazzy 재측정 뒤 C·D와 협의해 `docs/INTERFACES.md` §8~9 절차로 확정한다.

## 4. 원래 실행 환경 (이관하지 않음)

다음은 이전 프로젝트의 로봇 스택·실행 환경이라 옮기지 않았다. Gazebo·Nav2 모드(`sim`, `nav2plan`, `nav2sim`)와 실차 스크립트는
이것들이 없으면 이 폴더만으로 실행되지 않는다.

- Docker 이미지 `warehouse-sim` (원본 `Dockerfile`·`compose.yaml`, ROS2 Humble)
- `scripts/` — `env.sh`, `run.sh`, `start_sim.sh`, `stop_ros.sh`, `save_map.sh`, `pi_cmd_watchdog.py`, `sim_real_io.py`
- `config/amcl_hall.yaml`, 로봇 모델 `worlds/models/turtlebot3_waffle_pi_real`
- 자체 플래너 노드(`astar_planner_node.py`) 등 `warehouse_robot` 패키지의 나머지

원본 실행 명령은 [`experiments/r1/README.md` §6](experiments/r1/README.md#6-실행-방법)에 있다. 경로(`/app/...`)는 원본 컨테이너 기준이다.

## 5. 이 폴더만으로 확인할 수 있는 것

ROS 없이 자체 격자 플래너 단위 테스트를 실행할 수 있다 (`numpy`, `pytest` 필요).

```bash
cd experiments/path_planner/legacy_r1/ros2_ws/src/warehouse_robot
python3 -m pytest test/test_astar.py test/test_plan_map.py -q
```

이 테스트는 **이전 프로젝트 자체 플래너의 MOCK 수준 검증**이며 Nav2 NavFn 비교 결과를 재현하지 않는다.
Jazzy에서의 NavFn A*·Dijkstra 비교는 상위 README의 Phase 1 작업으로 새로 수행한다.
