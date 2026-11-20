# 20261119T012513Z_SIM_lecture_hall_v1_r1d

- run_mode: **SIM** (planner-only: Docker amr-path-planner-bench:jazzy, map_server + planner_server, blocked maps via map_server/load_map, no Gazebo/AMCL, static TF map to base_footprint)
- config_version: `navfn_compare@b6b5e3a1039a` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 차단 지도 `R1-D1` (`:blocked` 실행): `lecture_hall_real+R1-D1@c254d94b7898`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0]]
- 차단 지도 `R1-D2` (`:blocked` 실행): `lecture_hall_real+R1-D2@07dc060afa8c`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0], [2.3, 3.3, 0.8, 0.3, 0.0], [4.6, 3.3, 0.8, 0.3, 0.0]]

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.6895 | 0.0 | 0.481 | 0.645 | 12.8655 | - |
| R1-D1:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.6997 | 0.0 | 0.4375 | 0.711 | 14.765 | - |
| R1-D1:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.5734 | 0.0 | 0.87 | 1.317 | 14.0015 | - |
| R1-D1:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.5818 | 0.0 | 0.6455 | 1.359 | 14.1835 | - |
| R1-D2:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.4395 | 0.0 | 1.263 | 1.802 | 23.574 | - |
| R1-D2:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.4477 | 0.0 | 0.6875 | 1.027 | 23.9525 | - |
| R1-D2:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.2415 | 0.0 | 0.9985 | 1.437 | 19.456 | - |
| R1-D2:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.2798 | 0.0 | 1.2625 | 21.67 | 20.5535 | - |
| R1-D1:fwd:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.935 | 0.0 | 0.832 | 1.501 | 23.8815 | - |
| R1-D1:fwd:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 12.0787 | 0.0 | 0.6395 | 1.119 | 24.09 | - |
| R1-D1:rev:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.8513 | 0.0 | 0.861 | 1.561 | 24.181 | - |
| R1-D1:rev:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 10.8343 | 0.0 | 1.1405 | 1.684 | 21.2855 | - |
| R1-D2:fwd:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | 14.86 | NO_VALID_PATH |
| R1-D2:fwd:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | 15.227 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | 15.7875 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | 15.8055 | NO_VALID_PATH |
