# 20261120T010411Z_SIM_lecture_hall_v1_r1d

- run_mode: **SIM** (planner-only: Docker amr-path-planner-bench:jazzy, map_server + planner_server, blocked maps via map_server/load_map, no Gazebo/AMCL, static TF map to base_footprint, lifecycle active check before warmup)
- config_version: `navfn_compare@b6b5e3a1039a` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 차단 지도 `R1-D1` (`:blocked` 실행): `lecture_hall_real+R1-D1@c254d94b7898`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0]]
- 차단 지도 `R1-D2` (`:blocked` 실행): `lecture_hall_real+R1-D2@07dc060afa8c`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0], [2.3, 3.3, 0.8, 0.3, 0.0], [4.6, 3.3, 0.8, 0.3, 0.0]]

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.6895 | 0.0 | 0.424 | 0.744 | 11.006 | - |
| R1-D1:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.6997 | 0.0 | 0.3835 | 0.57 | 10.36 | - |
| R1-D1:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.5734 | 0.0 | 0.6705 | 0.995 | 11.04 | - |
| R1-D1:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.5818 | 0.0 | 0.54 | 0.902 | 10.784 | - |
| R1-D2:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.4395 | 0.0 | 0.9635 | 1.541 | 15.02 | - |
| R1-D2:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.4477 | 0.0 | 0.5595 | 0.898 | 14.876 | - |
| R1-D2:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.2415 | 0.0 | 0.7955 | 1.37 | 14.9075 | - |
| R1-D2:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.2798 | 0.0 | 1.0445 | 1.511 | 15.511 | - |
| R1-D1:fwd:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.935 | 0.0 | 0.7515 | 1.186 | 18.9975 | - |
| R1-D1:fwd:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 12.0787 | 0.0 | 0.5615 | 0.887 | 20.297 | - |
| R1-D1:rev:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.8513 | 0.0 | 0.7525 | 1.296 | 18.806 | - |
| R1-D1:rev:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 10.8343 | 0.0 | 0.9595 | 1.601 | 18.6565 | - |
| R1-D2:fwd:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | 14.277 | NO_VALID_PATH |
| R1-D2:fwd:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | 14.3915 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | 14.2735 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | 13.856 | NO_VALID_PATH |
