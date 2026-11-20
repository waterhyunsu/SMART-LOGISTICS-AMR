# 20261120T002730Z_SIM_lecture_hall_v1_r1d

- run_mode: **SIM** (planner-only: Docker amr-path-planner-bench:jazzy, map_server + planner_server, blocked maps via map_server/load_map, no Gazebo/AMCL, static TF map to base_footprint, lifecycle active check before warmup)
- config_version: `navfn_compare@b6b5e3a1039a` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 장애물 거리는 경로 점(로봇 중심)에서 가장 가까운 점유 칸까지의 거리(m, 칸 단위 정확도)다. 꺾인 각도 합은 경로를 0.10 m 간격으로 다시 찍어 더한 방향 변화(도)다.
- 0.10 m 간격 길이는 같은 방식으로 다시 찍은 경로의 길이다. 경로 길이와의 차이는 0.10 m 보다 잔 지그재그에서 나온 길이다.
- 차단 지도 `R1-D1` (`:blocked` 실행): `lecture_hall_real+R1-D1@c254d94b7898`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0]]
- 차단 지도 `R1-D2` (`:blocked` 실행): `lecture_hall_real+R1-D2@07dc060afa8c`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0], [2.3, 3.3, 0.8, 0.3, 0.0], [4.6, 3.3, 0.8, 0.3, 0.0]]

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 0.10 m 간격 길이 (m) | 최소 장애물 거리 (m) | 평균 장애물 거리 (m) | 꺾인 각도 합 (도) | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.6895 | 0.0 | 6.5849 | 0.45 | 0.5557 | 384.9 | 0.375 | 0.684 | 12.115 | - |
| R1-D1:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.6997 | 0.0 | 6.5641 | 0.45 | 0.556 | 307.5 | 0.394 | 0.54 | 11.3805 | - |
| R1-D1:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.5734 | 0.0 | 6.5334 | 0.45 | 0.5568 | 245.2 | 0.711 | 1.02 | 11.0205 | - |
| R1-D1:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.5818 | 0.0 | 6.5495 | 0.45 | 0.5552 | 336.8 | 0.605 | 0.744 | 11.741 | - |
| R1-D2:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.4395 | 0.0 | 12.2588 | 0.45 | 0.8237 | 701.8 | 1.202 | 1.29 | 15.0225 | - |
| R1-D2:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.4477 | 0.0 | 12.2002 | 0.45 | 0.7883 | 480.9 | 0.5135 | 0.778 | 17.4155 | - |
| R1-D2:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.2415 | 0.0 | 12.1933 | 0.45 | 0.7965 | 415.5 | 0.989 | 1.142 | 16.0785 | - |
| R1-D2:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.2798 | 0.0 | 12.2305 | 0.45 | 0.7877 | 464.2 | 1.2515 | 1.426 | 17.816 | - |
| R1-D1:fwd:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.935 | 0.0 | 10.7963 | 0.4 | 0.5478 | 536.4 | 0.9175 | 1.065 | 18.0295 | - |
| R1-D1:fwd:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 12.0787 | 0.0 | 10.8644 | 0.4 | 0.5491 | 721.0 | 0.524 | 0.738 | 18.589 | - |
| R1-D1:rev:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.8513 | 0.0 | 10.7732 | 0.4 | 0.5463 | 592.9 | 0.729 | 1.009 | 18.3645 | - |
| R1-D1:rev:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 10.8343 | 0.0 | 10.7657 | 0.35 | 0.5456 | 563.1 | 0.941 | 1.438 | 18.2825 | - |
| R1-D2:fwd:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.9655 | NO_VALID_PATH |
| R1-D2:fwd:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.7365 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 14.128 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.5195 | NO_VALID_PATH |
