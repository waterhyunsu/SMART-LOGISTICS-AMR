# 20261119T075917Z_SIM_lecture_hall_v1_r1d

- run_mode: **SIM** (planner-only, 3 candidates: Docker amr-sim:jazzy, map_server + planner_server, blocked maps via map_server/load_map, no Gazebo/AMCL, static TF map to base_footprint)
- config_version: `planner_candidates@46b8b7580ce1` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 장애물 거리는 경로 점(로봇 중심)에서 가장 가까운 점유 칸까지의 거리(m, 칸 단위 정확도)다. 꺾인 각도 합은 경로를 0.10 m 간격으로 다시 찍어 더한 방향 변화(도)다.
- 0.10 m 간격 길이는 같은 방식으로 다시 찍은 경로의 길이다. 경로 길이와의 차이는 0.10 m 보다 잔 지그재그에서 나온 길이다.
- 차단 지도 `R1-D1` (`:blocked` 실행): `lecture_hall_real+R1-D1@c254d94b7898`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0]]
- 차단 지도 `R1-D2` (`:blocked` 실행): `lecture_hall_real+R1-D2@07dc060afa8c`, 상자 [[0.0, 3.3, 0.8, 0.3, 0.0], [2.3, 3.3, 0.8, 0.3, 0.0], [4.6, 3.3, 0.8, 0.3, 0.0]]

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 0.10 m 간격 길이 (m) | 최소 장애물 거리 (m) | 평균 장애물 거리 (m) | 꺾인 각도 합 (도) | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.6895 | 0.0 | 6.5849 | 0.45 | 0.5557 | 384.9 | 0.396 | 0.792 | 12.304 | - |
| R1-D1:fwd | success | Smac2D (smac2d) | 30/30 | 30/30 | 30/30 | 6.5103 | 0.0 | 6.5101 | 0.45 | 0.5491 | 60.1 | 1.466 | 4.052 | 9.2575 | - |
| R1-D1:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.6997 | 0.0 | 6.5641 | 0.45 | 0.556 | 307.5 | 0.338 | 0.538 | 12.231 | - |
| R1-D1:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 6.5734 | 0.0 | 6.5334 | 0.45 | 0.5568 | 245.2 | 0.6785 | 1.003 | 13.1055 | - |
| R1-D1:rev | success | Smac2D (smac2d) | 30/30 | 30/30 | 30/30 | 6.5098 | 0.0 | 6.5096 | 0.45 | 0.5517 | 67.0 | 1.759 | 2.959 | 10.0075 | - |
| R1-D1:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 6.5818 | 0.0 | 6.5495 | 0.45 | 0.5552 | 336.8 | 0.5 | 0.69 | 12.6715 | - |
| R1-D2:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.4395 | 0.0 | 12.2588 | 0.45 | 0.8237 | 701.8 | 0.8725 | 1.346 | 17.3465 | - |
| R1-D2:fwd | success | Smac2D (smac2d) | 30/30 | 30/30 | 30/30 | 12.156 | 0.0 | 12.1548 | 0.45 | 0.7092 | 194.7 | 3.734 | 5.862 | 13.5185 | - |
| R1-D2:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.4477 | 0.0 | 12.2002 | 0.45 | 0.7883 | 480.9 | 0.4955 | 0.771 | 16.4655 | - |
| R1-D2:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 30/30 | 12.2415 | 0.0 | 12.1933 | 0.45 | 0.7965 | 415.5 | 0.7275 | 1.284 | 16.873 | - |
| R1-D2:rev | success | Smac2D (smac2d) | 30/30 | 30/30 | 30/30 | 12.4038 | 0.0 | 12.4023 | 0.45 | 0.6419 | 201.3 | 7.1295 | 10.403 | 15.74 | - |
| R1-D2:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 30/30 | 12.2798 | 0.0 | 12.2305 | 0.45 | 0.7877 | 464.2 | 0.9975 | 1.36 | 16.984 | - |
| R1-D1:fwd:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.935 | 0.0 | 10.7963 | 0.4 | 0.5478 | 536.4 | 0.737 | 1.069 | 23.4955 | - |
| R1-D1:fwd:blocked | success | Smac2D (smac2d) | 30/30 | 30/30 | 0/30 | 10.5959 | 0.0 | 10.5934 | 0.4 | 0.5287 | 279.9 | 2.6675 | 3.857 | 19.249 | - |
| R1-D1:fwd:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 12.0787 | 0.0 | 10.8644 | 0.4 | 0.5491 | 721.0 | 0.4955 | 0.726 | 24.6945 | - |
| R1-D1:rev:blocked | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 0/30 | 10.8513 | 0.0 | 10.7732 | 0.4 | 0.5463 | 592.9 | 0.709 | 1.125 | 23.0385 | - |
| R1-D1:rev:blocked | success | Smac2D (smac2d) | 30/30 | 30/30 | 0/30 | 10.5873 | 0.0 | 10.5844 | 0.4 | 0.5296 | 286.3 | 5.763 | 8.986 | 19.663 | - |
| R1-D1:rev:blocked | success | NavFnAStar (astar) | 30/30 | 30/30 | 0/30 | 10.8343 | 0.0 | 10.7657 | 0.35 | 0.5456 | 563.1 | 0.9465 | 1.375 | 23.4925 | - |
| R1-D2:fwd:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.815 | NO_VALID_PATH |
| R1-D2:fwd:blocked | failure | Smac2D (smac2d) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.8675 | NO_VALID_PATH |
| R1-D2:fwd:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.8505 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.8985 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | Smac2D (smac2d) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 14.084 | NO_VALID_PATH |
| R1-D2:rev:blocked | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.833 | NO_VALID_PATH |
