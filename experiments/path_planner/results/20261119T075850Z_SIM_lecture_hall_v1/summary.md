# 20261119T075850Z_SIM_lecture_hall_v1

- run_mode: **SIM** (planner-only, 3 candidates: Docker amr-sim:jazzy, map_server + planner_server, no Gazebo/AMCL, static TF map to base_footprint)
- config_version: `planner_candidates@46b8b7580ce1` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1@81133c5442f3`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 장애물 거리는 경로 점(로봇 중심)에서 가장 가까운 점유 칸까지의 거리(m, 칸 단위 정확도)다. 꺾인 각도 합은 경로를 0.10 m 간격으로 다시 찍어 더한 방향 변화(도)다.
- 0.10 m 간격 길이는 같은 방식으로 다시 찍은 경로의 길이다. 경로 길이와의 차이는 0.10 m 보다 잔 지그재그에서 나온 길이다.

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 0.10 m 간격 길이 (m) | 최소 장애물 거리 (m) | 평균 장애물 거리 (m) | 꺾인 각도 합 (도) | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-A:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9971 | 0.0 | 0.5465 | 0.795 | 19.0385 | - |
| R1-A:fwd | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.8 | 0.994 | 0.0 | 0.31 | 0.499 | 15.8755 | - |
| R1-A:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9971 | 0.0 | 0.3565 | 0.745 | 18.001 | - |
| R1-A:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9973 | 0.0 | 0.507 | 0.731 | 18.8885 | - |
| R1-A:rev | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.8 | 0.994 | 0.0 | 0.393 | 1.859 | 17.8585 | - |
| R1-A:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9973 | 0.0 | 0.367 | 0.589 | 17.911 | - |
| R1-B:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 8.6559 | 0.0 | 8.5069 | 0.4 | 0.5487 | 539.6 | 0.6455 | 0.95 | 20.5415 | - |
| R1-B:fwd | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 8.2955 | 0.0 | 8.2928 | 0.4 | 0.5301 | 281.0 | 2.515 | 4.0 | 16.6905 | - |
| R1-B:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.7573 | 0.0 | 8.5064 | 0.4 | 0.5489 | 555.3 | 0.513 | 0.703 | 20.8085 | - |
| R1-B:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 8.5294 | 0.0 | 8.4712 | 0.4 | 0.5484 | 526.9 | 0.7785 | 1.169 | 20.0375 | - |
| R1-B:rev | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 8.2808 | 0.0 | 8.2779 | 0.4 | 0.5313 | 266.8 | 5.1685 | 8.282 | 17.8205 | - |
| R1-B:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.5323 | 0.0 | 8.4756 | 0.4 | 0.5484 | 486.9 | 0.949 | 1.25 | 20.179 | - |
| R1-C:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 9.239 | 0.0 | 9.0754 | 0.35 | 0.4776 | 577.3 | 0.976 | 1.409 | 21.758 | - |
| R1-C:fwd | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 8.8623 | 0.0 | 8.8604 | 0.35 | 0.4708 | 176.7 | 3.426 | 6.1 | 17.772 | - |
| R1-C:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 17.5687 | 0.0 | 15.1083 | 0.3808 | 0.564 | 886.5 | 1.0985 | 1.653 | 27.8895 | - |
| R1-C:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 9.1211 | 0.0 | 9.041 | 0.35 | 0.476 | 554.6 | 0.89 | 1.666 | 20.4015 | - |
| R1-C:rev | success | Smac2D (smac2d) | 30/30 | 30/30 | - | 8.607 | 0.0 | 8.6053 | 0.35 | 0.4737 | 176.8 | 5.5125 | 8.779 | 17.153 | - |
| R1-C:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.9546 | 0.0 | 8.9229 | 0.3808 | 0.4552 | 339.4 | 1.034 | 1.489 | 21.1295 | - |
| FAIL-WALLED:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 15.0925 | NO_VALID_PATH |
| FAIL-WALLED:fwd | failure | Smac2D (smac2d) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 14.238 | NO_VALID_PATH |
| FAIL-WALLED:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.505 | NO_VALID_PATH |
| FAIL-OFFMAP:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 11.3025 | GOAL_OUTSIDE_MAP |
| FAIL-OFFMAP:fwd | failure | Smac2D (smac2d) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 13.796 | GOAL_OUTSIDE_MAP |
| FAIL-OFFMAP:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 10.2015 | GOAL_OUTSIDE_MAP |
