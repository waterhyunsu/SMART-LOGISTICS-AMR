# 20261118T061956Z_SIM_lecture_hall_v1

- run_mode: **SIM** (planner-only: Docker amr-path-planner-bench:jazzy, map_server + planner_server, no Gazebo/AMCL, static TF map->base_footprint)
- config_version: `navfn_compare@b6b5e3a1039a` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1@81133c5442f3`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 경로 길이 평균 (m) | 표준편차 | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-A:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 6.6 | 0.0 | 0.5305 | 0.891 | 13.3385 | - |
| R1-A:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 6.6 | 0.0 | 0.3435 | 0.542 | 12.7525 | - |
| R1-A:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 6.6 | 0.0 | 0.543 | 0.71 | 12.996 | - |
| R1-A:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 6.6 | 0.0 | 0.3345 | 0.552 | 13.0315 | - |
| R1-B:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 8.6559 | 0.0 | 0.614 | 1.005 | 14.326 | - |
| R1-B:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 8.7573 | 0.0 | 0.496 | 0.746 | 14.2285 | - |
| R1-B:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 8.5294 | 0.0 | 0.767 | 1.05 | 14.281 | - |
| R1-B:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 8.5323 | 0.0 | 0.8665 | 1.212 | 13.9845 | - |
| R1-C:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 9.239 | 0.0 | 0.8485 | 1.323 | 15.015 | - |
| R1-C:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | 17.5687 | 0.0 | 1.0385 | 1.53 | 20.2635 | - |
| R1-C:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | 9.1211 | 0.0 | 0.8245 | 1.198 | 14.3305 | - |
| R1-C:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | 8.9546 | 0.0 | 0.9135 | 1.68 | 14.5645 | - |
| FAIL-WALLED:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | 8.7175 | NO_VALID_PATH |
| FAIL-WALLED:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | 8.4455 | NO_VALID_PATH |
| FAIL-OFFMAP:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | 6.4285 | GOAL_OUTSIDE_MAP |
| FAIL-OFFMAP:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | 7.4325 | GOAL_OUTSIDE_MAP |
