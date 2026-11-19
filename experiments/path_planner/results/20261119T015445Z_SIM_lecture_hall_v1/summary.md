# 20261119T015445Z_SIM_lecture_hall_v1

- run_mode: **SIM** (planner-only: Docker amr-path-planner-bench:jazzy, map_server + planner_server, no Gazebo/AMCL, static TF map to base_footprint, with path quality metrics)
- config_version: `navfn_compare@b6b5e3a1039a` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1@81133c5442f3`
- repeat 30 (+ warmup 1, excluded) / ROS jazzy / nav2_planner 1.3.14
- planned_distance 는 계획 경로 길이(m)이며 실제 주행 거리가 아니다. planning_time 은 planner_server 가 보고한 값(ms).
- 장애물 거리는 경로 점(로봇 중심)에서 가장 가까운 점유 칸까지의 거리(m, 칸 단위 정확도)다. 꺾인 각도 합은 경로를 0.10 m 간격으로 다시 찍어 더한 방향 변화(도)다.
- 0.10 m 간격 길이는 같은 방식으로 다시 찍은 경로의 길이다. 경로 길이와의 차이는 0.10 m 보다 잔 지그재그에서 나온 길이다.

| 실행 | 기대 | 플래너 | 성공 | 기대 일치 | 상자 자리 통과 | 경로 길이 평균 (m) | 표준편차 | 0.10 m 간격 길이 (m) | 최소 장애물 거리 (m) | 평균 장애물 거리 (m) | 꺾인 각도 합 (도) | 계획 시간 중앙값 (ms) | 최대 | 왕복 중앙값 (ms) | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-A:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9971 | 0.0 | 0.5805 | 1.281 | 14.9565 | - |
| R1-A:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9971 | 0.0 | 0.384 | 0.564 | 14.964 | - |
| R1-A:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9973 | 0.0 | 0.5435 | 0.915 | 15.444 | - |
| R1-A:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 6.6 | 0.0 | 6.6 | 0.85 | 0.9973 | 0.0 | 0.3575 | 0.682 | 14.4345 | - |
| R1-B:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 8.6559 | 0.0 | 8.5069 | 0.4 | 0.5487 | 539.6 | 0.634 | 0.876 | 16.9775 | - |
| R1-B:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.7573 | 0.0 | 8.5064 | 0.4 | 0.5489 | 555.3 | 0.4965 | 0.734 | 16.0285 | - |
| R1-B:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 8.5294 | 0.0 | 8.4712 | 0.4 | 0.5484 | 526.9 | 0.808 | 1.139 | 16.2125 | - |
| R1-B:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.5323 | 0.0 | 8.4756 | 0.4 | 0.5484 | 486.9 | 0.962 | 1.512 | 15.8335 | - |
| R1-C:fwd | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 9.239 | 0.0 | 9.0754 | 0.35 | 0.4776 | 577.3 | 0.978 | 1.288 | 17.0155 | - |
| R1-C:fwd | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 17.5687 | 0.0 | 15.1083 | 0.3808 | 0.564 | 886.5 | 1.126 | 2.032 | 22.726 | - |
| R1-C:rev | success | NavFnDijkstra (dijkstra) | 30/30 | 30/30 | - | 9.1211 | 0.0 | 9.041 | 0.35 | 0.476 | 554.6 | 0.893 | 1.289 | 17.3295 | - |
| R1-C:rev | success | NavFnAStar (astar) | 30/30 | 30/30 | - | 8.9546 | 0.0 | 8.9229 | 0.3808 | 0.4552 | 339.4 | 0.9725 | 1.713 | 17.611 | - |
| FAIL-WALLED:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 10.4145 | NO_VALID_PATH |
| FAIL-WALLED:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 10.998 | NO_VALID_PATH |
| FAIL-OFFMAP:fwd | failure | NavFnDijkstra (dijkstra) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 8.081 | GOAL_OUTSIDE_MAP |
| FAIL-OFFMAP:fwd | failure | NavFnAStar (astar) | 0/30 | 30/30 | - | - | - | - | - | - | - | - | - | 7.2195 | GOAL_OUTSIDE_MAP |
