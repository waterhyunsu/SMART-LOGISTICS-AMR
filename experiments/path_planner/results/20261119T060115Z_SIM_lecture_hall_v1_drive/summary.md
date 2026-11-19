# 20261119T060115Z_SIM_lecture_hall_v1_drive

- run_mode: **SIM** (Gazebo Harmonic drive: Docker amr-sim:jazzy on WSL2 (GPU D3D12), TurtleBot3 Waffle Pi, Nav2 AMCL + NavFn + DWB, B temporary environment)
- config_version: `nav2_drive@ccfbbd427b1e` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1@81133c5442f3`
- repeat 5 / controller `dwb_core::DWBLocalPlanner` / ROS jazzy / nav2 1.3.14 / Gazebo 8.15.0
- 주행 시간은 목표 전송부터 결과까지의 시뮬레이션 시간(s), 주행 거리는 그동안의 바퀴 오도메트리 경로 길이(m)다. 시간·거리 통계는 도착한 시행만 쓴다.
- 첫 계획 길이는 목표를 보낸 뒤 처음 받은 계획 경로의 길이(m)이며 실제 주행 거리가 아니다.

| 실행 | 기대 | 알고리즘 | 도착 | 기대 일치 | 주행 시간 평균 (s) | 표준편차 | 최소~최대 | 주행 거리 평균 (m) | 표준편차 | 첫 계획 길이 평균 (m) | 복구 평균 | 복구 최대 | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-A:fwd | success | astar | 5/5 | 5/5 | 30.3 | 2.12 | 28.82~34.51 | 6.668 | 0.048 | 6.6 | 0.0 | 0.0 | - |
| R1-A:fwd | success | dijkstra | 5/5 | 5/5 | 29.19 | 0.7 | 28.05~30.2 | 6.708 | 0.083 | 6.6 | 0.0 | 0.0 | - |
| R1-A:rev | success | astar | 5/5 | 5/5 | 29.02 | 0.84 | 27.78~30.36 | 6.627 | 0.059 | 6.6 | 0.0 | 0.0 | - |
| R1-A:rev | success | dijkstra | 5/5 | 5/5 | 29.14 | 0.6 | 28.45~30.25 | 6.647 | 0.048 | 6.6 | 0.0 | 0.0 | - |
| R1-B:fwd | success | astar | 5/5 | 5/5 | 45.11 | 9.16 | 37.82~63.19 | 8.924 | 0.274 | 8.77 | 4.0 | 16.0 | - |
| R1-B:fwd | success | dijkstra | 5/5 | 5/5 | 58.89 | 25.07 | 38.93~107.57 | 8.93 | 0.235 | 8.655 | 5.0 | 15.0 | - |
| R1-B:rev | success | astar | 4/5 | 4/5 | 57.34 | 23.87 | 34.93~97.3 | 11.032 | 4.348 | 8.548 | 8.2 | 21.0 | NAV_STATUS_6_CODE_104 |
| R1-B:rev | success | dijkstra | 5/5 | 5/5 | 39.4 | 5.47 | 33.45~48.44 | 8.615 | 0.331 | 8.537 | 3.6 | 10.0 | - |
| R1-C:fwd | success | astar | 5/5 | 5/5 | 68.25 | 24.83 | 47.19~115.88 | 13.735 | 2.191 | 16.839 | 9.6 | 13.0 | - |
| R1-C:fwd | success | dijkstra | 5/5 | 5/5 | 35.18 | 6.79 | 30.31~48.11 | 8.902 | 0.177 | 9.248 | 0.8 | 3.0 | - |
| R1-C:rev | success | astar | 5/5 | 5/5 | 61.14 | 4.48 | 57.8~69.89 | 15.206 | 0.199 | 13.339 | 0.8 | 3.0 | - |
| R1-C:rev | success | dijkstra | 5/5 | 5/5 | 41.71 | 2.04 | 39.81~45.5 | 9.138 | 0.083 | 9.109 | 0.2 | 1.0 | - |
