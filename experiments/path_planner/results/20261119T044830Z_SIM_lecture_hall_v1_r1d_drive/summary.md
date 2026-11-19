# 20261119T044830Z_SIM_lecture_hall_v1_r1d_drive

- run_mode: **SIM** (Gazebo Harmonic drive: Docker amr-sim:jazzy on WSL2 (GPU D3D12), TurtleBot3 Waffle Pi, Nav2 AMCL + NavFn + DWB, B temporary environment)
- config_version: `nav2_drive@ccfbbd427b1e` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 5 / controller `dwb_core::DWBLocalPlanner` / ROS jazzy / nav2 1.3.14 / Gazebo 8.15.0
- 주행 시간은 목표 전송부터 결과까지의 시뮬레이션 시간(s), 주행 거리는 그동안의 바퀴 오도메트리 경로 길이(m)다. 시간·거리 통계는 도착한 시행만 쓴다.
- 첫 계획 길이는 목표를 보낸 뒤 처음 받은 계획 경로의 길이(m)이며 실제 주행 거리가 아니다.

| 실행 | 기대 | 알고리즘 | 도착 | 기대 일치 | 주행 시간 평균 (s) | 표준편차 | 최소~최대 | 주행 거리 평균 (m) | 표준편차 | 첫 계획 길이 평균 (m) | 복구 평균 | 복구 최대 | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | astar | 5/5 | 5/5 | 22.98 | 0.11 | 22.78~23.1 | 6.521 | 0.044 | 6.761 | 0.0 | 0.0 | - |
| R1-D1:fwd | success | dijkstra | 5/5 | 5/5 | 22.8 | 0.06 | 22.7~22.87 | 6.482 | 0.03 | 6.656 | 0.0 | 0.0 | - |
| R1-D1:rev | success | astar | 5/5 | 5/5 | 43.67 | 9.96 | 31.31~53.69 | 6.846 | 0.082 | 6.603 | 1.4 | 3.0 | - |
| R1-D1:rev | success | dijkstra | 5/5 | 5/5 | 36.53 | 7.24 | 31.48~50.76 | 6.753 | 0.057 | 6.576 | 0.6 | 3.0 | - |
| R1-D1:fwd:blocked | success | astar | 4/5 | 4/5 | 75.01 | 24.58 | 52.11~114.22 | 13.192 | 1.059 | 6.721 | 11.6 | 19.0 | NAV_STATUS_6_CODE_104 |
| R1-D1:fwd:blocked | success | dijkstra | 5/5 | 5/5 | 69.48 | 31.19 | 51.67~131.65 | 13.527 | 1.936 | 6.664 | 2.6 | 7.0 | - |
| R1-D1:rev:blocked | success | astar | 4/5 | 4/5 | 60.53 | 8.33 | 52.75~74.47 | 12.598 | 0.284 | 6.617 | 4.8 | 15.0 | TIMEOUT |
| R1-D1:rev:blocked | success | dijkstra | 5/5 | 5/5 | 64.51 | 16.74 | 54.25~97.65 | 12.887 | 0.48 | 6.585 | 2.4 | 10.0 | - |
