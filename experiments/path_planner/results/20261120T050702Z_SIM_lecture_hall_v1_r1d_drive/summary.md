# 20261120T050702Z_SIM_lecture_hall_v1_r1d_drive

- run_mode: **SIM** (Gazebo Harmonic drive, 3 candidates: Docker amr-sim:jazzy on WSL2 (GPU D3D12), TurtleBot3 Waffle Pi, Nav2 AMCL + DWB, B temporary environment)
- config_version: `nav2_drive@6d41440b0a66` / map_version: `lecture_hall_real@4995e81c1fb1` / scenario_version: `lecture_hall_v1_r1d@a663bec775cf`
- repeat 5 / controller `dwb_core::DWBLocalPlanner` / ROS jazzy / nav2 1.3.14 / Gazebo 8.15.0
- 주행 시간은 목표 전송부터 결과까지의 시뮬레이션 시간(s), 주행 거리는 그동안의 바퀴 오도메트리 경로 길이(m)다. 시간·거리 통계는 도착한 시행만 쓴다.
- 첫 계획 길이는 목표를 보낸 뒤 처음 받은 계획 경로의 길이(m)이며 실제 주행 거리가 아니다.
- 도착 오차는 주행이 끝났을 때 Gazebo 의 실제 로봇 위치와 목표 사이 거리(m)다. 도착한 시행만 쓴다.

| 실행 | 기대 | 알고리즘 | 도착 | 기대 일치 | 주행 시간 평균 (s) | 표준편차 | 최소~최대 | 주행 거리 평균 (m) | 표준편차 | 첫 계획 길이 평균 (m) | 복구 평균 | 복구 최대 | 도착 오차 평균 (m) | 최대 | 실패 사유 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1-D1:fwd | success | astar | 5/5 | 5/5 | 23.0 | 0.16 | 22.8~23.19 | 6.55 | 0.055 | 6.719 | 0.0 | 0.0 | 0.051 | 0.167 | - |
| R1-D1:fwd | success | dijkstra | 5/5 | 5/5 | 23.05 | 0.23 | 22.7~23.38 | 6.513 | 0.046 | 6.667 | 0.0 | 0.0 | 0.05 | 0.089 | - |
| R1-D1:fwd | success | smac2d | 5/5 | 5/5 | 23.06 | 0.22 | 22.85~23.38 | 6.511 | 0.045 | 6.552 | 0.0 | 0.0 | 0.059 | 0.092 | - |
| R1-D1:rev | success | astar | 5/5 | 5/5 | 33.89 | 2.99 | 30.66~38.87 | 6.826 | 0.115 | 6.603 | 0.0 | 0.0 | 0.048 | 0.083 | - |
| R1-D1:rev | success | dijkstra | 5/5 | 5/5 | 46.18 | 18.05 | 31.51~78.32 | 6.914 | 0.128 | 6.585 | 1.6 | 5.0 | 0.055 | 0.084 | - |
| R1-D1:rev | success | smac2d | 5/5 | 5/5 | 32.71 | 0.8 | 31.54~33.81 | 6.873 | 0.05 | 6.488 | 0.0 | 0.0 | 0.101 | 0.227 | - |
| R1-D1:fwd:blocked | success | astar | 4/5 | 4/5 | 68.52 | 17.36 | 50.98~86.93 | 12.933 | 0.315 | 6.736 | 9.2 | 23.0 | 0.043 | 0.069 | NAV_STATUS_6_CODE_104 |
| R1-D1:fwd:blocked | success | dijkstra | 4/5 | 4/5 | 68.73 | 20.4 | 50.91~101.07 | 13.081 | 1.082 | 6.66 | 7.4 | 18.0 | 0.053 | 0.072 | NAV_STATUS_6_CODE_208 |
| R1-D1:fwd:blocked | success | smac2d | 4/5 | 4/5 | 54.8 | 2.63 | 50.58~57.51 | 12.483 | 0.176 | 6.562 | 2.6 | 6.0 | 0.051 | 0.085 | NAV_STATUS_6_CODE_0 |
| R1-D1:rev:blocked | success | astar | 5/5 | 5/5 | 60.3 | 6.46 | 53.58~70.68 | 12.705 | 0.769 | 6.635 | 3.6 | 7.0 | 0.128 | 0.227 | - |
| R1-D1:rev:blocked | success | dijkstra | 4/5 | 4/5 | 63.99 | 14.04 | 52.77~87.48 | 12.301 | 0.146 | 6.585 | 8.2 | 24.0 | 0.153 | 0.285 | NAV_STATUS_6_CODE_208 |
| R1-D1:rev:blocked | success | smac2d | 2/5 | 2/5 | 85.52 | 29.36 | 56.16~114.87 | 12.982 | 0.526 | 6.509 | 2.6 | 9.0 | 0.176 | 0.221 | NAV_STATUS_6_CODE_205 |
