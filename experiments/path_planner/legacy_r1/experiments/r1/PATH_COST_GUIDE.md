# 계획 경로 비용 계산 기준 (연구 1 → C 담당 전달)

> 기획안 5.6-8 "C 담당과 계획 경로 비용 계산 기준 공유", 5.7 "C 담당에게 제공할 계획 경로 계산 기준".
> 물류 방문 순서 최적화(연구 2)의 **지점 간 거리표**를, 연구 1 에서 확정한 전역 플래너와 같은 기준으로 계산하기 위한 약속.
> 근거 수치는 [README](README.md) 7절 (강의실 복제 월드, Gazebo, 2026-11-17). 실차 검증 후 바뀌면 이 문서를 고친다.

## 1. 한 줄 요약

**Nav2 `ComputePathToPose` 를 확정 플래너(NavFn, `use_astar: false`, `planner_id: GridBased`)로, 등록 지점의 좌표끼리 방향마다 따로 부르고,
돌아온 `nav_msgs/Path` 의 연속 점 사이 거리를 더한 값(m)을 비용으로 쓴다.**

## 2. 계산 방법

| 항목 | 기준 | 이유 / 근거 |
| --- | --- | --- |
| 호출 | 액션 `compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`) | 기획안 6.3 과 같음. 실제 주행 때 쓰는 플래너·costmap 과 같은 경로 |
| 플래너 | `planner_id: "GridBased"` = [`nav2/planner_server_final.yaml`](nav2/planner_server_final.yaml) (NavFn, `use_astar: false`, `tolerance: 0.5`) | A* 모드는 거리표를 왜곡한다 (아래 4절) |
| 출발 | `use_start: true`, `start` = 출발 지점의 **등록 좌표** (frame `map`) | 로봇이 지금 어디 있든 같은 표가 나오게 |
| 목표 | `goal` = 도착 지점의 등록 좌표 (방향은 경로 길이에 영향 없음) | |
| 비용 | `Σ ‖p[i+1] − p[i]‖` (경로 점 사이 유클리드 거리 합, m) | NavFn 경로는 칸 단위보다 촘촘한 점이라 그대로 더하면 됨 |
| 방향 | A→B, B→A **따로** 계산 (5지점이면 20개) | 같은 두 지점도 정/역 길이가 최대 0.14 m 달랐음 (R1-B 0.13 m, R1-C 0.14 m) |
| 실패 | 결과 상태가 SUCCEEDED 가 아니거나 빈 경로 → **도달 불가** (∞) 로 표시하고 그 구간은 최적화에서 뺌 | NavFn 은 경로가 없으면 ABORTED (R1-D2) |
| 끝점 확인 | 경로 마지막 점과 목표의 거리가 0.10 m 보다 크면 경고 | `tolerance: 0.5` 라 목표가 벽·장애물에 붙어 있으면 근처 칸까지의 경로가 돌아옴 → 지점 좌표 다시 확인 |

참고 구현: [`tools/r1_bench.py`](../../tools/r1_bench.py) `Nav2Rig.compute_path()` (액션 호출) / `polyline_length(pts, 0.0)` (길이).

```python
# 최소 예시 (rclpy). start, goal: (x, y) map 좌표
goal_msg = ComputePathToPose.Goal()
goal_msg.start.header.frame_id = goal_msg.goal.header.frame_id = 'map'
goal_msg.start.pose.position.x, goal_msg.start.pose.position.y = start
goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y = goal
goal_msg.start.pose.orientation.w = goal_msg.goal.pose.orientation.w = 1.0
goal_msg.planner_id, goal_msg.use_start = 'GridBased', True
# send_goal_async → get_result_async → result.status == GoalStatus.STATUS_SUCCEEDED 확인
pts = [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses]
cost = sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(pts, pts[1:]))
```

## 3. costmap 상태와 재현성

- global costmap = static(맵) + obstacle(LiDAR) + inflation. 거리표는 **맵 기준**이어야 하므로, 계산 직전에
  `global_costmap/clear_entirely_global_costmap` 를 부르고, 통로에 일시 장애물(사람·상자)이 없을 때 로봇을 HOME 에 세워 두고 계산한다.
- 같은 costmap 에서 Dijkstra 경로 길이는 30회 반복에서 표준편차 0 ~ 0.013 m (범위 최대 0.038 m) 로 재현된다.
  LiDAR 장애물층이 조금씩 바뀌는 만큼 수 cm 차이는 날 수 있다.
- 비교: A* 모드는 같은 조건에서 표준편차 최대 1.63 m (범위 6.5 m) — 거리표에 쓰면 계산할 때마다 표가 달라진다.

## 4. A* 모드(`use_astar: true`)를 쓰면 안 되는 이유

| 경우 | Dijkstra | A* 모드 |
| --- | --- | --- |
| 무작위 200쌍 | 기준 | 5% 넘게 긴 쌍 6개, 최대 +1.87 m (+35%) |
| 좁은 통로·갈림길 (R1-C 정방향) | 9.26 m | 16.25 m (+75%) |
| 차단 우회 (R1-D1) | 10.94 m | 12.57 m (+15%), 핵심 계산 기준 +22% |

거리표의 일부 구간만 크게 길어지므로, 최적화가 실제로는 짧은 순서를 버리고 다른 순서를 고를 수 있다.

## 5. 다시 계산해야 할 때 (기획안 6.3)

거리표는 아래 버전과 함께 저장하고, 하나라도 바뀌면 다시 계산한다.

| 항목 | 기록할 값 |
| --- | --- |
| 맵 | 맵 yaml/pgm 파일 이름 + 해시 (예: sha1 앞 6자리) |
| Nav2 설정 | planner_server (플러그인, `use_astar`, `tolerance`) + global_costmap (`robot_radius`, `inflation_radius`, `cost_scaling_factor`) 의 해시 |
| 지점 | 지점 이름·좌표 목록 버전 (HOME, A~D) |
| 계산 시각·로봇 위치 | 계산한 시각, 계산 때 로봇 위치 |

`tools/r1_bench.py` 의 `config_version` (`<커밋>-n<Nav2 설정 해시>-m<맵 해시>`) 과 같은 방식을 써도 된다.

## 6. 거리 → 시간 (참고)

Gazebo 에서 Dijkstra + DWB (Nav2 기본값, 최고 0.26 m/s) 로 주행한 결과 (R1-A~C, 각 5회):

- 실제 주행 거리 = 계획 길이 × 0.92 ~ 0.95 (평균 0.94). DWB 가 모서리를 조금 잘라 가고, 목표 허용 오차가 0.25 m 라서
- 주행 시간 = 계획 길이 1 m 당 3.8 ~ 4.6 s (평균 4.3 s), 출발·도착 제자리 회전 포함

로컬 제어기·속도 설정(연구 3)과 실차에서 달라지므로, 시간 기준 최적화에는 실차 측정값으로 다시 맞춘다.
