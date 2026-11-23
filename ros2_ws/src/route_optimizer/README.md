## NavFn Dijkstra 방향별 비용표 검증

Ubuntu ROS 2 Jazzy에서 해상도 0.1m의 4m × 4m 임시 빈 지도를 사용했습니다.
NavFn 플러그인 ID는 `GridBased`이며 `use_astar=false`로 확인했습니다.
`map` 좌표계에서 A `(1.0, 1.0)`와 B `(3.0, 3.0)`를 사용했습니다.

| 방향 | 액션 상태 | 경로 좌표 수 | 계획 경로 길이 |
|---|---|---:|---:|
| A → B | `SUCCEEDED`, `error_code=0` | 55 | 2.828435m |
| B → A | `SUCCEEDED`, `error_code=0` | 55 | 2.828434m |

경로 길이는 `ComputePathToPose`가 반환한 연속 XY 좌표 사이의 거리를
합산해 계산했습니다. 가상 데이터 단위 테스트에서는 경로 실패와 빈 경로를
`reachable=false`, `distance_m=null`로 기록하는 동작을 확인했습니다.
신규 테스트 4개와 기존 회귀 테스트 12개가 Ubuntu에서 통과했습니다.

### 기록된 결과로 비용표 재생성

저장소 루트에서 실행합니다. `nav2_recordings.json`에는 실제 액션 결과에서
추출한 두 방향의 XY 좌표와 지도·설정의 해시가 들어 있습니다.

```bash
PYTHONPATH=ros2_ws/src/route_optimizer python3 -m route_optimizer.cost_snapshot \
  ros2_ws/src/route_optimizer/test/fixtures/navfn_smoke/nav2_recordings.json
```

두 방향 모두 `reachable=true`이며, 위 표와 같은 거리와
`cost_revision`이 출력됩니다. 같은 입력과 방향별 결과에는 같은 버전을
사용합니다. 지도·Waypoint·플래너·글로벌 Costmap 설정 또는 방향별 비용·실패
기록이 바뀌면 비용표를 다시 생성하고 새 버전을 사용합니다.

### 임시 지도에서 Nav2 호출 재현

터미널 1에서 저장소 루트 기준으로 실행합니다. 마지막 명령은 실행 상태로 둡니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
fixture_dir="$PWD/ros2_ws/src/route_optimizer/test/fixtures/navfn_smoke"

ros2 run tf2_ros static_transform_publisher \
  --x 1.0 --y 1.0 --frame-id map --child-frame-id base_link &
ros2 run nav2_map_server map_server --ros-args \
  -p "yaml_filename:=$fixture_dir/test_map.yaml" &
ros2 run nav2_planner planner_server --ros-args \
  --params-file "$fixture_dir/planner.yaml"
```

터미널 2에서 지도 서버와 플래너를 활성화한 뒤 두 방향을 각각 요청합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
ros2 param get /planner_server GridBased.use_astar

plan_pair() {
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{planner_id: GridBased, use_start: true, start: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: $3, y: $4}, orientation: {w: 1.0}}}}"
}

plan_pair 1.0 1.0 3.0 3.0
plan_pair 3.0 3.0 1.0 1.0
```

공식 강의실 지도와 좌표가 확정되면 해당 조건으로 양방향 경로를 다시 수집하고
최종 비용표를 생성합니다. 기존 `OptimizeRoute.srv` 규격은 유지합니다.