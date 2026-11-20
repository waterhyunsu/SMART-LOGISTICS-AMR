# Research 2 MOCK route optimizer

The server uses the fixed directed snapshot `research2-mock-v1` and exhaustive
search for up to eight pending steps. The service distance is the distance from
the requested start to the final pending step. The Python comparison explicitly
adds a return to HOME for its 22 m / 16 m / 16 m fixture.

## Build and run

From the repository root:

```bash
PYTHONPATH=ros2_ws/src/route_optimizer python3 -m unittest discover \
  -s ros2_ws/src/route_optimizer/test -p test_research2_mock.py -v
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to route_optimizer
source install/setup.bash
ros2 run route_optimizer route_optimizer
```

From a second terminal in `ros2_ws`, source Jazzy and `install/setup.bash`.
Call the service with a pickup and its dependent delivery:

```bash
ros2 service call /optimize_route logistics_msgs/srv/OptimizeRoute \
  "{request_id: 1, mission_id: mock, base_version: 0, cost_revision: research2-mock-v1, start_waypoint_id: HOME, pending_steps: [{step_id: S1, job_id: J1, kind: 1, waypoint_id: A}, {step_id: S2, job_id: J1, kind: 2, waypoint_id: C, prerequisite_ids: [S1]}]}"
```

Expected: `success=true`, `ordered_step_ids=[S1, S2]`, and 8 m.

Request a destination that has no entry in the snapshot:

```bash
ros2 service call /optimize_route logistics_msgs/srv/OptimizeRoute \
  "{request_id: 2, mission_id: mock, base_version: 0, cost_revision: research2-mock-v1, start_waypoint_id: HOME, pending_steps: [{step_id: S1, job_id: J1, kind: 1, waypoint_id: A}, {step_id: S2, job_id: J1, kind: 2, waypoint_id: UNREACHABLE, prerequisite_ids: [S1]}]}"
```

Expected: `success=false`, `failure_reason=No reachable route`, and an empty
order. The server echoes request ID, mission ID, and base version. When a
current step is supplied, the caller provides its destination as the start;
the server only computes the remaining steps. Applying the candidate after
handoff is the responsibility of the Mission Manager.
