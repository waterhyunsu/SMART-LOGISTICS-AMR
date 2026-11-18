# 연구 1 실차 검증 절차 (B + D 담당)

> 기획안 5.6-5 "D 담당과 실차 주행 검증". Gazebo 에서 얻은 결과(NavFn Dijkstra 선정, [README](README.md) 8절)를
> 실제 강의실·로봇에서 **같은 도구·같은 시나리오 유형**으로 확인한다. 결과는 README 7.3 과 선정 근거에 더한다.

## 0. 준비

| 항목 | 내용 |
| --- | --- |
| 로봇 | TurtleBot3 Waffle Pi (LDS-02). 바스켓·카메라 등 **시연 때와 같은 장착 상태** (D 담당) |
| PC | 로봇과 같은 공유기에 붙은 **리눅스 PC** (Ubuntu 22.04 + ROS 2 Humble + Nav2). Windows PC 의 Docker(WSL)는 공유기 네트워크에서 로봇과 DDS 통신이 바로 되지 않아 실차에는 쓰지 않는다 |
| Nav2 패키지 | `ros2 pkg list \| grep -E "nav2_navfn_planner\|nav2_bt_navigator\|dwb_core"` 가 모두 나와야 함. 없으면 `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| 장애물 | 상자 3개: 가로 0.8 m 이상, 높이 0.3 m 이상 (LiDAR 높이 약 0.17 m 보다 높게) |
| 기타 | 줄자, 바닥 표시용 테이프, 충전한 배터리 (시도마다 전압이 `trials.csv` 의 `battery_v` 에 기록됨) |

## 1. 강의실 맵 만들기 (D + B)

1. 기존 흐름으로 SLAM: `scripts/run.sh --robot real --mode slam` → 천천히 강의실 전체 주행 → `scripts/save_map.sh <이름>`
2. 맵 정리: `python3 tools/clean_map.py ~/maps/<이름>.yaml` (결과는 `free_thresh: 0.196`)
3. `experiments/r1/maps/real_classroom.yaml` (+ pgm) 로 복사. Nav2 의 AMCL·costmap 이 이 맵을 같이 쓴다
4. `scripts/stop_ros.sh --all` 로 기존 스택을 끈다 (프로젝트 브리지와 Nav2 를 동시에 띄우지 않음)

## 2. 지점·시나리오 정하기

- 지점 좌표: 로봇을 그 자리에 두고 AMCL 위치를 읽거나 (`ros2 topic echo --once /amcl_pose`), RViz **Publish Point** 로 맵에서 읽는다
- `experiments/r1/scenarios.yaml` 맨 아래 **`real_classroom` 템플릿**의 주석을 풀고 좌표를 채운다. Gazebo 와 같은 유형 4가지:

| ID | 유형 | 실제 강의실에서 고르는 예 |
| --- | --- | --- |
| R1-A | 개방 경로 | 교탁 앞 빈 공간을 가로지름 |
| R1-B | 장애물 우회 | 책상 줄 바로 뒤 → 바로 앞 (직선이 책상 줄에 막힘) |
| R1-C | 좁은 통로·갈림길 | 뒤쪽 통로의 갈림길을 지나 세로 통로 안으로 |
| R1-D1 | 경로 차단 (재계획) | 평소 경로의 통로를 상자로 막음 → 다른 통로로 |
| R1-D2 | 경로 없음 (선택) | 목표로 가는 통로를 모두 막음. 로봇이 오래 헤매므로 시간이 있을 때만 |

- 상자 놓을 자리는 바닥에 테이프로 표시해 매번 같은 곳에 놓는다
- 시나리오 지점 근처 0.35 m 안에 의자·가방이 없게 정리한다 (시연과 같은 배치 유지)

## 3. 실행

```bash
cd ~/webserver   # 저장소 위치
# Pi 에 bringup + 명령 감시(cmd_vel_pc → cmd_vel)를 띄우고 PC 에 Nav2 실행. X Y 방향 = 로봇을 놓은 위치
experiments/r1/nav2/run_nav2_real.sh real_classroom 0.0 0.0 0 --pi <user>@<Pi IP>
# RViz 에서 맵·스캔·로봇 위치가 맞는지 확인 (rviz2 → Nav2 기본 설정 또는 Map/LaserScan/TF 추가)

source scripts/env.sh
python3 tools/r1_bench.py nav2sim --real --world real_classroom --repeat 1 --only R1-A   # 먼저 1회로 확인
python3 tools/r1_bench.py nav2sim --real --world real_classroom --repeat 5                # 본 측정
```

- 도구가 시도마다 멈추고 안내한다: "출발점으로 이동합니다, 주변을 비우세요", "장애물 놓기: 중심 (x, y) 크기 …", "장애물 치우기" → 준비되면 Enter
- 출발점 이동(시도 전)은 기록하지 않는다. A*/Dijkstra 순서는 반복마다 자동으로 바뀐다
- 반복: 조건별 5회 이상, 주요 시나리오는 가능하면 10회 (기획안 12.2)

## 4. 안전

- 시험 중에는 경로에 사람이 들어가지 않는다. 한 명은 로봇 옆에서 전원 스위치를 바로 끌 수 있게 대기
- **Ctrl-C**: 도구가 진행 중인 Nav2 목표를 취소해 로봇이 선다 (Gazebo 에서 확인: 취소 뒤 4초 동안 위치 변화 없음)
- 명령 감시: PC → Pi 명령이 0.5 초 끊기면 Pi 가 로봇을 세운다 (`scripts/pi_cmd_watchdog.py`)
- 시도 하나가 180 초를 넘으면 목표를 취소하고 실패로 기록한다 (`--timeout`)

## 5. Gazebo 와 측정이 다른 점

| 항목 | Gazebo | 실차 |
| --- | --- | --- |
| 로봇 위치·주행 거리 | Gazebo 실제 위치 | AMCL 위치 (Gazebo 에서 같은 주행을 AMCL 로 재면 거리 +2%, 6.23 m vs 6.11 m) |
| 도착 오차 | 실제 위치 기준 | AMCL 기준. 몇 번은 줄자로 실제 위치를 재서 함께 적어 두면 좋다 |
| 주행 시간 | 시뮬레이션 시각 | 벽시계 |
| 장애물 | 자동 생성·삭제 | 사람이 놓고 치움 (도구가 안내) |

## 6. 결과 정리

```bash
python3 tools/r1_bench.py report experiments/r1/results/<시각>_nav2real_real_classroom
```

- README 7.3 에 Gazebo 결과와 나란히 표로 정리 (도달률, 주행 시간·거리, 복구 횟수, 경로 길이)
- 선정이 바뀌면 [`nav2/planner_server_final.yaml`](nav2/planner_server_final.yaml), [`PATH_COST_GUIDE.md`](PATH_COST_GUIDE.md) 를 같이 고친다
