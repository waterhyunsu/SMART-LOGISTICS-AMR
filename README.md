# SMART-LOGISTICS AMR

**실주행 데이터 기반 다중 물류 작업 최적화 및 동적 미션 관리 시스템**

ROS2 Jazzy·Nav2 기반 TurtleBot3 Waffle Pi를 활용한 5인 졸업 프로젝트.

- **최종 발표:** 2027-01-15
- **개발 환경:** Ubuntu 24.04 LTS / ROS2 Jazzy / Nav2 / Python 3.12 / FastAPI / SQLite
- **시뮬레이션:** Gazebo Harmonic (Gazebo Sim 8)
- **개발 플랫폼:** macOS Apple Silicon + UTM Ubuntu ARM64 가상머신
- **실차 환경:** TurtleBot3 Waffle Pi 1대, 기존 교육 강의실
- **기획 기준:** 2026-11-17 최종 기획 제안서
- **아키텍처:** v1.0 승인
- **현재 단계:** Phase 1 — 팀원별 담당 기능 개발

> 개발 PC의 ROS2 Jazzy 환경은 기본 동작을 확인했다. TurtleBot3 실차의 Jazzy 호환성 및 통합 주행은 별도 검증이 필요하다.

## 처음 개발을 시작한다면

코드·문서는 Mac에서도 읽을 수 있다. 공통 ROS2 빌드와 테스트는 Ubuntu 24.04의 Bash와 ROS2 Jazzy, 시스템 Python 3.12를 사용한다. 새 Ubuntu 환경에서 먼저 Jazzy와 개발 도구(`ros-dev-tools`, `colcon`, `rosdep`)를 준비한다. [Jazzy 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
git clone https://github.com/waterhyunsu/SMART-LOGISTICS-AMR.git
cd SMART-LOGISTICS-AMR
```

클론은 처음 한 번만 한다. 이미 클론했다면 해당 폴더에서 작업을 보존하고 `git switch main`과 `git pull --ff-only origin main`으로 동기화한다. [팀원별 시작 경로와 브랜치 선택](docs/TEAM_GUIDES/README.md)을 확인한 뒤 아래 공통 빌드를 실행한다.

## 1. 프로젝트 목표

복수의 물류 작업을 등록하고 수거·전달 선후 관계와 Nav2 계획 경로 비용을 기반으로 방문 순서를 최적화한다.

로봇은 계산된 순서에 따라 작업 지점을 방문하며, 작업자가 물품 인계를 확인하면 다음 작업을 수행한다.

신규 주문 발생 시 진행 중인 작업을 보호하면서 남은 방문 순서를 재계산하고 안전한 시점에 적용한다.

**핵심 연구**

- A* vs Dijkstra — 전역 경로 탐색
- FIFO vs Greedy vs 완전탐색 — 물류 방문 순서 최적화
- DWB vs Regulated Pure Pursuit — 로컬 주행 제어

계획 경로 비용과 실제 주행 거리는 구분하여 측정·분석한다.

## 2. 팀 구성

| 담당 | 이름 | 주요 책임 |
|---|---|---|
| A | 윤현수 | Mission Manager, 재계획 적용 정책, 시스템 통합 |
| B | 정지호 | A*·Dijkstra 전역 경로 탐색 |
| C | 이현수 | 방문 순서 최적화, 재계획 계산, 하드웨어·Bringup |
| D | 홍근태 | Nav2·AMCL, DWB·RPP, 자율주행 |
| E | 강연주 | FastAPI, 웹 관제, SQLite, 데이터 시각화 |

최종 기술 설계 및 공통 인터페이스 변경 승인권자는 A 담당 팀장이다.

## 3. 프로젝트 디렉터리

```text
SMART-LOGISTICS-AMR/
├── AGENTS.md
├── README.md
├── CONTRIBUTING.md
├── ros2_ws/
│   └── src/
│       ├── logistics_msgs/      # A 관리
│       ├── mission_manager/     # A
│       ├── route_optimizer/     # C
│       ├── amr_navigation/      # D
│       └── amr_bringup/         # C
├── web_control/                 # E
├── docs/
│   ├── PRD.md
│   ├── ARCHITECTURE.md
│   ├── INTERFACES.md
│   ├── SIM_TO_REAL.md
│   ├── DECISIONS.md
│   ├── INTEGRATION_A_D_HANDOFF.md
│   ├── TEAM_GUIDES/
│   └── reference/
│       ├── ORIGINAL_PROPOSAL.md
│       └── CODEX_FULL_BRIEF.md
├── prompts/
└── scripts/
```

팀원별 개발 범위·착수 명령과 AI 프롬프트는 [병렬 개발 가이드](docs/TEAM_GUIDES/README.md)에 있다.

### 3.1 공통 빌드 및 MOCK 검증

Ubuntu 24.04의 Bash에서 저장소 루트를 시작 위치로 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep check --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
ros2 interface show logistics_msgs/srv/OptimizeRoute
python3 -c 'from logistics_msgs.msg import TaskStep; from logistics_msgs.srv import OptimizeRoute'
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test
colcon test-result --verbose
```

`rosdep`을 처음 사용하는 시스템에서는 `sudo rosdep init`을 한 번 실행하고 `rosdep update`로 인덱스를 받는다. 이미 초기화된 환경에서는 `rosdep update`만 필요하다. 의존성이 부족하면 Jazzy를 source한 `ros2_ws`에서 `rosdep install --from-paths src --ignore-src --rosdistro jazzy -y`를 실행하고 다시 확인한다. [rosdep 공식 안내](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)

A의 패키지 테스트만 실행하려면 설치 overlay를 source한 같은 셸에서 다음 명령을 실행한다.

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q src/mission_manager/test
```

테스트는 메모리 내 상태 검증, 파일 SQLite 복구, 테스트용 OptimizeRoute 서버와의 ROS2 DDS 왕복, D·E 대역을 사용하는 미션 시나리오를 포함한다. 공통 테스트에는 실제 Nav2 서버, 지도, Gazebo, 로봇이 필요 없다.

| 담당 | 개발 시작 위치 | 첫 연동 지점 |
|---|---|---|
| A | `mission_manager`, `logistics_msgs` | 독립 MOCK 구현 완료. D 연동 계약 검토 대기 |
| B | `experiments/path_planner/` | 작업 중인 원격 `feat/b-path-planner`를 추적해 비교 실험을 이어간다 |
| C | `route_optimizer`, `amr_bringup` | 최적화는 OptimizeRoute 서비스로 입출력을 맞추고, Bringup은 별도로 센서·TF 점검 |
| D | `amr_navigation` | 지도 없는 별도 프로세스 MOCK부터 준비. [A↔D 인수인계](docs/INTEGRATION_A_D_HANDOFF.md) |
| E | `web_control/` | 작업 등록·인계 요청과 A가 제공할 상태를 기준으로 웹·DB 개발 |

개별 패키지와 의존 패키지를 빌드할 때는 워크스페이스에서 `colcon build --symlink-install --packages-up-to mission_manager`의 마지막 인자를 해당 패키지명으로 바꾼다. 공통 타입 변경 후에는 전체 빌드와 테스트를 수행한다. 필드 의미와 승인 범위는 [인터페이스 문서](docs/INTERFACES.md)를 따른다.

## 4. 개발환경

### 4.1 공식 개발 기준

| 항목 | 기준 |
|---|---|
| 운영체제 | Ubuntu 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Python | 3.12 |
| ROS2 빌드 | colcon |
| 의존성 관리 | rosdep |
| 내비게이션 | Nav2 |
| 시뮬레이터 | Gazebo Harmonic |
| Gazebo 연동 | ros_gz |
| 개발 도구 | VS Code, Git |
| 웹 백엔드 | FastAPI |
| 데이터베이스 | SQLite |

기존 Ubuntu 22.04 / ROS2 Humble은 이전 프로젝트의 개발환경이며, 본 프로젝트의 신규 구현 기준으로 사용하지 않는다.

### 4.2 Mac 기반 개발환경

초기 개발환경은 다음과 같이 구성했다.

- Host: Apple Silicon MacBook Air M2
- 가상화: UTM / QEMU
- Guest: Ubuntu 24.04 ARM64(aarch64)
- ROS2: Jazzy
- Gazebo: Harmonic
- IDE: VS Code
- 터미널: Terminator

UTM 환경에서 ROS2 통신, 패키지 생성·빌드·실행 및 Gazebo 기본 시뮬레이션을 확인했다.

다만 Gazebo와 RViz의 3D 렌더링은 가상 GPU 설정에 따라 안정성과 성능이 달라질 수 있다. 그래픽 가속 설정과 소프트웨어 렌더링 테스트 결과는 구분하여 관리한다.

이 환경의 검증 결과가 다른 팀원의 PC나 실제 TurtleBot3에서 동일하게 재현된다는 의미는 아니다.

### 4.3 개발환경 확인

Ubuntu 터미널에서 다음 명령어로 기본 환경을 확인한다.

```bash
# 운영체제 및 CPU 아키텍처
lsb_release -a
uname -m

# ROS2
echo $ROS_DISTRO
ros2 --version 2>/dev/null || ros2 doctor --report

# Python
python3 --version

# rclpy
python3 -c "import rclpy; print('rclpy OK')"

# colcon
colcon list

# Nav2
ros2 pkg prefix nav2_bringup

# Gazebo ROS2 연동
ros2 pkg prefix ros_gz_sim

# Gazebo
gz sim --version
```

`colcon list`는 현재 디렉터리에서 발견된 패키지를 출력한다. 패키지가 없는 디렉터리에서는 출력이 없어도 설치 오류를 의미하지 않는다.

### 4.4 ROS2 통신 테스트

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

`Publishing`과 `I heard` 로그가 연속적으로 출력되면 기본 ROS2 토픽 통신이 정상 동작하는 것으로 판단한다.

종료할 때는 각 터미널에서 `Ctrl+C`를 누른다.

### 4.5 Gazebo 및 RViz 실행

Gazebo 기본 시뮬레이션:

```bash
gz sim shapes.sdf
```

RViz 실행:

```bash
rviz2
```

UTM 환경에서 그래픽 문제가 발생하면 다음 명령으로 렌더러를 확인한다.

```bash
glxinfo -B | grep 'OpenGL renderer'
```

필요한 경우 RViz를 소프트웨어 렌더링으로 실행한다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

이 명령은 RViz의 소프트웨어 렌더링을 강제하는 것으로, GPU 가속을 활성화하는 명령이 아니다.

**주의:** UTM 환경에서 Gazebo와 RViz의 실행 성공 여부는 ROS2·Nav2 실차 주행 가능 여부와 별개다.

## 5. 문서 안내

| 문서 | 목적 |
|---|---|
| `AGENTS.md` | Codex 공통 규칙 및 개발 원칙 |
| `docs/reference/ORIGINAL_PROPOSAL.md` | 최종 기획 제안서 — 최상위 요구사항 |
| `docs/PRD.md` | 기능 요구사항 및 Priority P0/Priority P1/Priority P2 우선순위 |
| `docs/ARCHITECTURE.md` | 승인된 시스템 아키텍처 v1.0 |
| `docs/INTERFACES.md` | 공통 데이터·통신 계약 |
| `docs/SIM_TO_REAL.md` | 실차 검증 및 Gate 0~5 |
| `docs/DECISIONS.md` | 승인된 결정 및 미확정 기술 항목 |
| `docs/reference/CODEX_FULL_BRIEF.md` | Codex용 전체 프로젝트 맥락 |
| `CONTRIBUTING.md` | Git 협업 및 변경·검증 규칙 |

세부 인터페이스와 실제 실행환경은 확인 후 확정한다.

원본 기획 제안서는 보존하며, 이후 승인된 개발환경 변경사항은 `docs/DECISIONS.md`에서 관리한다.

## 6. Codex 사용 방법

1. 프로젝트 루트를 Codex에서 연다.
2. `AGENTS.md`와 현재 작업 관련 문서를 확인한다.
3. [공통 AI 개발 프롬프트](docs/TEAM_GUIDES/COMMON_AI_PROMPT.md)와 담당 가이드를 함께 사용한다.
4. 승인된 구조와 실제 저장소·환경의 차이를 확인한다.
5. 이후 A 담당자의 작업 승인 범위 내에서 구현과 검증을 진행한다.

초기 아키텍처 v1.0은 승인되었으며, 처음부터 다시 설계할 필요는 없다.

단, 구체적인 Topic·Service·Action 이름과 메시지 필드 등 미확정 사항은 관련 문서에서 검토한다.

ROS2 Humble 기준 예제나 기존 프로젝트 코드를 사용할 경우 Jazzy 호환성을 검증한다.

**검증 원칙:** MOCK·SIM·REAL 결과를 구분한다. 실차에서 검증하지 않은 기능은 구현 완료로 보고하지 않는다.

## 7. 개발 우선순위

**Priority P0 — 필수**

실차 Bringup, Nav2 기본 주행, 작업 상태 머신, 방문 순서 최적화, 웹 인계 확인, 다중 지점 이동, HOME 복귀, 기본 동적 재계획 및 주요 알고리즘 비교.

**Priority P1 — 고도화**

DWB·RPP 비교, 고급 예외 처리, 실험 분석 고도화, 필요 시 ArUco 정밀 정렬.

**Priority P2 — 선택**

자연어·음성 입력, 추가 최적화 및 실험 자동화.

## 8. Sim-to-Real 검증

개발환경 구축과 실차 주행 검증은 구분하여 진행한다.

| 구분 | 검증 대상 | 현재 상태 |
|---|---|---|
| 개발환경 | Ubuntu 24.04 / ROS2 Jazzy 설치 | 확인 |
| ROS2 통신 | Talker / Listener | 확인 |
| 패키지 빌드 | colcon 빌드 및 노드 실행 | 확인 |
| 시뮬레이터 | Gazebo 기본 월드 실행 | 확인 |
| 시각화 | RViz 기본 실행 | 확인 |
| 실차 Bringup | TurtleBot3 하드웨어·센서 | 미검증 |
| 실차 Nav2 | AMCL·목표 주행 | 미검증 |
| A 미션 실행 | 주문·재계획·인계·HOME 완료 및 복구 | MOCK 검증 완료([PR #8](https://github.com/waterhyunsu/SMART-LOGISTICS-AMR/pull/8)) |
| 실제 모듈 통합 | A↔C 실제 서버, A↔D 전송·복구, A↔E 웹 인계 | 통합 대기 |

Gazebo·RViz 기본 동작은 초기 UTM 환경에서 확인했다. 새 강의실 지도·Waypoint Pose와 실제 물류 주행 검증은 별도 단계에서 다룬다.

실차 통합 절차와 Gate 0~5의 세부 기준은 `docs/SIM_TO_REAL.md`를 따른다.

## 9. 관련 프로젝트

기존 ROS2·Nav2 개발 경험의 참고 자료:

- [SMART-VISION-AMR](https://github.com/kiki8105/SMART-VISION-AMR)
- [Resilient AMR A*](https://github.com/waterhyunsu/resilient-amr-astar)

기존 저장소는 참고 자산이며, 현재 프로젝트의 확정 구현 또는 검증 결과로 취급하지 않는다.

---

**개발 목표:** 실제 TurtleBot3에서 물류 작업 등록 → 방문 순서 최적화 → 주행 → 인계 확인 → 동적 재계획 → HOME 복귀까지 통합 검증한다.
