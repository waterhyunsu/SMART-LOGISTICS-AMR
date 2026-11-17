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
- **현재 단계:** P0 구현 준비 및 초기 실차 검증

> 개발 PC의 ROS2 Jazzy 환경은 기본 동작을 확인했다. TurtleBot3 실차의 Jazzy 호환성 및 통합 주행은 별도 검증이 필요하다.

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
│   └── reference/
│       ├── ORIGINAL_PROPOSAL.md
│       └── CODEX_FULL_BRIEF.md
├── prompts/
└── scripts/
```

위 구조는 승인된 초기 아키텍처 기준이며, 모든 패키지와 실행 기능이 이미 구현되었다는 의미는 아니다.

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
| `docs/PRD.md` | 기능 요구사항 및 P0/P1/P2 우선순위 |
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
3. 초기 검증 시 `prompts/01_INITIAL_ARCHITECTURE_REVIEW.md`를 사용한다.
4. 승인된 구조와 실제 저장소·환경의 차이를 확인한다.
5. 이후 A 담당자의 작업 승인 범위 내에서 구현과 검증을 진행한다.

초기 아키텍처 v1.0은 승인되었으며, 처음부터 다시 설계할 필요는 없다.

단, 구체적인 Topic·Service·Action 이름과 메시지 필드 등 미확정 사항은 관련 문서에서 검토한다.

ROS2 Humble 기준 예제나 기존 프로젝트 코드를 사용할 경우 Jazzy 호환성을 검증한다.

**검증 원칙:** MOCK·SIM·REAL 결과를 구분한다. 실차에서 검증하지 않은 기능은 구현 완료로 보고하지 않는다.

## 7. 개발 우선순위

**P0 — 필수**

실차 Bringup, Nav2 기본 주행, 작업 상태 머신, 방문 순서 최적화, 웹 인계 확인, 다중 지점 이동, HOME 복귀, 기본 동적 재계획 및 주요 알고리즘 비교.

**P1 — 고도화**

DWB·RPP 비교, 고급 예외 처리, 실험 분석 고도화, 필요 시 ArUco 정밀 정렬.

**P2 — 선택**

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
| 물류 미션 | 작업 등록부터 HOME 복귀 | 미구현·미검증 |

위 상태는 초기 Mac UTM 개발환경의 확인 결과이며, 프로젝트 전체 기능의 완료 상태를 의미하지 않는다.

실차 통합 절차와 Gate 0~5의 세부 기준은 `docs/SIM_TO_REAL.md`를 따른다.

## 9. 관련 프로젝트

기존 ROS2·Nav2 개발 경험의 참고 자료:

- [SMART-VISION-AMR](https://github.com/kiki8105/SMART-VISION-AMR)
- [Resilient AMR A*](https://github.com/waterhyunsu/resilient-amr-astar)

기존 저장소는 참고 자산이며, 현재 프로젝트의 확정 구현 또는 검증 결과로 취급하지 않는다.

---

**개발 목표:** 실제 TurtleBot3에서 물류 작업 등록 → 방문 순서 최적화 → 주행 → 인계 확인 → 동적 재계획 → HOME 복귀까지 통합 검증한다.