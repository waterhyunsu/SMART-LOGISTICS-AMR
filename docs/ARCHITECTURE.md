# 시스템 아키텍처 — SMART-LOGISTICS AMR
 
> **최상위 기준:** `docs/reference/ORIGINAL_PROPOSAL.md`  
> **요구사항 요약:** `docs/PRD.md`  
> **문서 상태:** 승인된 초기 아키텍처 설계 v1.0
> **승인권자:** A — 윤현수 (팀장)
> **기획 기준:** 2026.11.17. 최종 기획 제안서
> **기술 세부사항:** 실제 환경 검증 결과에 따라 변경 가능

본 문서는 ROS2 기반 물류 AMR 시스템의 권장 패키지 구조, 모듈 책임, 데이터 흐름 및 통합 경계를 정의하기 위한 초기 설계 작업지이다.

실제 패키지·노드·인터페이스의 생성 또는 구현 완료를 의미하지 않는다. 팀 합의가 필요한 항목은 `[초안-팀합의필요]`, 실제 환경에서 확인되지 않은 사항은 `[미확인]`으로 표시한다.

---

## 1. 아키텍처 설계 목표

1. 5명의 팀원이 독립적으로 개발할 수 있는 모듈 경계 확보
2. Job·TaskStep·Mission 상태와 로봇 주행 상태 분리
3. 전역 경로 탐색·물류 최적화·로컬 주행 제어의 책임 분리
4. 하드웨어·Bringup과 자율주행 기능 분리
5. 신규 주문 발생 시 현재 미션을 보호하는 동적 재계획
6. MOCK / SIM / REAL 환경에서 가능한 공통 인터페이스 재사용
7. 실차 검증과 단계별 통합을 우선하는 최소 구조 설계

구현 복잡도를 증가시키는 불필요한 패키지, 중간 노드, 사용자 정의 인터페이스는 추가하지 않는다.

---

## 2. 최종 팀원 역할

| 담당 | 이름 | 개발 영역 |
|---|---|---|
| A | 윤현수 | Mission Manager, 상태 머신, 재계획 적용 정책, 통합 |
| B | 정지호 | A*·Dijkstra 전역 경로 탐색 비교 및 플래너 검증 |
| C | 이현수 | 방문 순서 최적화, 재계획 계산, 하드웨어·Bringup |
| D | 홍근태 | Nav2·AMCL, DWB·RPP, 주행 제어 및 정밀 정렬 |
| E | 강연주 | FastAPI, SQLite, 웹 관제 및 실험 데이터 시각화 |

**책임 구분**

- A는 미션을 관리하고 실행을 승인한다.
- B는 전역 경로 탐색 방식을 연구·검증한다.
- C는 방문 순서를 계산하고 로봇 기본 구동 환경을 제공한다.
- D는 Nav2를 통해 실제 로봇을 주행시킨다.
- E는 작업 요청과 관제·기록 기능을 제공한다.

물류 작업 지점 A~D와 팀원 담당 코드 A~E는 서로 다른 식별 체계다.

---

## 3. Monorepo 권장 구조

```text
SMART-LOGISTICS-AMR/
├── AGENTS.md
├── README.md
├── CONTRIBUTING.md
├── .gitignore
│
├── ros2_ws/
│   └── src/
│       ├── logistics_msgs/      # A 관리, 공통 인터페이스
│       ├── mission_manager/     # A
│       ├── route_optimizer/     # C
│       ├── amr_navigation/      # D
│       └── amr_bringup/         # C
│
├── web_control/                 # E
├── scripts/                     # 공통 검증·실험 스크립트
├── prompts/                     # Codex 작업 지시
│
└── docs/
    ├── PRD.md
    ├── ARCHITECTURE.md
    ├── INTERFACES.md
    ├── SIM_TO_REAL.md
    ├── DECISIONS.md
    └── reference/
        ├── ORIGINAL_PROPOSAL.md
        └── CODEX_FULL_BRIEF.md
```

위 ROS2 패키지 디렉터리는 향후 생성할 **권장 구조**이며, 현재 실제 파일이 생성되었다는 의미가 아니다.

B의 전역 경로 탐색 비교 코드는 우선 Nav2 플래너 설정과 별도 실험 스크립트로 구성 가능한지 검토한다. 독립 ROS2 패키지가 반드시 필요한지는 초기 설계에서 결정한다.

---

## 4. ROS2 패키지별 책임

| 패키지 | 빌드 방식 후보 | 담당 | 주요 책임 |
|---|---|---|---|
| `logistics_msgs` | `ament_cmake` | A 관리 | 필요한 공통 ROS2 인터페이스 |
| `mission_manager` | `ament_python` | A | 미션 상태·실행·재계획 적용 |
| `route_optimizer` | `ament_python` | C | 계획 경로 비용표·방문 순서·재계획 계산 |
| `amr_navigation` | `ament_python` | D | Nav2 연동·목표 이동·주행 상태 |
| `amr_bringup` | `ament_cmake` | C | 하드웨어·센서·TF·통신·기본 실행 |
| `web_control` | ROS2 패키지 여부 미정 | E | FastAPI·SQLite·웹 UI |

빌드 방식은 초기 후보이며, 실제 패키지 구현 방식에 따라 검토한다.

### 4.1. `logistics_msgs` — A 관리

**책임**

- 공통 ROS2 메시지·서비스·액션의 필요한 부분 정의
- 작업 ID와 미션 상태의 일관된 표현
- 공통 인터페이스 버전 및 변경 관리

**설계 원칙**

- 표준 ROS2·Nav2 인터페이스 우선 재사용
- 모든 데이터 모델을 사용자 정의 메시지로 만들지 않음
- `ExperimentResult`는 기본적으로 JSON·CSV·SQLite로 저장
- 공통 인터페이스 변경은 관련 담당자 검토 후 반영

### 4.2. `mission_manager` — A

**책임**

- Job·TaskStep·Mission 상태 머신
- 작업 유효성 및 선후 관계 검증
- 최적화 요청 및 계산 결과 승인
- 주행 요청·도착 결과 처리
- 작업자 인계 확인 및 완료 판정
- 신규 주문 재계획 요청
- 계획 버전·상태 최신성 검사
- 미션 오류·중단·복구 정책

**범위 밖**

- 전역 경로 계산 알고리즘
- 방문 순서 최적화 계산 자체
- 모터 직접 제어
- 하드웨어 드라이버

### 4.3. `route_optimizer` — C

**책임**

- Nav2 계획 경로 기반 이동 비용표 생성
- 지점 간 출발·도착 방향별 비용 계산
- FIFO·Greedy·완전탐색 비교
- 수거·전달 선후 관계 반영
- 최적 방문 순서 계산
- 신규 주문에 따른 남은 작업 재계획
- 기준 계획 버전을 포함한 결과 반환
- 실험 결과 및 비용표 버전 관리

**범위 밖**

- 미션 상태의 최종 변경
- 재계획 결과의 실제 적용 승인
- 실제 로봇 주행 명령
- Nav2 Controller 설정

### 4.4. `amr_navigation` — D

**책임**

- Nav2·AMCL 기반 자율주행 환경
- Planner·Controller 및 Costmap 설정
- 목표 이동 요청 처리
- NavigateToPose 실행 및 피드백 관리
- 주행 결과·오류·취소 상태 반환
- Navigation Readiness 검증
- DWB·RPP 비교 및 주행 안정화
- 필요 시 도착 정밀 정렬

**범위 밖**

- 물류 방문 순서 최적화
- 작업 완료 판정
- 센서 드라이버 개발 및 기본 Bringup
- 작업 주문 DB 직접 변경

### 4.5. `amr_bringup` — C

**책임**

- TurtleBot3 하드웨어 상태 확인
- ROS2 Humble 및 로봇 기본 실행환경
- LiDAR·카메라·Odometry·IMU 등 실제 장착 장비 구동
- 센서 드라이버 및 기본 TF
- 통신·DDS·ROS_DOMAIN_ID 환경
- 하드웨어 기본 Launch
- 기본 장비 점검·복구 절차

**범위 밖**

- Nav2 로컬 Controller 성능 튜닝
- 물류 미션 실행 판단
- 방문 순서 최적화의 상태 관리

C는 `route_optimizer`와 `amr_bringup`을 모두 담당하지만, 두 패키지 간 불필요한 직접 의존성을 만들지 않는다.

### 4.6. `web_control` — E

**책임**

- 물류 주문 등록
- 물품 인계 완료 요청
- 현재 작업·미션 상태 표시
- 로봇 위치·주행 상태 관제
- 방문 순서 및 계획 경로 표시
- 실험 데이터 저장·조회·시각화
- FastAPI·SQLite 기반 웹 서비스

웹에서 인계 완료를 요청하더라도 실제 TaskStep의 상태 전이는 A의 Mission Manager가 검증·결정한다.

웹↔ROS2 연동은 FastAPI ROS2 직접 연동, 별도 게이트웨이 또는 rosbridge 방식 등을 비교한 후 결정한다.

---

## 5. 전역 경로 탐색 연구 구조 — B

B는 전역 경로 탐색 알고리즘의 성능 비교와 선정 근거를 담당한다.

**연구 후보**

- A*
- Dijkstra

우선 Nav2 Humble의 NavFn Planner 및 `use_astar` 설정을 이용한 비교 가능성을 검토한다.

**주요 산출물**

- 알고리즘별 경로 길이·계산 시간
- 경로 생성 성공률
- 실험 조건 및 설정 파일
- 시각화 자료
- 실차 검증 결과
- 최종 플래너 선정 근거

선정된 설정은 D의 Nav2 실행 환경에 적용하고, C가 계획 경로 비용표를 생성할 때 동일한 플래너 설정을 사용하도록 한다.

B가 독립적인 전역 경로 탐색 서버나 새 ROS2 패키지를 만들어야 한다고 사전에 단정하지 않는다.

---

## 6. 전체 시스템 데이터 흐름

### 6.1. 기본 물류 미션

```text
[E] Web Dashboard
    │
    │ 작업 등록
    ▼
[A] Mission Manager
    │
    │ 최적화 요청
    ▼
[C] Route Optimizer
    │
    │ 방문 순서 및 예상 이동 비용
    ▼
[A] Mission Manager
    │
    │ 목표 이동 요청
    ▼
[D] AMR Navigation / Nav2
    │
    │ 주행 결과
    ▼
[A] Mission Manager
    │
    │ 물품 인계 확인 대기
    ▼
[E] Web Dashboard
    │
    │ 작업자 인계 완료 요청
    ▼
[A] Mission Manager
    │
    ├── 다음 TaskStep 실행
    └── 모든 작업 완료 시 HOME 복귀
```

이 흐름은 업무 제어의 개념도이며, 실제 통신 연결이나 모든 데이터 경로가 이 순서로 직렬 연결된다는 의미는 아니다.

### 6.2. 경로 계산 및 실차 실행

```text
[B] Global Planner Research
    │
    │ 선정 플래너 설정·검증 기준
    ▼
[D] Nav2 Planner
    │
    │ ComputePathToPose Action
    ▼
[C] Route Optimizer
    │
    │ 계획 경로 비용표·최적 방문 순서
    ▼
[A] Mission Manager
    │
    │ 지점별 이동 요청
    ▼
[D] Nav2 Navigation
    │
    ▼
[C] TurtleBot3 Bringup / Hardware
```

Nav2 Planner는 D가 실행·관리하며, C는 해당 표준 Action으로 경로 계산을 요청할 수 있다.

C의 Bringup은 실제 로봇 구동을 위한 기반 환경이다. 최적화 계산 과정에서 하드웨어 드라이버를 직접 호출하도록 설계하지 않는다.

### 6.3. 계획 경로 비용과 실제 주행 거리

- **계획 경로 비용:** `ComputePathToPose`가 반환한 경로 길이를 기반으로 계산
- **실제 주행 거리:** 실차 실행 과정의 Odometry 또는 검증된 위치 데이터로 별도 측정

두 데이터를 구분하여 기록한다.

비용표는 지도·Costmap·플래너·지점 좌표 등의 버전과 연결하고, 관련 설정이 변경되면 유효성을 확인한다.

---

## 7. 동적 재계획 구조 — P0

### 7.1. 기본 재계획 흐름

```text
신규 주문 접수 (E)
        ↓
현재 미션 상태 확인 (A)
        ↓
재계획 계산 요청 (A → C)
        ↓
남은 방문 순서 계산 (C)
        ↓
계산 결과 + 기준 계획 버전 반환 (C → A)
        ↓
상태·버전 유효성 검증 (A)
        ↓
현재 작업 완료 후 변경 계획 적용 (A)
        ↓
다음 목표 주행 요청 (A → D)
```

### 7.2. 책임 분리

| 항목 | A | C |
|---|---|---|
| 신규 주문에 따른 재계획 판단 | 담당 | 입력 수신 |
| 현재 작업 보호 | 담당 | 계산 조건으로 반영 |
| 방문 순서 재계산 | 요청·검증 | 담당 |
| 기준 계획 버전 관리 | 최신성·적용 관리 | 기준 버전 포함 결과 반환 |
| 실제 적용 시점 | 담당 | 적용하지 않음 |
| 주행 명령 | D에 요청 | 직접 실행하지 않음 |

**설계 원칙**

- 진행 중인 작업의 상태를 보존한다.
- 완료된 작업을 다시 실행하지 않는다.
- 수거·전달 선후 관계를 유지한다.
- 오래된 계획 계산 결과가 최신 계획을 덮어쓰지 못하게 한다.
- P0에서는 현재 작업 완료 후 다음 작업부터 새 계획을 적용한다.
- 고급 실패 복구·복잡한 동시 주문 충돌 처리는 P1에서 고도화한다.

---

## 8. 모듈 인터페이스 후보

| 연결 | 목적 | 우선 검토 방식 |
|---|---|---|
| E → A | 작업 등록·인계 완료 요청 | REST 또는 ROS2 Gateway |
| A → C | 방문 순서·재계획 요청 | ROS2 Service 또는 Action |
| C → A | 계획 결과 반환 | 요청 응답 방식에 따라 결정 |
| A → D | 목표 이동 요청 | 표준 Nav2 Action 활용 여부 검토 |
| D → A | 주행 준비 상태·실행 결과 | Action 결과 및 상태 전달 |
| B ↔ D | 플래너 설정·검증 | 설정 파일·실험 및 Nav2 인터페이스 |
| C → D/Nav2 | 계획 경로 비용 계산 | `ComputePathToPose` Action |
| C → D | 로봇 기본 구동환경 제공 | Bringup Launch·표준 ROS2 Topic/TF |
| A → E | 현재 미션 상태 제공 | Topic 또는 웹 Gateway |
| B·C·D → E | 실험 결과 기록 | JSON·CSV·SQLite 등 |

위 표는 인터페이스 방향과 후보 기술만 정의한다.

실제 Topic·Service·Action의 이름, 메시지 구조, QoS, Timeout, 중복 요청 처리, 오류 코드는 `docs/INTERFACES.md`에서 검토하고 팀 합의 후 확정한다.

사용자 정의 ROS2 메시지는 꼭 필요한 부분만 `logistics_msgs`에 포함한다.

---

## 9. Sim-to-Real 책임 및 통합 경계

### 9.1. 실행 환경

- **MOCK:** 로봇 없이 상태 머신·계획 계산·웹 연동 검증
- **SIM:** Gazebo와 Nav2 기반 시뮬레이션 검증
- **REAL:** 실제 TurtleBot3 센서·위치 추정·주행 검증

실행 환경별 Launch·설정 및 ROS2 통신 환경을 구분한다.

`use_sim_time`은 시간 기준 설정이며 실행 환경을 격리하지 않는다.

### 9.2. Readiness 구분

| 검증 영역 | 담당 | 검사 대상 |
|---|---|---|
| Hardware / Bringup Readiness | C | 센서·드라이버·통신·기본 TF·구동 |
| Navigation Readiness | D | AMCL·Nav2 Lifecycle·Planner·Controller |
| Mission Execution Readiness | A | 현재 미션 상태·작업 조건·필수 주행 준비 상태 |

필수 상태가 미확인 또는 실패인 경우 자동 주행을 허용하지 않는 구조를 검토한다.

주행 중에도 통신·위치 추정·안전 상태를 감시한다.

실차 안전 및 검증 Gate 0~5의 상세 기준은 `docs/SIM_TO_REAL.md`에서 관리한다.

---

## 10. 초기 통합 순서

| 단계 | 개발 목표 | 담당 |
|---|---|---|
| 1 | 하드웨어·센서·ROS2 Bringup | C |
| 2 | Nav2 단일 목표 실차 주행 | D, C 지원 |
| 3 | Mock Mission Manager 및 상태 머신 | A |
| 4 | 전역 플래너 비교 기본 실험 | B |
| 5 | 계획 경로 비용표·기본 방문 순서 계산 | C |
| 6 | 웹 작업 등록·인계 확인 | E |
| 7 | 웹 → 미션 → 최적화 → 실차 주행 최초 통합 | A·C·D·E |
| 8 | 기본 동적 재계획 통합 | A·C |
| 9 | 알고리즘 비교 및 실차 반복 검증 | B·C·D |
| 10 | 최종 통합·시연 안정화 | 전체 |

각 담당자는 공통 인터페이스가 확정되기 전에도 Mock을 활용하여 독립 개발할 수 있도록 한다.

다만 실제 로봇과의 연결은 Readiness 및 안전 검증을 완료한 뒤 수행한다.

---

## 11. 승인 상태 및 세부 구현 결정

### 11.1. 승인 완료

- [x] 5개 기본 ROS2 패키지와 담당 소유권
- [x] A~E 역할 및 모듈 책임 경계
- [x] 기본 물류 데이터 흐름
- [x] C의 하드웨어·Bringup 전담
- [x] D의 Nav2·AMCL 및 자율주행 담당
- [x] A의 Mission Manager 상태·실행 관리
- [x] 기본 동적 재계획 P0 포함
- [x] MOCK / SIM / REAL 구분과 Gate 0~5 검증 구조

### 11.2. 세부 구현 결정 필요

- [ ] ROS2 패키지별 실제 빌드 방식과 노드 구성
- [ ] B의 전역 플래너 연구 코드 배치
- [ ] `logistics_msgs`의 메시지·서비스 정의
- [ ] A↔C 최적화 요청·응답 구현 방식
- [ ] A↔D 주행 실행 인터페이스
- [ ] E↔ROS2 연동 방식
- [ ] 실제 Topic·Service·Action 이름과 QoS
- [ ] Plan Version·중복 요청·Timeout 처리 방식
- [ ] 실제 장비 설정 및 실차 검증 결과

승인된 구조는 개발 기준으로 유지한다.

미확정 세부사항은 관련 담당자의 검토 후 A가 최종 승인하고, 주요 결정은 `docs/DECISIONS.md`에 기록한다.

---

## 12. 문서 관계

| 문서 | 역할 |
|---|---|
| `AGENTS.md` | 모든 Codex 작업의 공통 규칙 |
| `docs/reference/ORIGINAL_PROPOSAL.md` | 최종 기획 요구사항 |
| `docs/PRD.md` | 요구사항 및 우선순위 요약 |
| `docs/ARCHITECTURE.md` | 승인된 시스템 아키텍처 v1.0 |
| `docs/INTERFACES.md` | 세부 데이터·통신 계약 |
| `docs/SIM_TO_REAL.md` | 실차 검증 및 안전 기준 |
| `docs/DECISIONS.md` | 팀 합의·설계 승인 기록 |

최종 기획서와 충돌하는 설계 내용은 팀 합의 없이 확정하지 않는다.

---

## 수정 기록

- **2026-11-16:** 초기 아키텍처 작업지 작성
- **2026-11-17:** 최종 역할 분담 및 P0 요구사항 반영
  - A: Mission Manager·재계획 적용 정책
  - B: 전역 경로 탐색
  - C: 물류 최적화·하드웨어·Bringup
  - D: Nav2·자율주행·로컬 제어
  - E: 웹 관제·데이터
  - 계획 경로 비용과 실제 주행 데이터 분리
  - 기본 동적 재계획 P0 반영

**현재 상태: 초기 아키텍처 v1.0 승인. 패키지 구조·역할·모듈 경계는 확정되었으며, 구체적인 Topic·Service·Action 및 메시지 필드 등 세부 구현 계약은 별도 결정한다.**