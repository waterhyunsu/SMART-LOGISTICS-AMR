# SMART-LOGISTICS AMR

**실주행 데이터 기반 다중 물류 작업 최적화 및 동적 미션 관리 시스템**

ROS2 Humble·Nav2 기반 TurtleBot3 Waffle Pi를 활용한 5인 졸업 프로젝트.

- **최종 발표:** 2027-01-15
- **개발 환경:** Ubuntu 22.04 / ROS2 Humble / Nav2 / Python / FastAPI / SQLite
- **실차 환경:** TurtleBot3 Waffle Pi 1대, 기존 교육 강의실
- **기획 기준:** 2026-11-17 최종 기획 제안서
- **아키텍처:** v1.0 승인
- **현재 단계:** P0 구현 준비 및 초기 실차 검증

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

## 4. 문서 안내

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

## 5. Codex 사용 방법

1. 프로젝트 루트를 Codex에서 연다.
2. `AGENTS.md`와 현재 작업 관련 문서를 확인한다.
3. 초기 검증 시 `prompts/01_INITIAL_ARCHITECTURE_REVIEW.md`를 사용한다.
4. 승인된 구조와 실제 저장소·환경의 차이를 확인한다.
5. 이후 A 담당자의 작업 승인 범위 내에서 구현과 검증을 진행한다.

초기 아키텍처 v1.0은 승인되었으며, 처음부터 다시 설계할 필요는 없다.

단, 구체적인 Topic·Service·Action 이름과 메시지 필드 등 미확정 사항은 관련 문서에서 검토한다.

**검증 원칙:** MOCK·SIM·REAL 결과를 구분한다. 실차에서 검증하지 않은 기능은 구현 완료로 보고하지 않는다.

## 6. 개발 우선순위

**P0 — 필수**

실차 Bringup, Nav2 기본 주행, 작업 상태 머신, 방문 순서 최적화, 웹 인계 확인, 다중 지점 이동, HOME 복귀, 기본 동적 재계획 및 주요 알고리즘 비교.

**P1 — 고도화**

DWB·RPP 비교, 고급 예외 처리, 실험 분석 고도화, 필요 시 ArUco 정밀 정렬.

**P2 — 선택**

자연어·음성 입력, 추가 최적화 및 실험 자동화.

## 7. 관련 프로젝트

기존 ROS2·Nav2 개발 경험의 참고 자료:

- [SMART-VISION-AMR](https://github.com/kiki8105/SMART-VISION-AMR)
- [Resilient AMR A*](https://github.com/waterhyunsu/resilient-amr-astar)

기존 저장소는 참고 자산이며, 현재 프로젝트의 확정 구현 또는 검증 결과로 취급하지 않는다.

---

**개발 목표:** 실제 TurtleBot3에서 물류 작업 등록 → 방문 순서 최적화 → 주행 → 인계 확인 → 동적 재계획 → HOME 복귀까지 통합 검증한다.