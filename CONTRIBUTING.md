# Git 협업 규칙 — SMART-LOGISTICS AMR

> **문서 상태:** 초기 협업 운영 기준 v1.0  
> **기준일:** 2026-11-17  
> **최종 승인권자:** A — 윤현수 (팀장)

본 문서는 SMART-LOGISTICS AMR 프로젝트의 Git 브랜치 운영, 코드 변경, Pull Request 및 통합 검증 기준을 정의한다.

## 1. 기본 협업 원칙

- 단일 GitHub Monorepo를 사용한다.
- `main`은 통합 검증된 기준 브랜치로 관리한다.
- 기능 개발은 별도 브랜치에서 진행한다.
- `main` 직접 Push보다 Pull Request를 통한 병합을 우선한다.
- 공통 인터페이스와 아키텍처 변경은 A의 승인을 받는다.
- 다른 담당자 영역의 코드를 협의 없이 변경하지 않는다.
- AI 생성 코드도 해당 담당자가 검토·테스트한다.

## 2. 브랜치 규칙

**기능 개발**

- `feat/a-mission-manager`
- `feat/b-global-planner`
- `feat/c-route-optimizer`
- `feat/c-bringup`
- `feat/d-navigation`
- `feat/e-dashboard`

**버그 수정**

- `fix/a-plan-version`
- `fix/c-sensor-tf`
- `fix/d-nav2-controller`

**문서 수정**

- `docs/architecture-update`
- `docs/interface-contract`

브랜치명은 실제 작업 범위가 드러나도록 작성한다.

## 3. 패키지 소유권

| 패키지·영역 | 담당 |
|---|---|
| `logistics_msgs` | A 관리, 공통 |
| `mission_manager` | A |
| 전역 플래너 연구 코드·설정 | B |
| `route_optimizer` | C |
| `amr_bringup` | C |
| `amr_navigation` | D |
| `web_control` | E |

**공통 변경 규칙**

- 공통 메시지·서비스 변경: A 승인 및 영향받는 담당자 검토
- 전역 플래너 설정 변경: B·C·D 영향 확인
- 센서·Bringup 변경: C 주관, D 영향 확인
- Nav2·Controller 변경: D 주관, B·C 영향 확인
- 웹↔Mission Manager 계약 변경: A·E 검토
- 계획·재계획 계약 변경: A·C 검토

담당자가 자신의 영역을 수정하는 일반적인 작업과 공통 계약을 변경하는 작업을 구분한다.

## 4. Pull Request 기준

PR에는 다음 내용을 포함한다.

1. 변경 목적 및 작업 범위
2. 주요 수정 파일
3. 영향받는 모듈과 인터페이스
4. 실행한 테스트 및 결과
5. 미검증 사항
6. 필요 시 복구·롤백 방법

**PR 제목 예시**

- `[A] Implement Mission state transitions`
- `[B] Compare A* and Dijkstra`
- `[C] Add route cost matrix`
- `[C] Configure TurtleBot3 Bringup`
- `[D] Integrate Nav2 goal execution`
- `[E] Implement delivery confirmation UI`

공통 계약 변경은 영향받는 담당자 검토 후 A가 최종 승인한다.

## 5. 검증 기준

**ROS2 패키지**

- 가능한 환경에서 `colcon build` 실행
- 필요한 경우 `colcon test` 실행
- 메시지·서비스의 Build 및 Import 확인
- 관련 노드의 기본 실행 결과 기록

**웹·DB**

- API 및 요청·응답 검증
- DB 저장·조회 확인
- 작업 중복 요청 및 오류 처리 확인

**실차**

- MOCK / SIM / REAL 검증 결과 구분
- 실차 미검증 항목은 명시
- Nav2·센서·TF·통신 설정 변경 시 영향 검토
- 기존 정상 동작 설정 보존
- 실제 장비 구동 전 안전 절차 준수

테스트를 수행하지 않은 경우 통과했다고 기록하지 않는다.

## 6. 인터페이스 변경 절차

1. 변경 필요성 확인
2. 변경 대상과 영향받는 모듈 식별
3. 관련 담당자와 기술적 영향 검토
4. A의 최종 승인
5. 코드 및 문서 반영
6. 관련 모듈 회귀 테스트
7. `docs/DECISIONS.md`에 주요 결정 기록

공통 데이터 모델, Topic·Service·Action, Plan Version 정책 등의 변경은 이 절차를 적용한다.

## 7. 실차 운영 규칙

TurtleBot3는 팀 전체가 공유하는 장비 1대다.

- A: 실차 시험 일정 및 통합 관리
- C: 하드웨어·센서·Bringup 관리
- D: Nav2·AMCL·주행 설정 관리

로봇을 동시에 제어하는 중복 명령을 방지한다.

실차 관련 변경은 담당자 간 조율하고, 검증된 안전 절차를 따른다.

검증되지 않은 설정을 안정적인 통합 실행 버전에 바로 적용하지 않는다.

## 8. 추후 운영 설정

다음 GitHub 설정은 실제 저장소 상태를 확인한 후 적용한다.

- [ ] Branch Protection 설정
- [ ] 필수 PR 승인 수
- [ ] 자동 테스트 및 CI 적용
- [ ] 실차 안정 버전 Tag 관리
- [ ] Breaking Change 처리 방식

이 항목들은 아직 GitHub에 설정되었다고 가정하지 않는다.

---

**최종 원칙:** 개인 모듈은 독립적으로 개발하되, 공통 계약은 승인 절차를 거치고 실제 동작 검증을 기반으로 통합한다.