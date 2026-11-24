# D · 홍근태 — Nav2·AMCL과 주행 결과

공항 SIM과 강의실 REAL의 지도·Pose를 분리하고 조회·주행·도착 계약을 공통으로 사용한다. [공항 기획](../MOVEIT_AIRPORT.md)

**목표와 범위.** [amr_navigation](../../ros2_ws/src/amr_navigation/)에서 C의 기본 구동 환경을 받아 하나의 목표를 안정적으로 수행한다. 주행 결과는 A에게 전달하며 도착 이후 물품 인계와 미션 완료 판정은 A가 담당한다.

먼저 [A↔D 인수인계](../INTEGRATION_A_D_HANDOFF.md)에서 전송·복구 계약을 검토하고, 지도 없이 별도 프로세스 MOCK에서 동일 Attempt ID·UUID 왕복과 중복 전송 차단을 검증한다. 이후 SIM과 REAL에서 아래 흐름을 검증한다.

1. C가 확인한 센서·TF·Odometry를 입력으로 받아 AMCL 초기 위치, 지도 기준 프레임, Nav2 준비 상태를 점검한다.
2. A가 승인한 목표 Pose를 NavigateToPose로 실행한다. 요청과 결과를 연결해 성공·실패·취소를 구분하고 준비 상태와 원인을 A에 전달한다.
3. B의 플래너 설정을 반영해 C가 계획 경로를 구할 수 있는 환경을 제공한다. 단일 목표 검증 후 DWB·RPP 비교 및 도착 정렬로 확장한다.

**완료·테스트.** SIM에서 위치 추정 준비 전 목표 제한, 단일 목표 도착, 실패·취소 결과를 재현하고 Goal ID·설정·로그를 남긴다. REAL은 C의 Bringup과 A의 승인 후 별도 단일 목표 게이트로 검증한다. Phase 0 패키지 테스트는 설치 확인이다.

**협업.** 최신 main에서 `feat/d-navigation`로 분기한다. PR에는 사용한 지도·플래너·Controller, 환경과 결과를 적는다. C의 Launch 변경은 C와 협의하고 이유를 기술하며 공통 계약 변경은 A가 승인한다.

**TCREI 프롬프트** · [공통 AI 지침](COMMON_AI_PROMPT.md)과 함께 사용한다.

- Task — A↔D 인수인계를 기준으로 별도 프로세스 MOCK에 필요한 전송·복구 계약을 먼저 검토하라.
- Context — C가 센서·기본 TF를, D가 Nav2를 운영한다. 위치 도착 후 인계는 A가 관리한다.
- Reference — [인터페이스](../INTERFACES.md) §7·10과 [실차 게이트](../SIM_TO_REAL.md)를 적용하라.
- Evaluate — 준비 전·성공·실패·취소를 분리해 A에 전달되는 결과를 검증하라.
- Iterate — SIM 재현 명령과 REAL 미검증 항목을 PR에 남기고 안전 정지 절차를 보존하라.
