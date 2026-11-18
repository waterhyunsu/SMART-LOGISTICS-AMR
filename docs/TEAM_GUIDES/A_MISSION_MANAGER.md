# A · 윤현수 — Mission Manager와 통합

**목표와 범위.** 미션·작업 상태와 계획 적용을 단일 권한으로 관리한다. [mission_manager](../../ros2_ws/src/mission_manager/)가 구현 위치다. [logistics_msgs](../../ros2_ws/src/logistics_msgs/)는 A가 관리하며 변경 시 C 등 소비자의 검토를 받는다.

| 순서 | 구현 및 입출력 | 완료 증거 |
|---|---|---|
| 1 | E의 Job을 수거·전달 단계로 등록하고 C에 OptimizeRoute 요청. D의 도착 결과와 E의 인계 이벤트를 별도로 수신 | 수거 전 전달 거부, 도착 시 대기, 한 번의 유효한 인계만 완료 |
| 2 | 신규 주문에 최신 기준으로 재요청하고 완료 경계에서 후보 적용 | 오랜 응답·비용표 또는 누락된 단계 거부, 현재 목표 보호 |
| 3 | D의 주행 준비와 C의 Bringup 준비를 확인하고 HOME 하역까지 관리. E에 상태와 인계 결과 제공 | HOME 도착만으로 미션 종료되지 않음. 장애 후 이벤트 결과와 상태를 함께 복원 |

**검증.** Phase 0의 contract.py는 메모리 내 동작 예제다. 워크스페이스 overlay에서 `python3 -m pytest -q src/mission_manager/test`를 실행한다. Phase 2에서는 C 서비스와 요청·응답 한 쌍을 DDS로 검증하고, 인계 ACK 직전 재시작을 주입해 전이 횟수와 복원 결과를 비교한다.

**협업.** 최신 main에서 `feat/a-mission-manager`로 분기해 PR을 제출한다. 인터페이스 영향·테스트 결과를 기록한다. 공통 변경은 A 승인, 타 패키지 수정은 담당자 협의와 PR 사유 설명이 필요하다.

**TCREI 프롬프트** · [공통 AI 지침](COMMON_AI_PROMPT.md)과 함께 사용한다.

- Task — 현재 단계 인계 후에만 신규 주문 계획을 적용하는 A측 흐름부터 구현하라.
- Context — C는 후보 계산, D는 목표 도착, E는 사용자 확인을 제공한다. 상태 변경 권한은 A에 있다.
- Reference — [계약](../INTERFACES.md) §5~7과 기존 mission_manager 테스트를 사용하라.
- Evaluate — 완료 ID 제외, 출발점·비용표 변경, 중복 확인, HOME 하역을 독립 시나리오로 검증하라.
- Iterate — MOCK을 통과시킨 뒤 실제 연동한 단계만 증빙하여 PR에 기록하라.
