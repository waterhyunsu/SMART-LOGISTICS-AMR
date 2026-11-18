# C · 이현수 — 방문 순서 최적화와 Bringup

**목표와 범위.** [route_optimizer](../../ros2_ws/src/route_optimizer/)는 이동 비용·방문 순서 계산, [amr_bringup](../../ros2_ws/src/amr_bringup/)은 센서·구동 환경을 담당한다. 두 패키지는 별도로 개발·검증한다.

| 우선순위 | 작업과 연결 지점 | 완료 기준 |
|---|---|---|
| 1 | A의 OptimizeRoute를 받아 고정된 방향별 비용표로 유효 순서를 반환 | 요청·미션·기준 버전 보존, 대기 단계 완전성, 선행 조건 충족, 실패 사유 반환 |
| 2 | B의 플래너 설정과 D의 Nav2 환경에서 ComputePathToPose로 비용을 산출하고 FIFO·Greedy·완전탐색 비교 | 같은 스냅샷은 같은 cost_revision, 내용·조건 변경은 새 버전. 실패 구간은 정상 거리로 사용하지 않음 |
| 병행 | 하드웨어 OS·드라이버·센서·TF·Odometry·안전 정지 점검 결과를 D와 A에 전달 | 실차 모델·토픽·설정·검증 환경과 실패 항목이 기록됨 |

**검증.** MOCK 비용표로 수거·전달, 도달 불가, 방향 비대칭, 진행 중 단계 제외를 검사한다. 다음으로 서비스 서버를 실행해 A 클라이언트와 DDS 왕복을 확인한다. Bringup은 [실차 게이트](../SIM_TO_REAL.md)에 따라 별도 검증한다. P0에는 설치 골격과 생성 타입만 있다.

**협업.** 최신 main의 `feat/c-route-optimizer`에서 시작하고 하드웨어 변경은 `feat/c-bringup`의 별도 PR로 제출한다. 테스트와 설정 버전을 적고 공통 타입 변경은 A에게 제안한다. 다른 담당 패키지 변경은 사유와 담당자 검토가 필요하다.

**TCREI 프롬프트**

- Task — route_optimizer에서 고정 비용표 기반 계산과 optimize_route 응답부터 구현하라.
- Context — A가 승인하고 C는 계산한다. Bringup 작업은 독립 PR로 관리한다.
- Reference — [서비스 선언](../../ros2_ws/src/logistics_msgs/srv/OptimizeRoute.srv)과 [계약](../INTERFACES.md) §6·8을 따르라.
- Evaluate — 전체 대기 단계가 한 번씩 나오고 수거 이후 전달하는지, 실패·비용표 불일치를 판별할 수 있는지 검증하라.
- Iterate — 생성 타입 단위 테스트 후 DDS 왕복을 확인하고 MOCK 거리와 계획 경로 측정치를 구분하라.
