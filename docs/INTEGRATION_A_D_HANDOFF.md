# A↔D 통합 인수인계

대상은 D 담당 홍근태다. 확인 기준은 PR #8 병합 커밋 `5fae47b`이며 개발 환경은 Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Python 3.12다. 새 강의실 지도와 실제 Waypoint Pose는 미확인이다. [PR #7](https://github.com/waterhyunsu/SMART-LOGISTICS-AMR/pull/7) · [PR #8](https://github.com/waterhyunsu/SMART-LOGISTICS-AMR/pull/8)

## D의 첫 MOCK PR

최신 `main`에서 시작하고, 진행 중인 D 브랜치가 있으면 기존 작업을 보존해 이어간다. 첫 PR은 `ros2_ws/src/amr_navigation/`의 D 대역과 검증 절차를 대상으로 한다. 필드 초안을 작성해 A와 검토한 뒤, A 대역과 D 대역을 **별도 프로세스**로 실행한다. A의 실제 호출 경계 연결과 A측 테스트는 A가 담당한다. 지도·Pose·Nav2 Action 연결은 Gate 1 통과 후에 진행한다.

| 확인할 시나리오 | 통과 기준 |
|---|---|
| 정상 왕복 | A의 목표를 D가 접수하고, 같은 미션·단계·계획·Waypoint·Attempt ID·Goal UUID로 수락과 도착을 각각 보고한다. A 대역은 결과를 구분하고 도착 뒤 인계 대기로 처리한다 |
| 같은 목표 재요청 | D의 목표 송신 대역 호출 횟수가 1회다. 동일 Attempt ID·UUID·컨텍스트에는 기존 처리 상태를 반환하고, 같은 식별자에 다른 컨텍스트가 오면 송신을 거부한다 |
| 늦은 결과 | 이전 Attempt ID·UUID의 도착 결과와 이전 Recovery Check ID의 조회 응답이 현재 상태를 바꾸지 않는다 |
| 타임아웃 | 수락 응답 지연 또는 D 종료 시 기존 목표를 보존하고 새 송신은 0회다. 활성 실행·지연 송신이 남거나 알 수 없으면 기존 시도를 유지한다 |
| 명시적 복구 | D 대역에서 활성 실행과 지연 송신을 모두 제거하고 최신 조회에 부재를 보고한다. A 승인 전에는 송신 0회이며, 승인 이후 새 Attempt ID·UUID로만 목표를 전송한다 |

PR에는 두 프로세스의 실행 명령, 테스트 대역의 송신 횟수, 요청·응답 식별자와 통과 여부를 남긴다. A↔D 실제 연결 결과는 A측 어댑터가 준비된 후 별도로 기록한다.

## A 담당 완료 판정

| 구분 | 판정 |
|---|---|
| 독립 구현 완료 | Mission·Job·TaskStep, C 최적화 요청·검증, 인계 경계에서의 재계획, HOME 최종 완료, SQLite 영속화 및 ACK 멱등성, 논리 목표 선점, Attempt ID·UUID 저장, 송신 불확실 복구와 명시 재승인 |
| MOCK 검증 완료 | 파일 SQLite 재시작·저장 실패, 오래된 결과, 활성 실행·지연 송신 대역을 통한 승인 차단, 일반 단계와 HOME 복귀. PR #8 기록은 패키지 5개 빌드, 전용 12건, 전체 87건 통과 |
| 실제 연동 대기 | D의 Nav2 실행·관측과 안전 근거, A↔D 프로세스 전송, C 실제 비용표·최적화, E의 실제 인계 ACK, 새 지도 및 실차 |
| 설계 결정 필요 | 공통 서비스 타입·필드·네임스페이스와 기한 값, 이전 송신의 무효화 절차, 운영자 확인·복구 정책 |
| 결함 수정 필요 | 현재 검토 범위에서 독립 MOCK 개발 재개를 요구하는 신규 Critical·Major는 발견하지 못했다. 시스템 완료 판정은 실제 통합 결과가 필요하다. |

A의 독립 MOCK 개발을 종료하고 D 연동 준비로 전환한다. 현재 `amr_navigation`은 패키지 골격과 설치 검사 단계다. [A 가이드](TEAM_GUIDES/A_MISSION_MANAGER.md) · [D 가이드](TEAM_GUIDES/D_NAVIGATION.md) · [A 구현 README](../ros2_ws/src/mission_manager/README.md)

## 데이터 및 함수 계약

현재 연동 경계는 A의 단일 executor에서 호출하는 Python 객체와 함수다. 첫 별도 프로세스 MOCK은 아래 ROS 2 Service 세 개를 사용한다. 현재 저장소의 Python 경계가 전송 구현의 기준이며 공통 `.srv` 파일과 실제 프로세스 연결은 후속 PR에서 구현한다. [코드](../ros2_ws/src/mission_manager/mission_manager/navigation_boundary.py) · [노드](../ros2_ws/src/mission_manager/mission_manager/node.py) · [공통 계약](INTERFACES.md)

| 데이터 | 소유·생성 | 전달·검증 |
|---|---|---|
| Mission ID | A, 미션 초기화·복원 | 목표·복구 컨텍스트에서 현재 미션과 일치 |
| TaskStep ID | A, Job 등록 | 목표의 단계와 일치. 단독 HOME 복귀는 빈 단계인 `None` |
| Plan Version | A, 후보 승인 | 목표에 승인된 버전 보존. D가 재산정하지 않음 |
| Waypoint ID | A, 논리적 경유점 지정 | D가 승인된 지도 버전의 Pose로 해석. 누락·불일치 시 송신 보류 |
| Attempt ID | A, 최초 선점·명시 재승인 시 SQLite 기록 | 목표와 복구 컨텍스트에 포함. 이전 시도의 결과 거부 |
| Nav2 Goal UUID | A, 시도별 선발급 및 SQLite COMMIT | D가 동일 16바이트 UUID로 Nav2 송신. A는 현재 논리 목표와 UUID를 함께 검증 |
| 수락·거부·실행 상태 | D, Nav2 응답과 관측 | 동일 실행 컨텍스트로 보고. 현재 A에 수락 또는 진행 상태를 저장하는 전용 API는 없다 |
| 도착·실패·취소 | D, Nav2 결과 | 현재 A의 `observe`는 일치하는 `ARRIVED`만 상태 전이. 실패·취소의 영속 상태 매핑은 협의 대상 |
| 물품 인계·미션 완료 | A, 도착 이후 작업자 이벤트와 최종 조건 검증 | E의 동일 이벤트 키 재사용. 주행 성공은 TaskStep 인계를 완료하지 않는다 |
| Recovery Check ID | A, 복구 조회마다 새 발급 | D가 조회 컨텍스트와 UUID를 그대로 반환. 이전 조회는 폐기 |
| 기존 실행·지연 송신 부재 근거 | D, 복구 조회 시 확인 | `RecoveryReport(lookup, NO_ACTIVE_GOAL, no_pending_dispatch=True)`에는 D의 관측이 필요. 현재 보고 타입은 증빙 원문을 저장하지 않는다 |

## 첫 MOCK의 ROS 2 Service 세 개

아래 이름은 상대 서비스명이다. 실행 시 A·D가 같은 서비스 이름을 해석하도록 테스트 네임스페이스를 맞춘다. 표는 요청과 응답의 의미를 정의한다. 공통 `.srv`의 정확한 자료형, 상태 코드, 단독 HOME 단계의 빈 값 표현은 D가 초안을 내고 A가 검토한다.

| 서비스 | 호출 방향 | 요청과 응답 |
|---|---|---|
| `dispatch_goal` | A → D | A는 SQLite에 기록한 전체 목표 컨텍스트와 UUID를 전달한다. D는 접수 여부 또는 동일 요청의 기존 처리 상태를 반환한다. 이 응답은 D의 접수 확인이다. Nav2 또는 송신 대역의 Goal 수락은 별도 이벤트로 보고한다 |
| `report_goal_event` | D → A | D는 원래 실행 컨텍스트로 수락·거부·도착·실패·취소를 보고한다. A측은 현재 목표와 Attempt ID·UUID가 일치하는지 확인하고 처리 여부를 응답한다. 오래된 결과는 무시하고 같은 결과의 재전송은 추가 상태 전이 없이 처리한다 |
| `lookup_goal` | A → D | A는 현재 실행 컨텍스트와 최신 Recovery Check ID를 보낸다. D는 같은 컨텍스트를 돌려주고 실행이 남았는지, 도착했는지, 불확실한지 또는 실행·지연 송신 부재를 확인했는지 보고한다. 부재 보고에는 `no_pending_dispatch=True`가 필요하다 |

`dispatch_goal`의 접수 응답과 `report_goal_event`의 수락 보고는 각각 지연되거나 유실될 수 있다. A는 먼저 받은 응답으로 뒤의 상태를 추정하지 않으며, D는 수락 여부나 결과를 확인할 수 없으면 그 의미를 A에 전달한다. 동일 목표 재요청은 기존 기록을 조회하는 동작으로 취급한다. 상태 조회가 불가능하면 불확실로 남긴다.

## 정상 실행

1. D는 Nav2 lifecycle과 서버 가용성, 지도와 Waypoint 설정, TF·Odometry·AMCL 준비 상태를 확인한다. A가 논리적 실행을 승인한다.
2. A는 `claim_next_goal()`로 현재 단계를 선점한다. `prepare_dispatch(goal)`은 UUID를 커밋한 후 티켓을 반환한다. `submit_once(ticket, sender)`가 D 송신 함수를 한 번 호출한다. `current_goal()`은 조회용이다.
3. D는 미션·단계·계획·시도·UUID를 보존하고 Waypoint를 `PoseStamped`로 해석한다. Nav2 `NavigateToPose.Goal`의 페이로드는 `pose`와 `behavior_tree`다. A의 논리 목표는 별도 문맥이며 Goal UUID는 Action 송신 요청의 필드다. [Nav2 Jazzy Action 정의](https://api.nav2.org/actions/jazzy/navigatetopose.html)
4. D는 A의 32자리 hex 값을 `unique_identifier_msgs.msg.UUID(uuid=list(bytes.fromhex(ticket.nav_goal_id)))`로 변환해 `ActionClient.send_goal_async(goal, goal_uuid=uuid_msg)`에 제공한다. 수락 Future의 GoalHandle로 수락 여부 및 UUID를 대조한 뒤 결과·취소를 별도로 처리한다. 수락 응답이나 송신 함수의 반환만으로 도착을 판정하지 않는다. [rclpy Jazzy 클라이언트](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/action/client.py)
5. D가 동일 Goal의 최종 성공 상태와 결과를 확인하면 A 경계에 `observe(ticket, ARRIVED)`를 전달한다. A는 일반 단계를 인계 대기로 둔다. 다음 목표는 인계와 재계획 적용 조건을 충족한 뒤에만 선점한다. 물류 단계와 별개인 HOME 복귀는 도착으로 완료할 수 있으며, HOME 하역 TaskStep은 작업자 인계를 받는다.

## 실패·불확실 상태의 의미

| D 관측 | A에 전달할 의미와 현재 동작 |
|---|---|
| Nav2 서버 연결 불가 | 선점·송신 전에 확인하면 준비 부족으로 보류한다. 이미 준비했지만 송신 호출 전이면 기존 티켓을 보존하고 준비 확인 후 사용한다. 송신 여부가 불확실한 경우에는 복구 제한에 둔다 |
| 전송 호출 예외 | `submit_once`가 송신 티켓을 소진하고 같은 시도를 복구 제한에 둔다. D가 처리할 수 있도록 예외를 전달한다 |
| 수락 ACK 지연·시간 초과, 수락 여부 불확실 | 대기 중에는 상태를 보존한다. D 접수 응답 또는 Goal 수락 확인이 정해진 기한을 넘기면 A는 `report_dispatch_uncertain(ticket)`으로 동일 목표를 보류하고 `lookup_goal`을 시작한다. 결과를 모르면 `UNKNOWN`으로 유지한다. 타임아웃 자체는 미실행 증거가 될 수 없다 |
| 수락 거부 | 해당 요청의 명시적 거부를 기록한다. 재전송을 승인하려면 다른 지연 송신도 배제해야 한다 |
| 실행 중 통신 단절 | 현재 실행을 불확실로 관리한다. `report_dispatch_uncertain(ticket)` 이후 활성 실행과 지연 송신을 조회한다 |
| 실행 실패 | 해당 UUID의 종결 상태와 원인을 기록한다. 정상 도착·인계로 전이하지 않는다. 현재 경계에서는 복구 제한과 확인 절차로 처리한다 |
| 취소 성공·실패 | 취소 요청의 수락과 최종 `CANCELED`를 따로 확인한다. 거부·타임아웃·`CANCELING`은 실행 중일 수 있다. 수락 여부나 결과가 불확실하면 복구 제한을 유지한다 |
| 정상 도착 | 해당 UUID와 시도의 확정 성공을 현재 티켓 또는 최신 복구 조회에 연결한다. 늦은 결과는 현재 상태와 대조한다 |

종결 실패·취소·거부를 A의 별도 상태 및 운영 화면에 어떻게 표현할지는 구현 전에 합의한다. 현재 보류·확인 흐름에서는 목표를 그대로 유지하고, `NO_ACTIVE_GOAL` 기준과 명시 승인을 만족한 경우에만 새 실행이 가능하다. [ROS 2 Action 설계](https://design.ros2.org/articles/actions.html)

## 재시작 후 UUID 확인과 승인

설치된 환경에서 `rclpy 7.1.13`, `nav2_msgs 1.3.14`, `nav2_bt_navigator 1.3.14`와 `send_goal_async(..., goal_uuid=None)` 시그니처를 확인했다. `ClientGoalHandle.get_result_async()`와 `cancel_goal_async()`는 기존 핸들에서 호출된다. 공개 `ActionClient`에는 UUID를 넣어서 이전 GoalHandle을 복원하는 API가 없다.

- **D만 재시작하고 같은 Action Server가 유지된 경우** 해당 서버의 `_action/status`에서 같은 UUID의 관측을 얻을 수 있다. Action 프로토콜의 `_action/get_result`는 UUID를 받으므로 생성된 서비스 형식으로 결과를 조회하는 방법은 검토할 수 있다. 실행 중에는 응답이 완료까지 지연될 수 있으며, 기한 내 응답이 없거나 `UNKNOWN`이면 판단을 보류한다. 별도 서비스 접근을 사용할지는 통합에서 확인한다. [ROS 2 Action 상태·결과·이름 규칙](https://design.ros2.org/articles/actions.html#middleware-implementation)
- **Action Server가 재시작했거나 결과 캐시가 만료된 경우** 기존 UUID 기록을 다시 조회하지 못할 수 있다. 상태 목록의 UUID 누락, 빈 목록, 결과 조회 실패로는 기존 실행이나 로봇 상태를 확정할 수 없다. 서버 세대·lifecycle·컨트롤러/로봇 상태를 별도로 확인해야 한다. [ROS 2 결과 캐시](https://design.ros2.org/articles/actions.html#result-caching) · [Nav2 Jazzy Action 서버 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_util/include/nav2_util/simple_action_server.hpp)
- **지연 송신 배제**는 D의 책임이다. 앱 큐, 미완료 `send_goal_async`, 타이머·재시도, 다른 프로세스의 재생 및 이미 미들웨어에 넘긴 요청을 함께 고려한다. 로컬 Future 취소나 프로세스 종료만으로 서버 수신 취소를 추론할 수 없다. D는 시도별 발신 기록과 발신자 세대 관리, 이전 발신자의 종료·무효화 절차를 합의한다. 이를 증명할 수 없다면 운영자 확인과 안전한 중단 절차를 적용하고 자동 재시도를 보류한다. 이는 Action 프로토콜의 상태·결과 조회 범위에서 도출한 통합 판단이다. [ROS 2 Action 송신·취소 계약](https://design.ros2.org/articles/actions.html#middleware-implementation)

1. A가 송신 불확실 또는 재시작으로 복구 제한에 들어가면 `begin_recovery_lookup()`으로 현재 시도·UUID·새 확인 ID를 제공한다.
2. D는 같은 Nav2 인스턴스와 이전 실행을 대조한다. 활성 실행이 남아 있거나, 지연 송신이 가능하거나, 확인 근거가 부족하면 제한을 유지한다. `NOT_ARRIVED`, `UNKNOWN`, `TIMEOUT`, `DISCONNECTED`는 새 송신 승인 근거가 될 수 없다. 확정 도착을 확인한 경우에만 최신 조회에 `ARRIVED`를 반환한다.
3. D가 **기존 실행 부재와 지연 송신 배제**를 모두 확인하면 최신 조회에 `NO_ACTIVE_GOAL` 및 `no_pending_dispatch=True`를 반환한다. A가 `handle_recovery_report`로 일치 여부를 검증한 뒤, `approve_retry(lookup, explicitly_approved=True)`를 별도로 호출해야 새 Attempt ID가 커밋된다. 다음 UUID는 다시 `prepare_dispatch`에서 커밋한다. 이 경계의 SQLite COMMIT과 외부 Goal 송신은 서로 다른 작업이다. [승인된 복구 정책](https://github.com/waterhyunsu/SMART-LOGISTICS-AMR/pull/7#issuecomment-6972112713)

## 통합 테스트 Gate

아래는 A↔D 인수인계용 순서다. 실차 투입 시 기존 프로젝트의 준비·안전 게이트도 적용한다. 각 실행에 빌드 커밋, 환경, 미션·시도·UUID, 관측 결과와 실패 시점을 기록한다. [프로젝트 SIM→REAL 절차](SIM_TO_REAL.md)

| Gate | 진입 조건 | 성공 기준 | 실패 시 중단 |
|---|---|---|---|
| 1 · 별도 프로세스 MOCK | A와 D 대역, 위 ROS 2 Service 세 개, 격리된 테스트 환경. 지도 불필요 | 동일 Attempt ID·UUID 왕복. 중복 요청은 송신 1회. ACK 지연·D 종료 후 복구 제한 유지. 이전 결과 무시. 실행·지연 송신 부재 확인과 A 승인 후에만 새 시도 | 컨텍스트 누락, 예상 밖 전송·인계·재시도, 증거 없이 실행 부재 확정 |
| 2 · Nav2 SIM | Gate 1 통과. SIM 지도·Pose, TF·AMCL·Nav2 lifecycle 준비, 격리된 SIM | 준비 전 송신 제한. 단일 목표 수락·도착, 거부·실패·취소 구분. 연결 단절과 D/서버 재시작 주입 후 활성 Goal 식별 또는 제한 유지 | 준비 실패, 미확인 Goal 재전송, 예상과 다른 이동 또는 결과 |
| 3 · 복수 물류 미션 | Gate 2 통과. SIM 지도와 HOME·A~D 설정, C 비용표·E 인계 경로 또는 범위를 명시한 대역 | HOME 출발, 다중 방문, 도착 후 인계, 신규 주문 후 현재 단계 완료 경계에서 재계획, HOME 복귀와 필요한 하역 확인 | 선후 관계 위반, 인계 전 다음 단계, 이전 응답 적용 또는 하역 없이 종료 |
| 4 · 실차 | Gate 3 통과. 새 현장 지도·Waypoint Pose, TF·Odometry·AMCL, 비상정지 및 복구 준비를 C·D가 검증하고 A가 승인 | 비상정지·복구 확인, 단일 목표부터 단계적으로 확대, 승인된 전체 미션 재현 | 안전 정지·통신·위치 추정 이상, 미확인 목표, 지도·현장 불일치 |

## 결정이 필요한 항목과 A 개발 재개 조건

- ROS 2 Service 세 개와 타임아웃 시 복구 제한은 첫 MOCK의 기준이다. A·D는 공통 메시지 필드와 상태·오류 매핑, 네임스페이스, 세션 또는 발신자 식별을 검토한다. 접수·수락·조회 기한의 구체적인 값은 Gate 1·2의 실행 결과로 정한다.
- D는 Action namespace·서버 세대 식별과 UUID 조회 근거, 송신 큐의 관리 책임, 운영자가 안전하게 복구를 확인하는 조건을 제시한다. C·D는 지도 버전과 Pose 매핑을 확인한다.
- A는 D 어댑터의 검증된 동작이 현재 경계와 충돌하거나, 통합 테스트에서 중복 실행·잘못된 도착/인계·복구 교착이 재현되거나, 백로그의 미결정 전송·상태 계약이 승인된 경우에 해당 항목의 개발을 재개한다.
