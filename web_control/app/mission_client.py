"""E → A 요청 경계.

E는 이 인터페이스로만 A(Mission Manager)에게 요청하고, A가 확정한 상태를 받는다.
실제 전송 방식(rclpy 직접 / Gateway / rosbridge)은 DECISIONS P-003 [PENDING]이므로
여기서는 계약만 정의한다. 실제 구현은 방식이 확정된 뒤 별도 PR로 추가한다.

인계 결과 의미는 INTERFACES §5.3 [확정]을 따른다.
  accepted / rejected  : A가 최초로 처리한 결과 (같은 event_id 재전송 시 같은 결과 재사용)
  conflict             : 같은 event_id 를 다른 step_id 에 사용
  MissionTimeout       : 응답 미확인 — 실패로 간주하지 않는다
  MissionRecoveryPending : A가 재시작 후 복구 확인 중이라 아직 처리하지 않음 — 같은 ID로 나중에 재전송
                         (A의 mission_manager.persistence.RecoveryPending 에 대응. A README "D·E 복구 연동 경계" 3번)
"""
from typing import Literal, Protocol

HandoffOutcome = Literal["accepted", "rejected", "conflict"]


class MissionTimeout(Exception):
    """A의 응답을 받지 못함. A가 처리했는지 알 수 없는 상태."""


class MissionRecoveryPending(Exception):
    """A가 재시작 후 외부 확인을 기다리는 중이라 새 이벤트를 아직 처리하지 않음.
    이미 처리된 이벤트의 재전송은 A가 저장된 결과를 돌려준다."""


class MissionClient(Protocol):
    async def submit_job(self, request_id: str, pickup: str, dropoff: str) -> dict:
        """신규 Job 요청. 같은 request_id 재전송은 같은 결과를 돌려받아야 한다.
        반환: {"outcome": "accepted"|"rejected", "job_id": str|None, "mission_id": str|None, "reason": str}
        [초안-팀합의필요] request_id 기반 Job 요청 중복 방지는 A와 아직 합의되지 않았다."""

    async def start_mission(self) -> dict:
        """미션 실행 요청. 반환: {"outcome": ..., "mission_id": ..., "reason": ...}"""

    async def send_handoff(self, event_id: str, mission_id: str, step_id: str) -> HandoffOutcome:
        """물품 인계 완료 요청 (INTERFACES §5.2·5.3)."""

    async def get_state(self) -> dict:
        """A가 관리하는 미션·Job·TaskStep 상태 조회 (INTERFACES §11.2)."""
