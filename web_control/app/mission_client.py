"""E → A 요청 경계.

E는 이 인터페이스로만 A(Mission Manager)에게 요청하고, A가 확정한 상태를 받는다.
메서드 이름·인자·결과 값은 A의 실제 코드에 맞춘다.
  - ros2_ws/src/mission_manager/mission_manager/node.py  (register_jobs, handoff, mission_status, step_status,
                                                          job_status, current_goal, plan_status)
  - ros2_ws/src/mission_manager/README.md "D·E 복구 연동 경계" 3번
실제 전송 방식(rclpy 직접 / Gateway / rosbridge)은 DECISIONS P-003 [PENDING]이다.
전송 방식이 정해지면 이 Protocol을 구현하는 어댑터만 추가한다.

작업 등록 (A `register_jobs(steps, start_waypoint_id, cost_revision)`)
  - steps: 주문 1건 = TaskStep 2개(수거·전달). 전달 단계의 prerequisite_ids 에 수거 단계 ID를 넣는다.
    kind 는 "PICKUP" / "DROPOFF" 문자열로 다루고, ROS 어댑터에서 TaskStep.PICKUP=1 / DROPOFF=2 로 바꾼다.
  - 같은 job_id·step_id를 다시 보내면 A는 'duplicate job or step' 입력 오류를 낸다 (같은 결과 재사용이 아님).
  - start_waypoint_id·cost_revision 이 비었거나 미션이 HOME 복귀·완료 단계면 입력 오류.
  - 반환값: A가 C에 보낸 최적화 요청 번호(int).

인계 (A `handoff(event_id, mission_id, step_id)`, INTERFACES §5.3 [확정])
  accepted / rejected  : A가 최초로 처리한 결과 (같은 event_id 재전송 시 같은 결과 재사용)
  conflict             : 같은 event_id 를 다른 step_id 에 사용
  MissionInputError    : 빈 식별자 등 입력 오류 (A의 ValueError)
  MissionTimeout       : 응답 미확인 — 실패로 간주하지 않는다
  MissionRecoveryPending : A가 재시작 후 복구 확인 중이라 아직 처리하지 않음 — 같은 ID로 나중에 재전송
                         (A의 mission_manager.persistence.RecoveryPending 에 대응)

상태 (A → E, INTERFACES §11.2) — 값은 A 코드 그대로(소문자)
  mission_status : idle / active / returning_home / completed / recovery_required
  step status    : pending / executing / waiting_confirmation / completed
  job status     : pending / active / completed
  plan_status    : idle / pending / applied / deferred / rejected / timeout / failed / recovery_required
"""
from typing import Literal, Protocol

HandoffOutcome = Literal["accepted", "rejected", "conflict"]
PICKUP, DROPOFF = "PICKUP", "DROPOFF"


class MissionTimeout(Exception):
    """A의 응답을 받지 못함. A가 처리했는지 알 수 없는 상태."""


class MissionRecoveryPending(Exception):
    """A가 재시작 후 외부 확인을 기다리는 중이라 새 이벤트를 아직 처리하지 않음.
    이미 처리된 이벤트의 재전송은 A가 저장된 결과를 돌려준다."""


class MissionInputError(Exception):
    """A가 입력을 거부함 (A 코드의 ValueError). 메시지는 A가 준 그대로 둔다."""


def job_steps(job_id: str, pickup: str, dropoff: str) -> list[dict]:
    """주문 1건을 A가 받는 TaskStep 쌍으로 만든다 (INTERFACES §4.2, A contract.add_steps)."""
    p, d = f"{job_id}-PICKUP", f"{job_id}-DROPOFF"
    return [
        {"step_id": p, "job_id": job_id, "kind": PICKUP, "waypoint_id": pickup, "prerequisite_ids": []},
        {"step_id": d, "job_id": job_id, "kind": DROPOFF, "waypoint_id": dropoff, "prerequisite_ids": [p]},
    ]


class MissionClient(Protocol):
    async def register_jobs(self, steps: list[dict], start_waypoint_id: str, cost_revision: str) -> int:
        """작업 등록. A `register_jobs`와 같은 의미. 입력 오류는 MissionInputError."""

    async def handoff(self, event_id: str, mission_id: str, step_id: str) -> HandoffOutcome:
        """물품 인계 완료 요청 (INTERFACES §5.2·5.3). A `handoff`와 같은 의미."""

    async def request_mission_run(self, mission_id: str) -> dict:
        """미션 실행 요청 (INTERFACES §11.1 "미션 실행 요청", §4.6 "A가 준비 상태를 종합하여 미션 실행 허용 여부를 결정").
        반환 {"outcome": "accepted"|"rejected", "reason": str}. A가 허용하기 전에는 등록된 작업이 있어도 출발하지 않는다.
        [초안 · A와 합의 전] 호출 이름·필드·결과 값은 A 구현에 맞춰 바꾼다. 같은 요청을 다시 보내도 결과가 같아야 한다."""

    async def snapshot(self) -> dict:
        """A가 관리하는 상태 조회 (INTERFACES §11.2).
        {"mission_id", "mission_status", "plan_status", "active_version", "cost_revision", "recovery_required",
         "current_goal": {"step_id"|None, "waypoint_id", "plan_version"} | None, "waiting_confirmation",
         "order": [step_id...], "steps": [{TaskStep..., "status"}], "jobs": [{"job_id", "status"}]}
        [초안-팀합의필요] 묶음 형식은 A와 아직 합의되지 않았다. 필드 의미는 A 코드의 조회 메서드를 따른다."""
