"""MOCK A 대역이 실제 A 코드와 같은 결과를 내는지 대조한다.

같은 시나리오를 두 쪽에 똑같이 실행하고 매 단계 결과를 비교한다.
  - 실제 A: ros2_ws의 mission_manager.persistence.PersistentMission(노드가 쓰는 상태 저장소, 메모리 DB)과
            mission_manager.node.MissionManager의 상태 함수(mission_status, step_status, job_status)
            + A 테스트와 같은 '받은 순서 그대로 돌려주는' 최적화 응답
  - E 대역: app.mock_mission.MockMissionManager

ROS2 Jazzy 워크스페이스를 빌드·source한 터미널에서만 실행된다 (없으면 건너뜀):
  source /opt/ros/jazzy/setup.bash && source ../ros2_ws/install/setup.bash
  python3 -m pytest -q tests/test_a_parity.py
"""
import pytest

contract = pytest.importorskip("mission_manager.contract", reason="ROS2 워크스페이스(mission_manager)가 source되지 않음")
node_mod = pytest.importorskip("mission_manager.node")
persistence = pytest.importorskip("mission_manager.persistence")
from logistics_msgs.msg import TaskStep  # noqa: E402
from logistics_msgs.srv import OptimizeRoute  # noqa: E402

from app.main import load_waypoints  # noqa: E402
from app.mission_client import MissionInputError, job_steps  # noqa: E402
from app.mock_mission import MockMissionManager  # noqa: E402

MissionManager = node_mod.MissionManager
KIND = {"PICKUP": TaskStep.PICKUP, "DROPOFF": TaskStep.DROPOFF}
MID = "MISSION-PARITY"


def echo_optimizer(request):
    """A test/mock_optimizer.py 와 같은 응답: 받은 순서를 그대로 돌려준다."""
    return OptimizeRoute.Response(
        success=True, method="test_echo", mission_id=request.mission_id, request_id=request.request_id,
        base_version=request.base_version, cost_revision=request.cost_revision, estimated_distance_m=1.0,
        ordered_step_ids=[s.step_id for s in request.pending_steps])


class RealA:
    """A node.py 의 register_jobs / claim_next_goal / handoff 순서를 그대로 따라 실제 contract 를 구동한다.
    상태 값은 A node 의 함수를 그대로 빌려 쓴다."""
    mission_status = MissionManager.mission_status
    step_status = MissionManager.step_status
    job_status = MissionManager.job_status

    def __init__(self):
        self.state = persistence.PersistentMission(":memory:", MID)   # A 노드와 같은 상태 저장소
        self.candidate = None
        self.plan_status = "idle"

    def register_jobs(self, steps, start, cost):
        if self.state.work_completed:
            raise ValueError("mission has reached its HOME return or completion phase")
        if self.state.current:
            start = self.state.steps[self.state.current].waypoint_id
        if not start.strip() or not cost.strip():
            raise ValueError("missing start waypoint or cost revision")
        self.state.add_steps([TaskStep(step_id=s["step_id"], job_id=s["job_id"], kind=KIND[s["kind"]],
                                       waypoint_id=s["waypoint_id"], prerequisite_ids=s["prerequisite_ids"])
                              for s in steps])
        result = echo_optimizer(self.state.request(start, cost))
        assert self.state.valid_candidate(result)
        if self.state.current:
            self.candidate, self.plan_status = result, "deferred"
        else:
            self._apply(result)

    def _apply(self, result):
        assert self.state.apply(result)
        self.candidate, self.plan_status = None, "applied"

    def current_goal(self):
        if self.state.current is not None:
            return (self.state.current, self.state.steps[self.state.current].waypoint_id)
        if self.state.home_status == "returning":
            return (None, "HOME")
        return None

    def claim_next_goal(self):
        if self.current_goal() is not None:
            return None
        if self.candidate is not None:
            self._apply(self.candidate)
        if self.state.order:
            self.state.start_next()
        elif self.mission_status == "returning_home":
            self.state.start_home_return()
        return self.current_goal()

    def handoff(self, event_id, mission_id, step_id):
        if not all(isinstance(v, str) and v.strip() for v in (event_id, mission_id, step_id)):
            raise ValueError("event_id, mission_id and step_id must be nonempty strings")
        outcome, completed_now = self.state.handoff_transition(event_id, mission_id, step_id)
        if completed_now and self.candidate is not None:
            self._apply(self.candidate)
        return outcome

    def view(self):
        return {"mission": self.mission_status, "plan": self.plan_status, "version": self.state.active_version,
                "goal": self.current_goal(), "waiting": self.state.waiting, "order": list(self.state.order),
                "steps": {k: self.step_status(k) for k in self.state.steps},
                "jobs": {j: self.job_status(j) for j in dict.fromkeys(s.job_id for s in self.state.steps.values())}}


class MockA:
    def __init__(self):
        self.m = MockMissionManager(load_waypoints(), auto_run=False, run_id="")
        self.m.mission_id = MID
        self.m.run_authorized = True   # 실제 A 코드에는 실행 허용 단계가 아직 없음 → 대조할 때는 허용된 상태로 비교

    def register_jobs(self, steps, start, cost):
        try:
            self.m._register_jobs(steps, start, cost)
        except MissionInputError as e:
            raise ValueError(str(e)) from e

    def claim_next_goal(self):
        g = self.m.claim_next_goal()
        return (g["step_id"], g["waypoint_id"]) if g else None

    def handoff(self, *args):
        try:
            return self.m._handoff(*args)
        except MissionInputError as e:
            raise ValueError(str(e)) from e

    def arrive(self, step_id):
        return self.m.arrive(step_id)

    def view(self):
        g = self.m.current_goal()
        return {"mission": self.m.mission_status, "plan": self.m.plan_status, "version": self.m.active_version,
                "goal": (g["step_id"], g["waypoint_id"]) if g else None, "waiting": self.m.waiting,
                "order": list(self.m.order), "steps": {k: self.m.step_status(k) for k in self.m.steps},
                "jobs": {j: self.m.job_status(j) for j in dict.fromkeys(s["job_id"] for s in self.m.steps.values())}}


def run(a, ops):
    out = []
    for op, *args in ops:
        try:
            if op == "register":
                res = a.register_jobs(*args)
            elif op == "claim":
                res = a.claim_next_goal()
            elif op == "arrive":
                res = a.state.arrive(*args) if isinstance(a, RealA) else a.arrive(*args)
            elif op == "arrive_home":
                res = a.state.arrive_home() if isinstance(a, RealA) else a.m.arrive_home()
            elif op == "handoff":
                res = a.handoff(*args)
        except ValueError as e:
            res = ("ValueError", str(e))
        out.append((op, args, res if op != "register" else None, a.view()))
    return out


J1 = job_steps("JOB-1", "PICKUP_1", "DROP_1")
J2 = job_steps("JOB-2", "DROP_1", "HOME")
J3 = job_steps("JOB-3", "PICKUP_1", "HOME")

SCENARIOS = {
    "two_jobs_with_replan_and_home_return": [
        ("register", J1, "HOME", "r1"),
        ("claim",),
        ("handoff", "early", MID, "JOB-1-PICKUP"),          # 도착 전 → rejected
        ("arrive", "JOB-1-PICKUP"),
        ("register", J2, "HOME", "r1"),                      # 운행 중 → deferred
        ("handoff", "e1", MID, "JOB-1-PICKUP"),             # accepted + 보류 계획 적용
        ("handoff", "e1", MID, "JOB-1-PICKUP"),             # 재전송 → 같은 결과
        ("handoff", "e1", MID, "JOB-1-DROPOFF"),            # 같은 키 다른 단계 → conflict
        ("handoff", "x", "OTHER-MISSION", "JOB-1-PICKUP"),  # 다른 미션 → rejected
        ("claim",), ("arrive", "JOB-1-DROPOFF"), ("handoff", "e2", MID, "JOB-1-DROPOFF"),
        ("claim",), ("arrive", "JOB-2-PICKUP"), ("handoff", "e3", MID, "JOB-2-PICKUP"),
        ("claim",), ("arrive", "JOB-2-DROPOFF"), ("handoff", "e4", MID, "JOB-2-DROPOFF"),  # HOME 하역 → completed
        ("register", J3, "HOME", "r1"),                      # 완료 후 등록 → 입력 오류
        ("claim",),
    ],
    "last_dropoff_away_needs_home_return": [
        ("register", J1, "HOME", "r1"),
        ("claim",), ("arrive", "JOB-1-PICKUP"), ("handoff", "a", MID, "JOB-1-PICKUP"),
        ("claim",), ("arrive", "JOB-1-DROPOFF"), ("handoff", "b", MID, "JOB-1-DROPOFF"),  # returning_home
        ("claim",),                                           # HOME 복귀 목표 (step 없음)
        ("handoff", "c", MID, "JOB-1-DROPOFF"),              # 이미 완료 → rejected
        ("arrive_home",), ("claim",),
    ],
    "input_errors": [
        ("register", J1, "HOME", ""),                         # cost_revision 없음
        ("register", J1, "HOME", "r1"),
        ("register", J1, "HOME", "r1"),                       # 같은 job 재등록 → duplicate
        ("register", [J2[1]], "HOME", "r1"),                  # 수거 없는 전달
        ("claim",), ("arrive", "JOB-1-PICKUP"),
        ("handoff", "", MID, "JOB-1-PICKUP"),                 # 빈 event_id
        ("handoff", "ok", MID, "JOB-1-PICKUP"),
    ],
}


@pytest.mark.parametrize("name", list(SCENARIOS))
def test_mock_matches_real_a(name):
    real, mock = run(RealA(), SCENARIOS[name]), run(MockA(), SCENARIOS[name])
    for (op, args, r_res, r_view), (_, _, m_res, m_view) in zip(real, mock):
        step = f"{name}: {op} {args[:3] if op != 'register' else args[1:]}"
        if isinstance(r_res, tuple) and r_res and r_res[0] == "ValueError":
            assert isinstance(m_res, tuple) and m_res[0] == "ValueError", step
            assert m_res[1] == r_res[1], step
        else:
            assert m_res == r_res, step
        assert m_view == r_view, step
