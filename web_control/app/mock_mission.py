"""A(Mission Manager) 테스트 대역 — MOCK 전용. ⚠ 운영 코드가 아니다.

A가 연결되기 전 E의 기능(요청·기록·재전송·화면)을 검증하기 위한 대역이다.
A의 실제 코드와 같은 이름·순서·결과 값을 쓴다 (2026-11-23 main 92ce3eb 기준).
  - ros2_ws/src/mission_manager/mission_manager/contract.py   add_steps, request, apply, start_next, arrive,
                                                              confirm, handoff, start_home_return, arrive_home
  - ros2_ws/src/mission_manager/mission_manager/node.py       register_jobs, claim_next_goal, mission_status,
                                                              step_status, job_status, current_goal, plan_status
  - ros2_ws/src/mission_manager/mission_manager/persistence.py 복구 중 새 인계 → RecoveryPending

문서 기준으로 넣은 것 (A 코드에는 아직 없음):
  - 미션 실행 요청 (INTERFACES §11.1, §4.6). A가 실행을 허용하기 전에는 작업이 등록돼도 다음 목표를 넘기지 않는다
    (claim_next_goal → None). 허용 조건: 등록된 작업 있음, 완료 전, 복구 확인 중 아님, 준비 상태 정상(ready).
    호출 이름·결과 값은 A 구현이 올라오면 그대로 맞춘다.

A와 다른 점 (대역이라 단순화):
  - 방문 순서 최적화(C)는 하지 않는다. 남은 단계를 등록 순서 그대로 계획한다
    (A의 test/mock_optimizer.py 처럼 받은 순서를 그대로 돌려주는 수준). 계획은 즉시 승인된다.
  - 주행(D)은 지점 사이를 일정 시간에 걸쳐 이동한 것으로 흉내만 낸다.
  - 미션 ID는 실행마다 하나다 (A는 DB 하나 = 미션 하나, 기본 uuid4).

결함 주입 (테스트·시연용):
  drop_next_ack     : 다음 요청을 **처리한 뒤** 응답을 잃어버린다 (MissionTimeout)
  timeout_next      : 다음 요청을 **처리하지 않고** 타임아웃
  recovery_required : A 재시작 후 복구 확인 중 상태. A persistence.py 와 같게
                      이미 처리한 event_id 는 저장된 결과를 돌려주고, 새 event_id 는 MissionRecoveryPending
"""
import asyncio
import math
import time
from uuid import uuid4
from graphlib import CycleError, TopologicalSorter

from .mission_client import DROPOFF, PICKUP, MissionInputError, MissionRecoveryPending, MissionTimeout

ROBOT_SPEED = 0.22  # m/s


def _new_run_id() -> str:
    """실행마다 전체 UUID(16진수 32자리)를 쓴다. 같은 DB로 재시작한 이전 실행과 미션 ID가 겹치지 않게 한다.
    (시각·짧은 무작위 조합은 같은 초에 재시작하면 앞자리가 겹칠 수 있어 쓰지 않는다 — PR #18 리뷰)"""
    return uuid4().hex.upper()


class MockMissionManager:
    def __init__(self, waypoints: dict, speed_scale: float = 4.0, auto_run: bool = True,
                 run_id: str | None = None):
        # 실행마다 다른 run_id를 미션 ID에 넣는다. 같은 DB로 MOCK 서버를 재시작해도 새 미션 ID가
        # 이전 실행의 인계 기록과 겹쳐 409로 막히지 않게 한다. run_id="" 이면 접두어 없음(테스트용).
        self.run_id = _new_run_id() if run_id is None else run_id
        self.mission_id = f"MISSION-MOCK-{self.run_id}" if self.run_id else "MISSION-MOCK"
        self.wps = waypoints
        self.scale = speed_scale
        self.auto_run = auto_run
        # ---- A contract.MissionState 와 같은 필드
        self.steps: dict[str, dict] = {}            # step_id → TaskStep (등록 순서 유지)
        self.completed: set[str] = set()
        self.current: str | None = None
        self.waiting = False
        self.order: list[str] = []
        self.active_version = 0
        self.cost_revision: str | None = None
        self.latest_request_id = 0
        self.home_status = "away"
        self.handoff_results: dict[str, tuple[str, str]] = {}  # event_id → (step_id, outcome)
        # ---- A node 와 같은 필드
        self.plan_status = "idle"
        self.candidate: list[str] | None = None     # 운행 중 받은 계획 (현재 단계 인계 후 적용)
        self.recovery_required = False
        self.run_authorized = False   # 미션 실행 허용 여부 (문서 §11.1·§4.6, A 코드 미구현)
        self.ready = True             # 준비 상태 (MOCK: 결함 주입으로 False 가능)
        # ---- 검증용
        self.handoff_accept_count: dict[str, int] = {}   # 단계별 완료 전이 횟수
        # ---- 로봇 흉내
        self.at_wp = "HOME" if "HOME" in waypoints else next(iter(waypoints))
        self.robot = dict(self._xy(self.at_wp))
        self.move: dict | None = None
        # ---- 결함 주입
        self.drop_next_ack = False
        self.timeout_next = False
        self._task: asyncio.Task | None = None

    # ---------------------------------------------------------------- 수명
    async def start(self):
        if self.auto_run:
            self._task = asyncio.create_task(self._loop())

    async def stop(self):
        if self._task:
            self._task.cancel()

    def _fault(self, process):
        """process(): 실제 처리. drop_next_ack 면 처리 후 응답 유실."""
        if self.timeout_next:
            self.timeout_next = False
            raise MissionTimeout("[MOCK] 응답 시간 초과 (A는 처리하지 않음)")
        result = process()
        if self.drop_next_ack:
            self.drop_next_ack = False
            raise MissionTimeout("[MOCK] 응답 유실 (A는 이미 처리함)")
        return result

    # ---------------------------------------------------------------- MissionClient 구현
    async def register_jobs(self, steps, start_waypoint_id, cost_revision):
        return self._fault(lambda: self._register_jobs(steps, start_waypoint_id, cost_revision))

    async def handoff(self, event_id, mission_id, step_id):
        return self._fault(lambda: self._handoff(event_id, mission_id, step_id))

    async def request_mission_run(self, mission_id):
        return self._fault(lambda: self._request_mission_run(mission_id))

    async def snapshot(self):
        return self._snapshot()

    # ---------------------------------------------------------------- 미션 실행 요청 (문서 기준)
    def _request_mission_run(self, mission_id):
        if not isinstance(mission_id, str) or not mission_id.strip():
            raise MissionInputError("mission_id must be a nonempty string")
        if mission_id != self.mission_id:
            return {"outcome": "rejected", "reason": "다른 미션"}
        if self.work_completed:
            return {"outcome": "rejected", "reason": "이미 완료된 미션"}   # 완료 뒤에는 실행 허용 여부와 관계없이 거부
        if self.run_authorized:
            return {"outcome": "accepted", "reason": "이미 실행 중"}   # 같은 요청 재전송 → 같은 결과
        if self.recovery_required:
            return {"outcome": "rejected", "reason": "재시작 후 복구 확인 중"}
        if not self.steps:
            return {"outcome": "rejected", "reason": "등록된 작업 없음"}
        if not self.ready:
            return {"outcome": "rejected", "reason": "준비 상태 미확인 (INTERFACES §4.6)"}
        self.run_authorized = True
        return {"outcome": "accepted", "reason": ""}

    # ---------------------------------------------------------------- A node.register_jobs
    def _register_jobs(self, steps, start_waypoint_id, cost_revision):
        """Order intake boundary; while busy, plan from the current destination."""
        if self.work_completed:
            raise MissionInputError("mission has reached its HOME return or completion phase")
        if self.current:
            start_waypoint_id = self.steps[self.current]["waypoint_id"]
        if not start_waypoint_id.strip() or not cost_revision.strip():
            raise MissionInputError("missing start waypoint or cost revision")
        self._add_steps(steps)
        return self._request_plan(cost_revision)

    def _add_steps(self, steps):
        """A contract.add_steps: 수거·전달 쌍을 한 번에 검증한 뒤 등록."""
        additions, jobs = {}, {}
        known_jobs = {s["job_id"] for s in self.steps.values()}
        for step in steps:
            if not all(str(step.get(k, "")).strip() for k in ("job_id", "step_id", "waypoint_id")):
                raise MissionInputError("missing job, step or waypoint ID")
            if step["step_id"] in self.steps or step["step_id"] in additions or step["job_id"] in known_jobs:
                raise MissionInputError("duplicate job or step")
            pair = jobs.setdefault(step["job_id"], {})
            if step["kind"] in pair or step["kind"] not in (PICKUP, DROPOFF):
                raise MissionInputError("invalid or repeated step kind")
            pair[step["kind"]] = step
            additions[step["step_id"]] = dict(step, prerequisite_ids=list(step.get("prerequisite_ids", [])))
        for pair in jobs.values():
            pickup, dropoff = pair.get(PICKUP), pair.get(DROPOFF)
            if not pickup or not dropoff or pickup["step_id"] not in dropoff.get("prerequisite_ids", []):
                raise MissionInputError("delivery requires its own pickup")
        for step in additions.values():
            if not set(step["prerequisite_ids"]) <= self.steps.keys() | additions.keys():
                raise MissionInputError("unknown prerequisite")
        try:
            TopologicalSorter({k: s["prerequisite_ids"] for k, s in additions.items()}).prepare()
        except CycleError as e:
            raise MissionInputError("cyclic prerequisites") from e
        self.steps.update(additions)

    def _request_plan(self, cost_revision):
        """A node.request_plan + MOCK 최적화: 남은 단계를 등록 순서대로. 운행 중이면 보류(deferred)."""
        self.latest_request_id += 1
        self.cost_revision = cost_revision
        remaining = [s for s in self.steps if s not in self.completed and s != self.current]
        if self.current:
            self.candidate, self.plan_status = remaining, "deferred"
        else:
            self._apply(remaining)
        return self.latest_request_id

    def _apply(self, order):
        self.order = list(order)
        self.active_version += 1
        self.candidate = None
        self.plan_status = "applied"

    # ---------------------------------------------------------------- A 상태 전이
    @property
    def work_completed(self):
        return bool(self.steps) and self.completed == self.steps.keys()

    @property
    def mission_status(self):
        if self.recovery_required:
            return "recovery_required"
        if self.work_completed:
            return "completed" if self.home_status == "home" else "returning_home"
        return "active" if self.steps else "idle"

    def step_status(self, step_id):
        if step_id not in self.steps:
            raise KeyError(step_id)
        if step_id in self.completed:
            return "completed"
        if step_id == self.current:
            return "waiting_confirmation" if self.waiting else "executing"
        return "pending"

    def job_status(self, job_id):
        statuses = [self.step_status(sid) for sid, s in self.steps.items() if s["job_id"] == job_id]
        if not statuses:
            raise KeyError(job_id)
        if all(x == "completed" for x in statuses):
            return "completed"
        return "active" if any(x != "pending" for x in statuses) else "pending"

    def current_goal(self):
        if self.current is not None:
            return {"mission_id": self.mission_id, "step_id": self.current, "plan_version": self.active_version,
                    "waypoint_id": self.steps[self.current]["waypoint_id"]}
        if self.home_status == "returning":
            return {"mission_id": self.mission_id, "step_id": None, "plan_version": self.active_version,
                    "waypoint_id": "HOME"}
        return None

    def claim_next_goal(self):
        """A node.claim_next_goal: 다음 목표 하나를 선점 (D 어댑터 역할을 MOCK 주행이 대신 호출)."""
        if not self.run_authorized:          # 문서 기준: 실행 허용 전에는 목표를 넘기지 않음
            return None
        if self.recovery_required or self.current_goal() is not None:
            return None
        if self.candidate is not None:
            self._apply(self.candidate)
        if self.order:
            self.current = self.order.pop(0)
        elif self.mission_status == "returning_home" and self.home_status == "away":
            self.home_status = "returning"
        return self.current_goal()

    def arrive(self, step_id):
        """D의 도착 결과 주입 (A contract.arrive)."""
        if not self.current or self.current != step_id:
            return False
        self.waiting = True
        self.at_wp = self.steps[step_id]["waypoint_id"]
        self.robot, self.move = dict(self._xy(self.at_wp)), None
        return True

    def arrive_home(self):
        if self.home_status != "returning":
            return False
        self.home_status = "home"
        self.at_wp = "HOME"
        self.robot, self.move = dict(self._xy("HOME")), None
        return True

    def _confirm(self, mission_id, step_id):
        if mission_id != self.mission_id or step_id != self.current or not self.waiting:
            return False
        self.completed.add(step_id)
        self.handoff_accept_count[step_id] = self.handoff_accept_count.get(step_id, 0) + 1
        self.home_status = "home" if self.steps[step_id]["waypoint_id"] == "HOME" else "away"
        self.current, self.waiting = None, False
        return True

    def _handoff(self, event_id, mission_id, step_id):
        # A node.handoff: 빈 식별자는 입력 오류
        if not all(isinstance(v, str) and v.strip() for v in (event_id, mission_id, step_id)):
            raise MissionInputError("event_id, mission_id and step_id must be nonempty strings")
        # A persistence.handoff_transition: 원장에 없는 새 이벤트는 복구 확인 중이면 처리하지 않는다
        if event_id not in self.handoff_results and self.recovery_required:
            raise MissionRecoveryPending("[MOCK] A 재시작 후 외부 확인 대기 — 새 인계는 아직 처리하지 않음")
        # A contract.MissionState.handoff
        if mission_id != self.mission_id:
            return "rejected"
        if event_id in self.handoff_results:
            original_step, outcome = self.handoff_results[event_id]
            return outcome if original_step == step_id else "conflict"
        completed_now = self._confirm(mission_id, step_id)
        outcome = "accepted" if completed_now else "rejected"
        self.handoff_results[event_id] = (step_id, outcome)
        # A node.handoff: 인계가 끝나면 보류된 계획을 적용
        if completed_now and self.candidate is not None:
            self._apply(self.candidate)
        return outcome

    # ---------------------------------------------------------------- 상태 (A → E)
    def _snapshot(self):
        steps = [{**s, "status": self.step_status(sid)} for sid, s in self.steps.items()]
        job_ids = list(dict.fromkeys(s["job_id"] for s in self.steps.values()))
        return {
            "source": "MOCK",
            "mission_id": self.mission_id,
            "mission_status": self.mission_status,
            "plan_status": self.plan_status,
            "active_version": self.active_version,
            "cost_revision": self.cost_revision,
            "recovery_required": self.recovery_required,
            "run_authorized": self.run_authorized,
            "ready": self.ready,
            "current_goal": self.current_goal(),
            "waiting_confirmation": self.waiting,
            "order": list(self.order),
            "deferred_order": list(self.candidate) if self.candidate is not None else None,
            "steps": steps,
            "jobs": [{"job_id": j, "status": self.job_status(j)} for j in job_ids],
            "robot": {**self.robot, "at": self.at_wp, "moving_to": self.move["to"] if self.move else None},
        }

    # ---------------------------------------------------------------- 주행 흉내 (auto_run)
    def _xy(self, wp):
        w = self.wps[wp]
        return {"x": w["x"], "y": w["y"]}

    def _begin_move(self, to):
        a, b = self.robot, self._xy(to)
        dur = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / ROBOT_SPEED / self.scale
        self.move = {"from": dict(a), "to": to, "t0": time.monotonic(), "dur": max(dur, 0.3)}
        self.at_wp = None

    def tick(self):
        if self.move:
            m = self.move
            k = min(1.0, (time.monotonic() - m["t0"]) / m["dur"])
            b = self._xy(m["to"])
            self.robot = {"x": m["from"]["x"] + (b["x"] - m["from"]["x"]) * k,
                          "y": m["from"]["y"] + (b["y"] - m["from"]["y"]) * k}
            if k < 1.0:
                return
            self.move, self.at_wp = None, m["to"]
            goal = self.current_goal()
            if goal and goal["waypoint_id"] == m["to"]:
                if goal["step_id"]:
                    self.arrive(goal["step_id"])
                else:
                    self.arrive_home()
            return
        if self.waiting:
            return
        goal = self.current_goal() or self.claim_next_goal()
        if goal:
            if self.at_wp == goal["waypoint_id"]:     # 이미 그 지점에 있음 → 바로 도착 처리
                self.arrive(goal["step_id"]) if goal["step_id"] else self.arrive_home()
            else:
                self._begin_move(goal["waypoint_id"])

    async def _loop(self):
        while True:
            self.tick()
            await asyncio.sleep(0.1)
