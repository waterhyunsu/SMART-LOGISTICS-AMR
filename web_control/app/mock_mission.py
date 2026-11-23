"""A(Mission Manager) 테스트 대역 — MOCK 전용. ⚠ 운영 코드가 아니다.

A가 준비되기 전 E의 기능(요청·기록·재전송·화면)을 검증하기 위한 대역이다.
인계 처리 의미는 A의 실제 구현 `ros2_ws/src/mission_manager/mission_manager/contract.py`
`MissionState.handoff()` 를 따른다:
  - event_id 가 비었거나 mission_id 가 다르면 rejected
  - 같은 event_id 가 다시 오면 최초 결과 재사용, 다른 step_id 면 conflict
  - 현재 인계 대기 단계일 때만 accepted
방문 순서 계산(C)은 하지 않는다: 등록 순서 그대로 사용한다 (A의 test/mock_optimizer.py 와 같은 수준).
로봇 이동(D)은 지점 사이를 일정 시간에 걸쳐 이동한 것으로 흉내만 낸다.

결함 주입 (테스트·시연용):
  drop_next_ack     : 다음 요청을 **처리한 뒤** 응답을 잃어버린다 (MissionTimeout)
  timeout_next      : 다음 요청을 **처리하지 않고** 타임아웃
  recovery_required : A 재시작 후 복구 확인 중 상태. A persistence.py 와 같게
                      이미 처리한 event_id 는 저장된 결과를 돌려주고, 새 event_id 는 MissionRecoveryPending
"""
import asyncio
import math
import time
import uuid

from .mission_client import MissionRecoveryPending, MissionTimeout

PICKUP, DROPOFF = "PICKUP", "DROPOFF"
ROBOT_SPEED = 0.22  # m/s


def _new_run_id() -> str:
    """시작 시각(초, 36진수) + 무작위 2자리. 같은 DB를 쓰는 이전 실행과 겹치지 않게 한다."""
    t, digits, out = int(time.time()), "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ", ""
    while t:
        t, r = divmod(t, 36)
        out = digits[r] + out
    return out + uuid.uuid4().hex[:2].upper()


class MockMissionManager:
    def __init__(self, waypoints: dict, speed_scale: float = 4.0, auto_run: bool = True,
                 run_id: str | None = None):
        # 실행마다 다른 run_id를 ID에 넣는다. 같은 DB로 MOCK 서버를 재시작해도 새 미션·Job·단계 ID가
        # 이전 실행의 인계 기록과 겹쳐 409로 막히지 않게 한다. run_id="" 이면 접두어 없음(테스트용).
        self.run_id = _new_run_id() if run_id is None else run_id
        self._id_prefix = f"{self.run_id}-" if self.run_id else ""
        self.wps = waypoints
        self.scale = speed_scale
        self.auto_run = auto_run
        self.mission_seq = 0
        self.job_seq = 0
        self.mission_id: str | None = None
        self.mission_status = "IDLE"
        self.jobs: dict[str, dict] = {}            # job_id → {job_id, pickup, dropoff, mission_id}
        self.steps: dict[str, dict] = {}           # step_id → {step_id, job_id, kind, waypoint_id, prerequisite_ids}
        self.completed: set[str] = set()
        self.current: str | None = None
        self.waiting = False
        self.order: list[str] = []
        self.unplanned: list[str] = []             # 운행 중 추가된 단계 (현재 단계 완료 후 반영)
        self.active_version = 0
        self.handoff_results: dict[str, tuple[str, str]] = {}  # event_id → (step_id, outcome)
        self.job_requests: dict[str, dict] = {}    # request_id → 결과 (중복 요청 방지)
        self.handoff_accept_count: dict[str, int] = {}  # 검증용: 단계별 완료 전이 횟수
        # 로봇 흉내
        self.at_wp = "HOME"
        self.robot = dict(self._xy("HOME"))
        self.move: dict | None = None
        # 결함 주입
        self.drop_next_ack = False
        self.timeout_next = False
        self.recovery_required = False
        self._task: asyncio.Task | None = None

    # ---------------------------------------------------------------- 수명
    async def start(self):
        if self.auto_run:
            self._task = asyncio.create_task(self._loop())

    async def stop(self):
        if self._task:
            self._task.cancel()

    # ---------------------------------------------------------------- 결함 주입
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
    async def submit_job(self, request_id, pickup, dropoff):
        return self._fault(lambda: self._register_job(request_id, pickup, dropoff))

    async def start_mission(self):
        return self._fault(self._start_mission)

    async def send_handoff(self, event_id, mission_id, step_id):
        return self._fault(lambda: self._handoff(event_id, mission_id, step_id))

    async def get_state(self):
        return self._state()

    # ---------------------------------------------------------------- A 로직 흉내
    def _register_job(self, request_id, pickup, dropoff):
        if request_id in self.job_requests:      # 같은 요청 재전송 → 같은 결과
            return self.job_requests[request_id]
        if pickup not in self.wps or dropoff not in self.wps or pickup == dropoff:
            res = {"outcome": "rejected", "job_id": None, "mission_id": None, "reason": "유효하지 않은 지점"}
        else:
            self.job_seq += 1
            jid = f"JOB-{self._id_prefix}{self.job_seq:03d}"
            running = self.mission_status == "RUNNING"
            self.jobs[jid] = {"job_id": jid, "pickup": pickup, "dropoff": dropoff,
                              "mission_id": self.mission_id if running else None}
            p, d = f"{jid}-PICKUP", f"{jid}-DROPOFF"
            self.steps[p] = {"step_id": p, "job_id": jid, "kind": PICKUP, "waypoint_id": pickup, "prerequisite_ids": []}
            self.steps[d] = {"step_id": d, "job_id": jid, "kind": DROPOFF, "waypoint_id": dropoff, "prerequisite_ids": [p]}
            if running:
                self.unplanned += [p, d]
            res = {"outcome": "accepted", "job_id": jid, "mission_id": self.jobs[jid]["mission_id"], "reason": ""}
        self.job_requests[request_id] = res
        return res

    def _start_mission(self):
        if self.mission_status == "RUNNING":
            return {"outcome": "rejected", "mission_id": self.mission_id, "reason": "이미 운행 중"}
        pending = [j for j in self.jobs.values() if j["mission_id"] is None]
        if not pending:
            return {"outcome": "rejected", "mission_id": None, "reason": "대기 중인 Job 없음"}
        self.mission_seq += 1
        self.mission_id = f"MISSION-MOCK-{self._id_prefix}{self.mission_seq:03d}"
        for j in pending:
            j["mission_id"] = self.mission_id
        self.order = [s for j in pending for s in (f"{j['job_id']}-PICKUP", f"{j['job_id']}-DROPOFF")]
        self.unplanned = []
        self.active_version = 1
        self.mission_status = "RUNNING"
        return {"outcome": "accepted", "mission_id": self.mission_id, "reason": ""}

    def _confirm(self, mission_id, step_id):
        if mission_id != self.mission_id or step_id != self.current or not self.waiting:
            return False
        self.completed.add(step_id)
        self.handoff_accept_count[step_id] = self.handoff_accept_count.get(step_id, 0) + 1
        self.current, self.waiting = None, False
        return True

    def _handoff(self, event_id, mission_id, step_id):
        # A persistence.handoff_transition 과 같은 순서:
        # 원장에 없는 새 이벤트는 복구 확인 중이면 처리하지 않는다
        if event_id not in self.handoff_results and self.recovery_required:
            raise MissionRecoveryPending("[MOCK] A 재시작 후 외부 확인 대기 — 새 인계는 아직 처리하지 않음")
        # A contract.MissionState.handoff
        if not event_id.strip() or mission_id != self.mission_id:
            return "rejected"
        if event_id in self.handoff_results:
            original_step, outcome = self.handoff_results[event_id]
            return outcome if original_step == step_id else "conflict"
        outcome = "accepted" if self._confirm(mission_id, step_id) else "rejected"
        self.handoff_results[event_id] = (step_id, outcome)
        return outcome

    def arrive(self, step_id):
        """D의 도착 결과 주입 (테스트용)."""
        if self.current == step_id:
            self.waiting = True
            self.at_wp = self.steps[step_id]["waypoint_id"]
            self.robot = dict(self._xy(self.at_wp))
            self.move = None
            return True
        return False

    def start_next(self):
        if self.current is None and self.unplanned:   # 신규 주문은 현재 단계 완료 후 반영
            self.order += self.unplanned
            self.unplanned = []
            self.active_version += 1
        if self.current is None and self.order:
            self.current = self.order.pop(0)
        return self.current

    # ---------------------------------------------------------------- 상태 (A → E)
    def _step_status(self, sid):
        if sid in self.completed:
            return "COMPLETED"
        if sid == self.current:
            return "WAITING_CONFIRMATION" if self.waiting else "IN_PROGRESS"
        if set(self.steps[sid]["prerequisite_ids"]) <= self.completed:
            return "READY"
        return "PENDING"

    def _state(self):
        steps = [{**s, "status": self._step_status(sid), "mission_id": self.jobs[s["job_id"]]["mission_id"]}
                 for sid, s in self.steps.items()]
        return {
            "source": "MOCK",
            "mission_id": self.mission_id,
            "mission_status": self.mission_status,
            "active_version": self.active_version,
            "current_step_id": self.current,
            "waiting_confirmation": self.waiting,
            "plan": list(self.order),
            "pending_replan": list(self.unplanned),
            "jobs": list(self.jobs.values()),
            "steps": steps,
            "recovery_required": self.recovery_required,
            "run_id": self.run_id,
            "robot": {**self.robot, "at": self.at_wp, "moving_to": self.move["to"] if self.move else None},
        }

    # ---------------------------------------------------------------- 로봇 흉내 (auto_run)
    def _xy(self, wp):
        w = self.wps[wp]
        return {"x": w["x"], "y": w["y"]}

    def _begin_move(self, to):
        a, b = self.robot, self._xy(to)
        dur = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / ROBOT_SPEED / self.scale
        self.move = {"from": dict(a), "to": to, "t0": time.monotonic(), "dur": max(dur, 0.3)}
        self.at_wp = None

    def tick(self):
        if self.mission_status != "RUNNING":
            return
        if self.move:
            m = self.move
            k = min(1.0, (time.monotonic() - m["t0"]) / m["dur"])
            b = self._xy(m["to"])
            self.robot = {"x": m["from"]["x"] + (b["x"] - m["from"]["x"]) * k,
                          "y": m["from"]["y"] + (b["y"] - m["from"]["y"]) * k}
            if k < 1.0:
                return
            self.move, self.at_wp = None, m["to"]
            if self.current and self.steps[self.current]["waypoint_id"] == m["to"]:
                self.arrive(self.current)
            return
        if self.current is None:
            if self.start_next():
                self._begin_move(self.steps[self.current]["waypoint_id"])
            elif self.at_wp != "HOME":
                self._begin_move("HOME")
            else:
                self.mission_status = "COMPLETED"

    async def _loop(self):
        while True:
            self.tick()
            await asyncio.sleep(0.1)
