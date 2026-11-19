#!/usr/bin/env python3
"""D (amr_navigation) Gate 1 MOCK: A<->D 서비스 3개 + Nav2 dispatcher + 가짜 Nav2 서버.

프로세스 (모두 같은 환경변수/네임스페이스에서 실행):
  server : 가짜 navigate_to_pose      (mode: succeed | abort | reject | no_response)
  d      : D 노드  - dispatch_goal / lookup_goal 서버, report_goal_event 클라이언트
  a      : A 대역  - dispatch_goal / lookup_goal 클라이언트, report_goal_event 서버
           시나리오: normal | dup | conflict | abort | uncertain | result_error
  demo   : (PR #12) D 단독 데모        시나리오: normal | dup | cancel | badwp

복구 규칙:
  - 결과 조회 실패(RESULT_ERROR)·상태 불명은 몇 번 조회해도 UNKNOWN 유지
  - NO_ACTIVE_GOAL 은 (1) 해당 UUID 가 종결 상태로 확인되고
                     (2) 다른 실행·지연 송신이 하나도 없을 때만 반환 (no_pending_dispatch=True)
"""
import math
import re
import sys
import time
from dataclasses import dataclass, fields, replace
from enum import Enum, auto
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID

try:
    from mission_manager.node import NavigationGoal, NavigationObservation
    from mission_manager.navigation_boundary import DispatchTicket
    USING_A_TYPES = True
except ImportError:
    USING_A_TYPES = False

    class NavigationObservation(Enum):
        ARRIVED = auto()
        NOT_ARRIVED = auto()
        NO_ACTIVE_GOAL = auto()
        UNKNOWN = auto()
        DISCONNECTED = auto()
        TIMEOUT = auto()

    @dataclass(frozen=True)
    class NavigationGoal:
        mission_id: str
        step_id: str | None
        plan_version: int
        waypoint_id: str
        attempt_id: str | None

    @dataclass(frozen=True)
    class DispatchTicket:
        goal: NavigationGoal
        nav_goal_id: str


UUID_RE = re.compile(r'^[0-9a-f]{32}$')

# MOCK 전용 Waypoint (지도 확정 전 임시값, 단위 m / rad)
MOCK_WAYPOINTS = {
    'HOME': (0.0, 0.0, 0.0),
    'A': (1.0, 0.5, 0.0),
    'B': (2.0, -0.5, 1.57),
    'C': (0.5, 1.5, 3.14),
}

PENDING_STATES = ('SENT', 'ACTIVE', 'UNCERTAIN', 'RESULT_UNKNOWN')
CONFIRMED_NOT_ACTIVE = ('ABORTED', 'CANCELED', 'REJECTED')


def uuid_hex(msg_uuid):
    return bytes(bytearray(int(b) for b in msg_uuid)).hex()


# ===================== 실행 컨텍스트 =====================
@dataclass(frozen=True)
class GoalContext:
    """A가 SQLite에 기록한 실행 컨텍스트. step_id '' = 단독 HOME 복귀 (A의 None)."""
    mission_id: str
    step_id: str
    plan_version: int
    waypoint_id: str
    attempt_id: str
    nav_goal_id: str

    @classmethod
    def from_msg(cls, m):
        return cls(m.mission_id, m.step_id, int(m.plan_version),
                   m.waypoint_id, m.attempt_id, m.nav_goal_id)

    def to_msg(self, m):
        for f in fields(self):
            setattr(m, f.name, getattr(self, f.name))
        return m

    def problems(self, waypoints):
        errs = []
        if not self.mission_id:
            errs.append('mission_id empty')
        if not self.attempt_id:
            errs.append('attempt_id empty')
        if not UUID_RE.match(self.nav_goal_id or ''):
            errs.append('nav_goal_id must be 32 lowercase hex')
        if self.waypoint_id not in waypoints:
            errs.append(f'unknown waypoint: {self.waypoint_id}')
        return errs


@dataclass(frozen=True)
class SimpleTicket:
    goal: object          # .waypoint_id 필요
    nav_goal_id: str


# ===================== D: Nav2 dispatcher =====================
@dataclass
class DispatchRecord:
    ticket: object
    state: str              # SENT | ACTIVE | DONE | UNCERTAIN | RESULT_UNKNOWN
    sent_at: float
    goal_handle: object = None
    terminal: str | None = None   # SUCCEEDED | ABORTED | CANCELED | REJECTED
    detail: str = ''
    error_code: int = 0
    late: bool = False


class Nav2Dispatcher:
    """A가 확정한 UUID로 Nav2에 한 번만 보내고, 확인된 사실만 보고한다.

    on_event(ticket, event, detail, error_code)
      event: ACCEPTED | REJECTED | SUCCEEDED | ABORTED | CANCELED | UNCERTAIN
    """

    def __init__(self, node, waypoints, action_name='navigate_to_pose', frame_id='map',
                 accept_timeout_sec=5.0, on_event=None, fault_result_error=None):
        self.node = node
        self.log = node.get_logger()
        self.waypoints = dict(waypoints)
        self.frame_id = frame_id
        self.accept_timeout = accept_timeout_sec
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.records = {}
        self.on_event = on_event or (lambda ticket, event, detail, err: None)
        self.fault_result_error = fault_result_error or (lambda: False)
        node.create_timer(0.2, self._check_accept_timeouts)

    # ---------- 전송 전 점검 ----------
    def ready_for(self, goal):
        if goal.waypoint_id not in self.waypoints:
            return False, f'unknown waypoint: {goal.waypoint_id}'
        if not self.client.server_is_ready():
            return False, 'navigate_to_pose server not ready'
        if self.has_pending():
            return False, 'another dispatch is active, pending or uncertain'
        return True, 'ok'

    def has_pending(self, exclude=None):
        return any(r.state in PENDING_STATES
                   for k, r in self.records.items() if k != exclude)

    # ---------- sender ----------
    def send(self, ticket):
        gid = ticket.nav_goal_id
        if not isinstance(gid, str) or not UUID_RE.match(gid):
            raise ValueError(f'nav_goal_id must be 32 lowercase hex: {gid!r}')
        if gid in self.records:
            self.log.warn(f'[DUP] {gid} 이미 전송됨 → 무시 (재전송 안 함)')
            return False
        wp = self.waypoints.get(ticket.goal.waypoint_id)
        if wp is None:
            raise KeyError(f'unknown waypoint: {ticket.goal.waypoint_id}')

        x, y, yaw = wp
        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = ps

        # 전송 전에 기록 → 예외가 나도 같은 UUID 재전송 차단
        self.records[gid] = DispatchRecord(ticket, 'SENT', time.monotonic())
        future = self.client.send_goal_async(
            goal_msg, goal_uuid=UUID(uuid=list(bytes.fromhex(gid))))
        future.add_done_callback(lambda f, g=gid: self._on_goal_response(g, f))
        self.log.info(f'[SEND] {gid} → waypoint {ticket.goal.waypoint_id} ({x}, {y}, {yaw})')
        return True

    def cancel(self, gid):
        """안전 정지 등 명시적 취소용. 재계획 이유로는 사용하지 않는다(INTERFACES 7.4)."""
        rec = self.records.get(gid)
        if rec is None or rec.state != 'ACTIVE' or rec.goal_handle is None:
            self.log.warn(f'[CANCEL] {gid} 취소 불가 (state={rec.state if rec else None})')
            return False
        rec.goal_handle.cancel_goal_async()
        self.log.info(f'[CANCEL] {gid} 취소 요청')
        return True

    # ---------- 콜백 ----------
    def _on_goal_response(self, gid, future):
        rec = self.records[gid]
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._mark_uncertain(gid, f'GOAL_RESPONSE_ERROR: {exc}')
            return
        if rec.state == 'UNCERTAIN':
            rec.late = True
            self.log.warn(f'[LATE] {gid} 타임아웃 이후 수락 응답 도착')
        if not handle.accepted:
            self._terminal(gid, 'REJECTED', 'goal rejected by server')
            return
        returned = uuid_hex(handle.goal_id.uuid)
        if returned != gid:
            self._mark_uncertain(gid, f'UUID_MISMATCH returned={returned}')
            return
        rec.goal_handle = handle
        rec.state = 'ACTIVE'      # 수락이 확인되었으므로 실행 중
        self.log.info(f'[ACCEPTED] {gid} (UUID 왕복 일치{", 늦은 응답" if rec.late else ""})')
        self.on_event(rec.ticket, 'ACCEPTED', 'late' if rec.late else '', 0)
        handle.get_result_async().add_done_callback(lambda f, g=gid: self._on_result(g, f))

    def _on_result(self, gid, future):
        rec = self.records[gid]
        try:
            if self.fault_result_error():
                raise RuntimeError('injected result lookup failure')
            res = future.result()
            status = res.status
            err = int(getattr(res.result, 'error_code', 0))
        except Exception as exc:  # noqa: BLE001
            # 결과를 못 받았다 = 목표가 아직 실행 중일 수 있다 → 끝까지 UNKNOWN
            rec.state, rec.detail = 'RESULT_UNKNOWN', f'RESULT_ERROR: {exc}'
            self.log.error(f'[RESULT_UNKNOWN] {gid} ({rec.detail}) → 복구 조회도 UNKNOWN 유지')
            self.on_event(rec.ticket, 'UNCERTAIN', rec.detail, 0)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._terminal(gid, 'SUCCEEDED', '', err)
        elif status == GoalStatus.STATUS_ABORTED:
            self._terminal(gid, 'ABORTED', f'error_code={err}', err)
        elif status == GoalStatus.STATUS_CANCELED:
            self._terminal(gid, 'CANCELED', '', err)
        else:
            rec.state, rec.detail = 'RESULT_UNKNOWN', f'unexpected status {status}'
            self.log.error(f'[RESULT_UNKNOWN] {gid} ({rec.detail})')
            self.on_event(rec.ticket, 'UNCERTAIN', rec.detail, err)

    def _terminal(self, gid, terminal, detail, err=0):
        rec = self.records[gid]
        rec.state, rec.terminal, rec.detail, rec.error_code = 'DONE', terminal, detail, err
        tag = '[LATE-RESULT]' if rec.late else '[RESULT]'
        self.log.info(f'{tag} {gid} → {terminal} {detail}'.rstrip())
        self.on_event(rec.ticket, terminal, detail, err)

    def _mark_uncertain(self, gid, reason):
        rec = self.records[gid]
        rec.state, rec.detail = 'UNCERTAIN', reason
        self.log.error(f'[UNCERTAIN] {gid} ({reason}) → 미실행으로 간주하지 않음')
        self.on_event(rec.ticket, 'UNCERTAIN', reason, 0)

    def _check_accept_timeouts(self):
        now = time.monotonic()
        for gid, rec in list(self.records.items()):
            if rec.state == 'SENT' and now - rec.sent_at > self.accept_timeout:
                self._mark_uncertain(gid, 'ACCEPT_TIMEOUT')

    # ---------- 복구 조회 ----------
    def inspect(self, gid):
        """returns (NavigationObservation, no_pending_dispatch, detail)."""
        rec = self.records.get(gid) if gid else None
        others = self.has_pending(exclude=gid)
        if rec is None:
            return NavigationObservation.UNKNOWN, False, 'no record in this D process'
        if rec.state == 'ACTIVE':
            return NavigationObservation.NOT_ARRIVED, False, 'goal still executing'
        if rec.state == 'DONE' and rec.terminal == 'SUCCEEDED':
            return NavigationObservation.ARRIVED, not others, 'succeeded'
        if rec.state == 'DONE' and rec.terminal in CONFIRMED_NOT_ACTIVE:
            if others:
                return (NavigationObservation.UNKNOWN, False,
                        f'{rec.terminal} but another dispatch is pending')
            return NavigationObservation.NO_ACTIVE_GOAL, True, f'confirmed {rec.terminal}'
        # SENT / UNCERTAIN / RESULT_UNKNOWN: 실행 여부를 확인할 수 없음
        return NavigationObservation.UNKNOWN, False, rec.detail or rec.state

    def state_name(self, gid):
        rec = self.records.get(gid)
        if rec is None:
            return 'NONE'
        if rec.state == 'DONE':
            return rec.terminal
        if rec.state == 'RESULT_UNKNOWN':
            return 'UNCERTAIN'
        return rec.state


# ===================== D 노드 (서비스 3개) =====================
def consts(cls, prefix):
    """srv 상수 → 이름 표 (예: RESULT_ACCEPTED=1 → {1: 'ACCEPTED'})."""
    return {getattr(cls, k): k[len(prefix):] for k in dir(cls)
            if k.startswith(prefix) and isinstance(getattr(cls, k), int)}


def load_srvs():
    from logistics_msgs.srv import DispatchGoal, LookupGoal, ReportGoalEvent
    return DispatchGoal, ReportGoalEvent, LookupGoal


class DNavigationNode(Node):
    def __init__(self):
        super().__init__('d_navigation_mock')
        self.DispatchGoal, self.ReportGoalEvent, self.LookupGoal = load_srvs()
        self.declare_parameter('accept_timeout_sec', 5.0)
        self.declare_parameter('fault_result_error', False)
        self.disp = Nav2Dispatcher(
            self, MOCK_WAYPOINTS,
            accept_timeout_sec=float(self.get_parameter('accept_timeout_sec').value),
            on_event=self._on_event,
            fault_result_error=lambda: bool(self.get_parameter('fault_result_error').value))
        self.contexts = {}        # nav_goal_id -> GoalContext
        self.attempts = {}        # attempt_id -> nav_goal_id
        self.sent_count = 0
        self.event_seq = 0
        self.outbox = []          # 아직 A가 응답하지 않은 ReportGoalEvent 요청
        self.inflight = None
        self.inflight_at = 0.0
        self.create_service(self.DispatchGoal, 'dispatch_goal', self.on_dispatch)
        self.create_service(self.LookupGoal, 'lookup_goal', self.on_lookup)
        self.report_cli = self.create_client(self.ReportGoalEvent, 'report_goal_event')
        self.create_timer(0.5, self._flush_outbox)
        self.get_logger().info('D 노드 시작: dispatch_goal / lookup_goal 서버, report_goal_event 클라이언트')

    STATE_CODES = {'NONE': 0, 'SENT': 1, 'ACTIVE': 2, 'SUCCEEDED': 3, 'ABORTED': 4,
                   'CANCELED': 5, 'REJECTED': 6, 'UNCERTAIN': 7}
    EVENT_CODES = {'ACCEPTED': 1, 'REJECTED': 2, 'SUCCEEDED': 3, 'ABORTED': 4,
                   'CANCELED': 5, 'UNCERTAIN': 6}
    OBS_CODES = {'ARRIVED': 1, 'NOT_ARRIVED': 2, 'NO_ACTIVE_GOAL': 3, 'UNKNOWN': 4,
                 'DISCONNECTED': 5, 'TIMEOUT': 6}

    # ---------- dispatch_goal ----------
    def on_dispatch(self, req, res):
        D = self.DispatchGoal.Response
        ctx = GoalContext.from_msg(req)
        gid = ctx.nav_goal_id
        res.attempt_id, res.nav_goal_id = ctx.attempt_id, gid
        errs = ctx.problems(MOCK_WAYPOINTS)
        if errs:
            res.result, res.reason = D.RESULT_INVALID, '; '.join(errs)
        elif gid in self.contexts:
            if self.contexts[gid] == ctx:
                res.result, res.reason = D.RESULT_DUPLICATE, 'same request: no new send'
            else:
                res.result, res.reason = D.RESULT_CONFLICT, 'same nav_goal_id with different context'
        elif ctx.attempt_id in self.attempts:
            res.result, res.reason = D.RESULT_CONFLICT, 'same attempt_id with different nav_goal_id'
        else:
            ok, why = self.disp.ready_for(ctx)
            if not ok:
                res.result, res.reason = D.RESULT_NOT_READY, why
            else:
                self.contexts[gid] = ctx
                self.attempts[ctx.attempt_id] = gid
                res.result, res.reason = D.RESULT_ACCEPTED, 'received; Nav2 send started'
                try:
                    if self.disp.send(SimpleTicket(ctx, gid)):
                        self.sent_count += 1
                except Exception as exc:  # noqa: BLE001
                    self.disp.records.setdefault(
                        gid, DispatchRecord(SimpleTicket(ctx, gid), 'UNCERTAIN', time.monotonic()))
                    self.disp._mark_uncertain(gid, f'SEND_ERROR: {exc}')
        res.goal_state = self.STATE_CODES[self.disp.state_name(gid)]
        name = consts(D, 'RESULT_').get(res.result, res.result)
        self.get_logger().info(
            f'[dispatch_goal] {gid} attempt={ctx.attempt_id} → {name} ({res.reason}) '
            f'state={self.disp.state_name(gid)} / 누적 Nav2 송신 {self.sent_count}회')
        return res

    # ---------- lookup_goal ----------
    def on_lookup(self, req, res):
        ctx = GoalContext.from_msg(req)
        ctx.to_msg(res)
        res.check_id = req.check_id
        stored = self.contexts.get(ctx.nav_goal_id)
        if stored is not None and stored != ctx:
            obs, nop, detail = NavigationObservation.UNKNOWN, False, 'context mismatch'
        else:
            obs, nop, detail = self.disp.inspect(ctx.nav_goal_id)
        res.observation = self.OBS_CODES[obs.name]
        res.no_pending_dispatch = bool(nop)
        res.detail = detail
        self.get_logger().info(
            f'[lookup_goal] check={req.check_id[:8]} {ctx.nav_goal_id} → {obs.name}, '
            f'no_pending_dispatch={nop} ({detail})')
        return res

    # ---------- report_goal_event ----------
    def _on_event(self, ticket, event, detail, err):
        ctx = ticket.goal
        if not isinstance(ctx, GoalContext):
            return
        self.event_seq += 1
        req = self.ReportGoalEvent.Request()
        ctx.to_msg(req)
        req.event = self.EVENT_CODES[event]
        req.event_seq = self.event_seq
        req.nav2_error_code = int(err)
        req.detail = detail
        self.outbox.append((event, req))
        self._flush_outbox()

    def _flush_outbox(self):
        if self.inflight is not None:
            if time.monotonic() - self.inflight_at < 3.0:
                return
            self.get_logger().warn('[report_goal_event] 응답 없음 → 같은 event_seq로 재전송')
            self.inflight = None
        if not self.outbox or not self.report_cli.service_is_ready():
            return
        event, req = self.outbox[0]
        fut = self.report_cli.call_async(req)
        self.inflight, self.inflight_at = fut, time.monotonic()
        fut.add_done_callback(lambda f, e=event, r=req: self._on_report_done(f, e, r))

    def _on_report_done(self, fut, event, req):
        if fut is not self.inflight:
            return
        self.inflight = None
        try:
            res = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[report_goal_event] 실패 {exc} → 재시도')
            return
        if self.outbox and self.outbox[0][1] is req:
            self.outbox.pop(0)
        self.get_logger().info(
            f'[report_goal_event] seq={req.event_seq} {event} → A handled={res.handled} ({res.reason})')
        self._flush_outbox()


# ===================== A 대역 (테스트용) =====================
class AMockNode(Node):
    def __init__(self):
        super().__init__('a_mission_mock')
        self.DispatchGoal, self.ReportGoalEvent, self.LookupGoal = load_srvs()
        self.dispatch_cli = self.create_client(self.DispatchGoal, 'dispatch_goal')
        self.lookup_cli = self.create_client(self.LookupGoal, 'lookup_goal')
        self.create_service(self.ReportGoalEvent, 'report_goal_event', self.on_event)
        self.current = None
        self.events = []
        self.seen = set()
        self.ev_names = consts(self.ReportGoalEvent.Request, 'EVENT_')
        self.obs_names = consts(self.LookupGoal.Response, 'OBS_')
        self.res_names = consts(self.DispatchGoal.Response, 'RESULT_')

    def on_event(self, req, res):
        ctx = GoalContext.from_msg(req)
        name = self.ev_names.get(req.event, str(req.event))
        key = (ctx.nav_goal_id, req.event_seq)
        if ctx != self.current:
            res.handled, res.reason = False, 'stale or mismatched context'
        elif key in self.seen:
            res.handled, res.reason = False, 'duplicate event'
        else:
            res.handled, res.reason = True, 'ok'
            self.events.append(name)
        self.seen.add(key)
        self.get_logger().info(
            f'[event] {name} seq={req.event_seq} {ctx.nav_goal_id[:8]} '
            f'detail="{req.detail}" → handled={res.handled}')
        return res

    def call(self, cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{cli.srv_name} 서비스 없음 (D 노드 실행/환경변수 확인)')
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def dispatch(self, ctx, set_current=True):
        if set_current:
            self.current = ctx
        res = self.call(self.dispatch_cli, ctx.to_msg(self.DispatchGoal.Request()))
        name = self.res_names.get(res.result) if res else 'NO_RESPONSE'
        self.get_logger().info(f'[dispatch] {ctx.nav_goal_id[:8]} wp={ctx.waypoint_id} → {name}'
                               f'{" (" + res.reason + ")" if res else ""}')
        return name

    def lookup(self, ctx):
        req = ctx.to_msg(self.LookupGoal.Request())
        req.check_id = uuid4().hex
        res = self.call(self.lookup_cli, req)
        if res is None:
            self.get_logger().info('[lookup] NO_RESPONSE')
            return 'NO_RESPONSE', False
        echo_ok = (res.check_id == req.check_id and GoalContext.from_msg(res) == ctx)
        name = self.obs_names.get(res.observation)
        self.get_logger().info(f'[lookup] {name}, no_pending_dispatch={res.no_pending_dispatch} '
                               f'({res.detail}) / 컨텍스트·check_id 반환 일치={echo_ok}')
        return name, res.no_pending_dispatch

    def wait_event(self, names, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(n in self.events for n in names):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def run_a(scenario):
    node = AMockNode()
    log = node.get_logger()
    ctx = GoalContext('mock-mission', 'step-1', 1, 'A', f'attempt-{uuid4().hex[:6]}', uuid4().hex)
    checks = []

    def check(label, ok):
        checks.append((label, ok))

    first = node.dispatch(ctx)
    check('dispatch → ACCEPTED', first == 'ACCEPTED')

    if scenario == 'dup':
        check('같은 요청 재전송 → DUPLICATE', node.dispatch(ctx) == 'DUPLICATE')
    if scenario == 'conflict':
        other = replace(ctx, waypoint_id='B')
        check('같은 UUID·다른 컨텍스트 → CONFLICT',
              node.dispatch(other, set_current=False) == 'CONFLICT')

    if scenario in ('normal', 'dup', 'conflict'):
        check('SUCCEEDED 이벤트 수신', node.wait_event(['SUCCEEDED'], 15.0))
        obs, nop = node.lookup(ctx)
        check('lookup → ARRIVED', obs == 'ARRIVED')
    elif scenario == 'abort':
        check('ABORTED 이벤트 수신', node.wait_event(['ABORTED'], 15.0))
        obs, nop = node.lookup(ctx)
        check('lookup → NO_ACTIVE_GOAL + no_pending_dispatch=True', obs == 'NO_ACTIVE_GOAL' and nop)
    elif scenario in ('uncertain', 'result_error'):
        check('UNCERTAIN 이벤트 수신', node.wait_event(['UNCERTAIN'], 15.0))
        for i in (1, 2):
            obs, nop = node.lookup(ctx)
            check(f'lookup {i}회차 → UNKNOWN (no_pending=False)', obs == 'UNKNOWN' and not nop)
            node.wait_event(['__none__'], 1.0)

    log.info('==================== 결과 ====================')
    log.info(f'시나리오: {scenario} / nav_goal_id: {ctx.nav_goal_id}')
    for label, ok in checks:
        log.info(f'  {"PASS" if ok else "FAIL"}  {label}')
    log.info(f'A가 처리한 이벤트: {node.events}')
    log.info('Nav2 송신 횟수는 D 창의 "누적 Nav2 송신 N회" 로그로 확인')
    log.info('=============================================')
    node.destroy_node()


# ===================== 가짜 Nav2 서버 =====================
class MockNav2Server(Node):
    def __init__(self):
        super().__init__('mock_nav2_server')
        self.declare_parameter('mode', 'succeed')
        self.declare_parameter('delay_sec', 3.0)
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=ReentrantCallbackGroup())
        self.get_logger().info('MOCK navigate_to_pose 서버 시작 (mode: succeed|abort|reject|no_response)')

    def mode(self):
        return str(self.get_parameter('mode').value)

    def on_goal(self, request):
        p = request.pose.pose.position
        m = self.mode()
        self.get_logger().info(f'[GOAL] ({p.x:.2f}, {p.y:.2f}) frame={request.pose.header.frame_id} mode={m}')
        if m == 'reject':
            self.get_logger().warn('[MOCK] 거절')
            return GoalResponse.REJECT
        if m == 'no_response':
            self.get_logger().warn('[MOCK] 수락 응답 30초 지연')
            time.sleep(30.0)
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info('[MOCK] 취소 요청 수락')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        gid = uuid_hex(goal_handle.goal_id.uuid)
        m = self.mode()
        delay = float(self.get_parameter('delay_sec').value)
        self.get_logger().info(f'[EXEC] 받은 UUID={gid}, {delay:.1f}s 주행 흉내')
        fb = NavigateToPose.Feedback()
        end = time.monotonic() + delay
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f'[EXEC] {gid} CANCELED')
                return NavigateToPose.Result()
            fb.distance_remaining = float(end - time.monotonic())
            goal_handle.publish_feedback(fb)
            time.sleep(0.2)
        if m == 'abort':
            goal_handle.abort()
            self.get_logger().info(f'[EXEC] {gid} ABORTED')
        else:
            goal_handle.succeed()
            self.get_logger().info(f'[EXEC] {gid} SUCCEEDED')
        return NavigateToPose.Result()


# ===================== (PR #12) D 단독 데모 =====================
def run_demo(scenario):
    node = Node('d_gate1_demo')
    log = node.get_logger()
    events = []
    disp = Nav2Dispatcher(node, MOCK_WAYPOINTS,
                          on_event=lambda t, e, d, err: events.append(e))
    log.info(f'A 타입 사용: {USING_A_TYPES} / 시나리오: {scenario}')
    t0 = time.monotonic()
    while not disp.client.server_is_ready() and time.monotonic() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    wp = 'ZZZ' if scenario == 'badwp' else 'A'
    goal = NavigationGoal(mission_id='mock-mission', step_id='step-1', plan_version=1,
                          waypoint_id=wp, attempt_id='attempt-1')
    ticket = DispatchTicket(goal, uuid4().hex)
    ok, why = disp.ready_for(goal)
    sent = 0
    if not ok:
        log.warn(f'[HOLD] 전송 보류: {why}')
    else:
        sent += 1 if disp.send(ticket) else 0
        if scenario == 'dup':
            sent += 1 if disp.send(ticket) else 0
        cancel_sent = False
        deadline = time.monotonic() + 40.0
        final = ('SUCCEEDED', 'ABORTED', 'CANCELED', 'REJECTED')
        while time.monotonic() < deadline and not any(e in final for e in events):
            rclpy.spin_once(node, timeout_sec=0.1)
            rec = disp.records[ticket.nav_goal_id]
            if (scenario == 'cancel' and not cancel_sent and rec.state == 'ACTIVE'
                    and time.monotonic() - rec.sent_at > 1.0):
                cancel_sent = disp.cancel(ticket.nav_goal_id)
    obs, nop, detail = disp.inspect(ticket.nav_goal_id)
    log.info('==================== 요약 ====================')
    log.info(f'nav_goal_id      : {ticket.nav_goal_id}')
    log.info(f'실제 전송 횟수   : {sent}  (1 이하여야 PASS) → {"PASS" if sent <= 1 else "FAIL"}')
    log.info(f'이벤트           : {events}')
    log.info(f'복구 조회 inspect: {obs.name}, no_pending_dispatch={nop} ({detail})')
    log.info('=============================================')
    node.destroy_node()


def main():
    args = sys.argv[1:]
    cmd = args[0] if args else 'demo'
    scenario = args[1] if len(args) > 1 else 'normal'
    rclpy.init(args=sys.argv)
    try:
        if cmd == 'server':
            node = MockNav2Server()
            ex = MultiThreadedExecutor()
            ex.add_node(node)
            try:
                ex.spin()
            except KeyboardInterrupt:
                pass
        elif cmd == 'd':
            node = DNavigationNode()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
        elif cmd == 'a':
            run_a(scenario)
        else:
            run_demo(scenario)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
