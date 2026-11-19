#!/usr/bin/env python3
"""Gate 1 MOCK (D, amr_navigation): Nav2 dispatcher + fake NavigateToPose server + demo.

- Nav2Dispatcher.send(ticket): A의 submit_once(ticket, sender)에 넘길 sender.
  * A가 확정한 32자리 hex UUID를 그대로 Nav2 goal_uuid로 사용
  * 같은 UUID는 두 번 보내지 않음
  * 최종 result가 SUCCEEDED일 때만 ARRIVED
  * 수락 응답 타임아웃/예외는 UNCERTAIN (미실행으로 간주하지 않음)
- MockNav2Server: 지도 없이 동작하는 가짜 navigate_to_pose 서버
  mode 파라미터: succeed | abort | reject | no_response

사용법 (터미널 2개, 둘 다 같은 환경변수):
  T1: python3 ros2_ws/src/amr_navigation/amr_navigation/gate1_mock.py server
  T2: python3 ros2_ws/src/amr_navigation/amr_navigation/gate1_mock.py demo [normal|dup|cancel|badwp]
  모드 변경: ros2 param set /mock_nav2_server mode abort
"""
import math
import re
import sys
import time
from dataclasses import dataclass
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

# A의 정의를 그대로 사용 (워크스페이스가 source 되어 있으면). 없으면 동일 구조의 대체 정의.
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


def uuid_hex(msg_uuid):
    return bytes(bytearray(int(b) for b in msg_uuid)).hex()


@dataclass
class DispatchRecord:
    ticket: object
    state: str              # SENT | ACTIVE | DONE | UNCERTAIN
    sent_at: float
    goal_handle: object = None
    observation: object = None
    reason: str = ''
    late: bool = False


class Nav2Dispatcher:
    """D adapter: A가 확정한 티켓을 Nav2에 한 번만 보내고 결과를 판정한다."""

    def __init__(self, node, waypoints, action_name='navigate_to_pose', frame_id='map',
                 accept_timeout_sec=5.0, on_observation=None, on_uncertain=None):
        self.node = node
        self.log = node.get_logger()
        self.waypoints = dict(waypoints)
        self.frame_id = frame_id
        self.accept_timeout = accept_timeout_sec
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.records = {}
        self.on_observation = on_observation or (lambda ticket, obs, reason: None)
        self.on_uncertain = on_uncertain or (lambda ticket, reason: None)
        node.create_timer(0.2, self._check_accept_timeouts)

    # ---------- 전송 전 점검 (A의 submit_once 호출 전에 사용) ----------
    def ready_for(self, goal):
        if goal.waypoint_id not in self.waypoints:
            return False, f'unknown waypoint: {goal.waypoint_id}'
        if not self.client.server_is_ready():
            return False, 'navigate_to_pose server not ready'
        if self.has_pending():
            return False, 'another dispatch is pending or uncertain'
        return True, 'ok'

    def has_pending(self):
        return any(r.state in ('SENT', 'ACTIVE', 'UNCERTAIN') for r in self.records.values())

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
        """안전 정지 등 명시적 취소용. 재계획 이유로는 사용하지 않는다(7.4)."""
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
            self.log.warn(f'[LATE] {gid} 타임아웃 이후 수락 응답 도착 (계속 추적만 함)')
        if not handle.accepted:
            self._finish(gid, NavigationObservation.NOT_ARRIVED, 'REJECTED')
            return
        returned = uuid_hex(handle.goal_id.uuid)
        if returned != gid:
            self._mark_uncertain(gid, f'UUID_MISMATCH returned={returned}')
            return
        self.log.info(f'[ACCEPTED] {gid} (UUID 왕복 일치)')
        rec.goal_handle = handle
        if not rec.late:
            rec.state = 'ACTIVE'
        handle.get_result_async().add_done_callback(lambda f, g=gid: self._on_result(g, f))

    def _on_result(self, gid, future):
        try:
            res = future.result()
            status = res.status
            err = getattr(res.result, 'error_code', 0)
        except Exception as exc:  # noqa: BLE001
            self._finish(gid, NavigationObservation.UNKNOWN, f'RESULT_ERROR: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            obs, reason = NavigationObservation.ARRIVED, 'SUCCEEDED'
        elif status == GoalStatus.STATUS_ABORTED:
            obs, reason = NavigationObservation.NOT_ARRIVED, f'ABORTED(error_code={err})'
        elif status == GoalStatus.STATUS_CANCELED:
            obs, reason = NavigationObservation.NOT_ARRIVED, 'CANCELED'
        else:
            obs, reason = NavigationObservation.UNKNOWN, f'STATUS_{status}'
        self._finish(gid, obs, reason)

    def _finish(self, gid, obs, reason):
        rec = self.records[gid]
        was_late = rec.late
        rec.state, rec.observation, rec.reason = 'DONE', obs, reason
        tag = '[LATE-RESULT]' if was_late else '[RESULT]'
        self.log.info(f'{tag} {gid} → {obs.name} ({reason})')
        self.on_observation(rec.ticket, obs, reason)

    def _mark_uncertain(self, gid, reason):
        rec = self.records[gid]
        rec.state, rec.reason = 'UNCERTAIN', reason
        self.log.error(f'[UNCERTAIN] {gid} ({reason}) → 미실행으로 간주하지 않음')
        self.on_uncertain(rec.ticket, reason)

    def _check_accept_timeouts(self):
        now = time.monotonic()
        for gid, rec in list(self.records.items()):
            if rec.state == 'SENT' and now - rec.sent_at > self.accept_timeout:
                self._mark_uncertain(gid, 'ACCEPT_TIMEOUT')

    # ---------- 복구 조회 (RecoveryReport 재료) ----------
    def inspect(self, gid):
        """returns (NavigationObservation, no_pending_dispatch). 기록 없음은 아무것도 증명하지 않는다."""
        others = any(r.state in ('SENT', 'UNCERTAIN')
                     for k, r in self.records.items() if k != gid)
        rec = self.records.get(gid) if gid else None
        if rec is None:
            return NavigationObservation.UNKNOWN, False
        if rec.state == 'DONE':
            if rec.observation is NavigationObservation.ARRIVED:
                return NavigationObservation.ARRIVED, not others
            return NavigationObservation.NO_ACTIVE_GOAL, not others
        if rec.state == 'ACTIVE':
            return NavigationObservation.NOT_ARRIVED, not others
        return NavigationObservation.UNKNOWN, False


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


# ===================== 데모 =====================
def run_demo(scenario):
    node = Node('d_gate1_demo')
    log = node.get_logger()
    done = {'obs': None, 'uncertain': None}

    def on_obs(ticket, obs, reason):
        done['obs'] = (obs, reason)

    def on_unc(ticket, reason):
        done['uncertain'] = reason

    disp = Nav2Dispatcher(node, MOCK_WAYPOINTS, on_observation=on_obs, on_uncertain=on_unc)
    log.info(f'A 타입 사용: {USING_A_TYPES} / 시나리오: {scenario}')

    t0 = time.monotonic()
    while not disp.client.server_is_ready() and time.monotonic() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)

    wp = 'ZZZ' if scenario == 'badwp' else 'A'
    goal = NavigationGoal(mission_id='mock-mission', step_id='step-1', plan_version=1,
                          waypoint_id=wp, attempt_id='attempt-1')
    ticket = DispatchTicket(goal, uuid4().hex)

    ok, why = disp.ready_for(goal)
    if not ok:
        log.warn(f'[HOLD] 전송 보류: {why}')
        print_summary(log, disp, ticket, sent_count=0)
        node.destroy_node()
        return

    sent = 1 if disp.send(ticket) else 0
    if scenario == 'dup':
        sent += 1 if disp.send(ticket) else 0

    cancel_sent = False
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        rec = disp.records[ticket.nav_goal_id]
        if scenario == 'cancel' and not cancel_sent and rec.state == 'ACTIVE':
            if time.monotonic() - rec.sent_at > 1.0:
                cancel_sent = disp.cancel(ticket.nav_goal_id)
        if done['obs'] is not None:
            break
    print_summary(log, disp, ticket, sent_count=sent)
    node.destroy_node()


def print_summary(log, disp, ticket, sent_count):
    rec = disp.records.get(ticket.nav_goal_id)
    obs, no_pending = disp.inspect(ticket.nav_goal_id)
    log.info('==================== 요약 ====================')
    log.info(f'nav_goal_id      : {ticket.nav_goal_id}')
    log.info(f'실제 전송 횟수   : {sent_count}  (1 이하여야 PASS)')
    log.info(f'최종 상태        : {rec.state if rec else "NOT_SENT"} / {rec.reason if rec else "-"}')
    log.info(f'관찰 결과        : {rec.observation.name if rec and rec.observation else "-"}')
    log.info(f'복구 조회 inspect: {obs.name}, no_pending_dispatch={no_pending}')
    log.info(f'단일 전송        : {"PASS" if sent_count <= 1 else "FAIL"}')
    log.info('=============================================')


def main():
    args = sys.argv[1:]
    cmd = args[0] if args else 'demo'
    rclpy.init()
    try:
        if cmd == 'server':
            node = MockNav2Server()
            ex = MultiThreadedExecutor()
            ex.add_node(node)
            try:
                ex.spin()
            except KeyboardInterrupt:
                pass
        else:
            run_demo(args[1] if len(args) > 1 else 'normal')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
