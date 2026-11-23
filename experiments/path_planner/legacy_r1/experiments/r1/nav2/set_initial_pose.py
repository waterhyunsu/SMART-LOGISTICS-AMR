"""Publish the robot pose to AMCL (/initialpose) without the project bridge (research 1 Nav2 runs).

    python3 experiments/r1/nav2/set_initial_pose.py X Y YAW_DEG [--localize]
      --localize: refine the pose first with tools/localize.py (scan matching around X Y YAW, robot does not move)
    python3 experiments/r1/nav2/set_initial_pose.py --check-cmd   (exit 0 if something subscribes to cmd_vel_pc)

Used by run_nav2_sim.sh / run_nav2_real.sh after sourcing scripts/env.sh.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def localize(x, y, yaw_deg):
    """tools/localize.py --dry-run around (x, y, yaw). Returns (x, y, yaw_rad, fit) or None."""
    r = subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'localize.py'), str(x), str(y), str(yaw_deg),
                        '--range', '0.3', '--yaw-range', '20', '--dry-run'], capture_output=True, text=True, timeout=120)
    for line in r.stdout.splitlines():
        if line.startswith('RESULT'):
            _, rx, ry, ryaw, fit = line.split()
            return float(rx), float(ry), float(ryaw), float(fit)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float, nargs='?')
    ap.add_argument('y', type=float, nargs='?')
    ap.add_argument('yaw', type=float, nargs='?', default=0.0, help='도')
    ap.add_argument('--localize', action='store_true')
    ap.add_argument('--check-cmd', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('r1_initial_pose')
    if args.check_cmd:
        deadline = time.time() + 5.0
        n = 0
        while time.time() < deadline and not n:
            rclpy.spin_once(node, timeout_sec=0.2)
            n = node.count_subscribers('cmd_vel_pc')
        print(f'cmd_vel_pc 구독자 {n}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if n else 1)
    if args.x is None or args.y is None:
        ap.error('X Y YAW_DEG 를 지정하세요')

    x, y, yaw = args.x, args.y, math.radians(args.yaw)
    if args.localize:
        res = localize(args.x, args.y, args.yaw)
        if res is None:
            print('스캔 맞추기 실패: 주어진 위치를 그대로 씁니다')
        else:
            x, y, yaw, fit = res
            print(f'스캔 맞추기: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°), 맞는 정도 {fit:.2f}'
                  + ('  (경고: 낮음, 위치를 확인하세요)' if fit < 0.8 else ''))
    pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.02 ** 2
    m.pose.covariance[35] = math.radians(3) ** 2
    for _ in range(15):   # wait for AMCL to subscribe, then send a few times
        if pub.get_subscription_count():
            break
        time.sleep(0.2)
    for _ in range(3):
        pub.publish(m)
        time.sleep(0.3)
    print(f'/initialpose 보냄: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
