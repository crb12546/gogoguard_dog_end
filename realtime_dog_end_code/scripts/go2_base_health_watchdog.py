#!/usr/bin/env python3
import argparse
import math
import os
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


WS = os.environ.get('GO2_WS', '/home/unitree/go2_fastlio_ws')
DEFAULT_MAX_ABS_XY_M = 5000.0


def shell(cmd, timeout=10):
    try:
        run = subprocess.run(
            ['bash', '-lc', cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        return run.returncode, run.stdout.strip(), run.stderr.strip()
    except Exception as exc:  # noqa: BLE001
        return 255, '', repr(exc)


def process_running(pattern):
    rc, _, _ = shell("pgrep -f %s >/dev/null" % quote_shell(pattern), timeout=3)
    return rc == 0


def quote_shell(value):
    return "'" + str(value).replace("'", "'\\''") + "'"


def patrol_running():
    return process_running('[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver')


def stop_patrol_processes():
    cmd = r'''
pkill -TERM -f '[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver' 2>/dev/null || true
sleep 1
pkill -KILL -f '[w]aypoint_follower|[u]nitree_safe_cmd_node|[c]md_vel_udp_sender|[g]o2_sdk2_udp_receiver' 2>/dev/null || true
sdk_if=${GO2_SDK_IF:-eth1}
/home/unitree/go2_fastlio_ws/build/go2_cmd_vel_bridge/go2_sdk2_motion_probe --iface "$sdk_if" stop >/dev/null 2>&1 || true
'''
    return shell(cmd, timeout=8)


def validate_odom_sample(
    args,
    last_pos,
    last_time,
    now,
    x,
    y,
    z,
):
    """Validate odometry for either planar patrol or explicit 3D monitoring."""
    vals = (x, y, z)
    if not all(math.isfinite(v) for v in vals):
        return 'non_finite_odom:x=%.3f y=%.3f z=%.3f' % vals
    if abs(x) > args.max_abs_xy or abs(y) > args.max_abs_xy:
        return 'odom_xy_abs_out_of_range:x=%.3f y=%.3f z=%.3f' % vals
    if args.monitor_z and abs(z) > args.max_abs_z:
        return 'odom_z_abs_out_of_range:x=%.3f y=%.3f z=%.3f' % vals
    if last_pos is None or last_time is None:
        return ''

    dt = max(1e-3, now - last_time)
    if dt > args.max_jump_dt:
        return ''
    dx = x - last_pos[0]
    dy = y - last_pos[1]
    dz = z - last_pos[2]
    horizontal_step = math.hypot(dx, dy)
    allowed = args.max_jump_distance + args.max_jump_speed * dt
    if horizontal_step > allowed:
        return (
            'odom_horizontal_jump:step=%.3f allowed=%.3f dz=%.3f dt=%.3f '
            'pos=(%.3f,%.3f,%.3f) last=(%.3f,%.3f,%.3f)'
            % (
                horizontal_step,
                allowed,
                dz,
                dt,
                x,
                y,
                z,
                last_pos[0],
                last_pos[1],
                last_pos[2],
            )
        )
    if args.monitor_z and abs(dz) > args.max_jump_z:
        return (
            'odom_z_jump:horizontal_step=%.3f dz=%.3f allowed_z=%.3f '
            'dt=%.3f pos=(%.3f,%.3f,%.3f) last=(%.3f,%.3f,%.3f)'
            % (
                horizontal_step,
                dz,
                args.max_jump_z,
                dt,
                x,
                y,
                z,
                last_pos[0],
                last_pos[1],
                last_pos[2],
            )
        )
    return ''


class BaseHealthWatchdog(Node):
    def __init__(self, args):
        super().__init__('go2_base_health_watchdog')
        self.args = args
        self.last_pos = None
        self.last_time = None
        self.last_msg_time = None
        self.bad_count = 0
        self.last_action_time = 0.0
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            'base health watchdog started: odom=%s mode=%s '
            'max_abs_xy=%.2f max_abs_z=%.2f '
            'max_jump_xy=%.2f max_jump_z=%.2f bad_count=%d cooldown=%.1f '
            'auto_restart=false'
            % (
                args.odom_topic,
                '3d' if args.monitor_z else 'planar_xy',
                args.max_abs_xy,
                args.max_abs_z,
                args.max_jump_distance,
                args.max_jump_z,
                args.bad_count,
                args.cooldown,
            )
        )

    def monitoring_enabled(self):
        return self.args.always or patrol_running()

    def reset_sample(self):
        self.last_pos = None
        self.last_time = None
        self.bad_count = 0

    def on_timer(self):
        if not self.monitoring_enabled():
            self.reset_sample()
            return
        if self.last_msg_time is None:
            return
        elapsed = time.time() - self.last_msg_time
        if elapsed > self.args.no_odom_timeout:
            self.handle_bad('no_odom_for_%.1fs' % elapsed)

    def on_odom(self, msg):
        now = time.time()
        self.last_msg_time = now
        if not self.monitoring_enabled():
            self.reset_sample()
            return

        p = msg.pose.pose.position
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)
        reason = self.validate_sample(now, x, y, z)
        if reason:
            self.handle_bad(reason)
            return

        self.bad_count = 0
        self.last_pos = (x, y, z)
        self.last_time = now

    def validate_sample(self, now, x, y, z):
        return validate_odom_sample(
            self.args,
            self.last_pos,
            self.last_time,
            now,
            x,
            y,
            z,
        )

    def handle_bad(self, reason):
        now = time.time()
        self.bad_count += 1
        self.get_logger().error('base health bad %d/%d: %s' % (self.bad_count, self.args.bad_count, reason))
        if self.bad_count < self.args.bad_count:
            return
        if now - self.last_action_time < self.args.cooldown:
            return
        self.last_action_time = now
        if reason.startswith('no_odom_for_'):
            self.bad_count = 0
            self.get_logger().error(
                'odometry unavailable: keep patrol safety at zero and wait '
                'for the same FAST-LIO instance to recover; base restart disabled'
            )
            return
        self.reset_sample()
        if self.args.dry_run:
            self.get_logger().error(
                'dry-run: would stop patrol; base restart is disabled'
            )
            return
        rc, out, err = stop_patrol_processes()
        self.get_logger().error('stopped patrol processes rc=%s out=%s err=%s' % (rc, out, err))
        self.get_logger().error(
            'base was not restarted; coordinate continuity is preserved'
        )


def parse_args():
    parser = argparse.ArgumentParser(description='Go2 base health watchdog')
    parser.add_argument('--odom-topic', default='/Odometry')
    parser.add_argument('--always', action='store_true', help='monitor even when patrol is not running')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument(
        '--monitor-z',
        action='store_true',
        help='also enforce z absolute/jump limits; planar patrol ignores finite z by default',
    )
    parser.add_argument(
        '--max-abs-xy',
        type=float,
        default=DEFAULT_MAX_ABS_XY_M,
        help='absolute FAST-LIO XY sanity bound in metres (default: 5000)',
    )
    parser.add_argument('--max-abs-z', type=float, default=5.0)
    parser.add_argument('--max-jump-distance', type=float, default=0.80)
    parser.add_argument('--max-jump-speed', type=float, default=0.60)
    parser.add_argument('--max-jump-z', type=float, default=0.50)
    parser.add_argument('--max-jump-dt', type=float, default=2.0)
    parser.add_argument('--bad-count', type=int, default=2)
    parser.add_argument('--cooldown', type=float, default=90.0)
    parser.add_argument('--no-odom-timeout', type=float, default=6.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = BaseHealthWatchdog(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
