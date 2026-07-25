#!/usr/bin/env python3
"""Exercise line rejoin and corner handling on isolated ROS topics."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DryRunNode:
    """Publish controlled poses and capture the resulting patrol commands."""

    def __init__(self):
        self.node = rclpy.create_node('fake_odom_line_corner_dryrun')
        self.publisher = self.node.create_publisher(
            Odometry,
            '/dryrun_odom',
            10,
        )
        self.node.create_subscription(
            Twist,
            '/dryrun_patrol_cmd',
            self.command_callback,
            10,
        )
        self.phase = 'startup'
        self.commands = {}
        self.all_commands = []

    def command_callback(self, message):
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self.commands.setdefault(self.phase, []).append(values)
        self.all_commands.append(values)

    def publish_pose(self, x, y, yaw, duration=0.05):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = Odometry()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = 'dryrun_map'
            message.child_frame_id = 'dryrun_body'
            message.pose.pose.position.x = float(x)
            message.pose.pose.position.y = float(y)
            message.pose.pose.orientation.z = math.sin(yaw / 2.0)
            message.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.02)

    def ramp(self, start, end, steps):
        for index in range(1, steps + 1):
            ratio = index / float(steps)
            pose = tuple(
                first + ratio * (last - first)
                for first, last in zip(start, end)
            )
            self.publish_pose(*pose)

    def run(self):
        time.sleep(0.5)
        self.publish_pose(0.0, 0.0, 0.0, 0.8)

        self.phase = 'straight_rejoin'
        self.ramp((0.0, 0.0, 0.0), (0.60, 0.06, 0.0), 20)
        self.publish_pose(0.60, 0.06, 0.0, 1.3)

        self.phase = 'corner_approach'
        self.ramp((0.60, 0.06, 0.0), (2.20, 0.06, 0.0), 40)
        self.publish_pose(2.20, 0.06, 0.0, 1.3)

        self.phase = 'turn'
        self.ramp((2.20, 0.06, 0.0), (2.95, 0.08, 0.0), 20)
        self.publish_pose(2.95, 0.08, 0.0, 1.3)

        self.phase = 'turn_alignment'
        for degrees in range(10, 91, 10):
            self.publish_pose(
                2.95,
                0.08,
                math.radians(degrees),
                0.12,
            )
        self.publish_pose(2.95, 0.08, math.pi / 2.0, 0.6)

        self.phase = 'outgoing_rejoin'
        self.publish_pose(3.08, 0.10, math.pi / 2.0, 1.3)
        self.publish_pose(3.08, 0.16, math.pi / 2.0, 0.5)

    def phase_yaw_extreme(self, phase, find_min):
        values = [
            command[2]
            for command in self.commands.get(phase, [])
            if command[0] > 0.02 or phase == 'turn'
        ]
        if not values:
            raise RuntimeError(f'no usable commands captured for {phase}')
        return min(values) if find_min else max(values)

    def result(self):
        if not self.all_commands:
            raise RuntimeError('no patrol commands captured')
        max_abs_vy = max(abs(command[1]) for command in self.all_commands)
        result = {
            'command_count': len(self.all_commands),
            'max_abs_vy': max_abs_vy,
            'straight_rejoin_min_yaw': self.phase_yaw_extreme(
                'straight_rejoin',
                True,
            ),
            'corner_approach_min_yaw': self.phase_yaw_extreme(
                'corner_approach',
                True,
            ),
            'turn_max_yaw': self.phase_yaw_extreme('turn', False),
            'outgoing_rejoin_max_yaw': self.phase_yaw_extreme(
                'outgoing_rejoin',
                False,
            ),
        }
        checks = {
            'vy_is_zero': result['max_abs_vy'] <= 1e-9,
            'straight_steers_toward_line': (
                result['straight_rejoin_min_yaw'] < -0.05
            ),
            'corner_aims_at_ordered_point': (
                result['corner_approach_min_yaw'] < -0.05
            ),
            'turn_rotates_to_outgoing_segment': (
                result['turn_max_yaw'] > 0.10
            ),
            'outgoing_segment_rejoin_is_immediate': (
                result['outgoing_rejoin_max_yaw'] > 0.05
            ),
        }
        result['checks'] = checks
        print('DRYRUN_RESULT=' + json.dumps(result, sort_keys=True))
        return all(checks.values())

    def close(self):
        self.node.destroy_node()


def main():
    rclpy.init()
    dry_run = DryRunNode()
    try:
        dry_run.run()
        success = dry_run.result()
    finally:
        dry_run.close()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
