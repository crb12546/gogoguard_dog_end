#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request

from .patrol_control import limit_planar_command


SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class UnitreeCmdNode(Node):
    def __init__(self):
        super().__init__('unitree_cmd_node')

        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('sport_request_topic', '/api/sport/request')

        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('max_yaw_rate', 0.45)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.sport_request_topic = self.get_parameter('sport_request_topic').value

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_yaw_rate = 0.0
        self.last_cmd_time = 0.0
        self.last_log_time = 0.0

        self.pub = self.create_publisher(Request, self.sport_request_topic, 10)

        self.sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('unitree_cmd_node ACTIVE')
        self.get_logger().info(f'{self.cmd_topic} -> {self.sport_request_topic}')
        self.get_logger().info('Move api_id=1008, StopMove api_id=1003')
        self.get_logger().info(f'max_vx={self.max_vx}')
        self.get_logger().info(f'max_vy={self.max_vy}')
        self.get_logger().info(f'max_yaw_rate={self.max_yaw_rate}')
        self.get_logger().info(f'publish_rate={self.publish_rate} Hz')
        self.get_logger().info(f'cmd_timeout={self.cmd_timeout} s')

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cmd_callback(self, msg):
        self.last_vx, self.last_vy, self.last_yaw_rate = limit_planar_command(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            self.max_vx,
            self.max_vy,
            self.max_yaw_rate,
        )
        self.last_cmd_time = time.time()

    def make_request(self, api_id, parameter):
        req = Request()

        # 实测结论：
        # /api/sport/request 连续 Move 使用固定 request id 更稳定。
        if api_id == SPORT_API_ID_MOVE:
            req.header.identity.id = 9100
        else:
            req.header.identity.id = 9101

        req.header.identity.api_id = api_id
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = parameter
        req.binary = []
        return req

    def publish_stop(self):
        req = self.make_request(SPORT_API_ID_STOPMOVE, '{}')
        self.pub.publish(req)

    def publish_move(self, vx, vy, yaw_rate):
        param = {
            'x': float(vx),
            'y': float(vy),
            'z': float(yaw_rate)
        }
        req = self.make_request(SPORT_API_ID_MOVE, json.dumps(param))
        self.pub.publish(req)

    def timer_callback(self):
        now = time.time()

        if now - self.last_cmd_time > self.cmd_timeout:
            self.publish_stop()
            if now - self.last_log_time > 1.0:
                self.last_log_time = now
                self.get_logger().warn('StopMove because /patrol_cmd timeout')
            return

        self.publish_move(self.last_vx, self.last_vy, self.last_yaw_rate)

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            self.get_logger().info(
                f'Move x={self.last_vx:.3f}, y={self.last_vy:.3f}, '
                f'z={self.last_yaw_rate:.3f}'
            )

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeCmdNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C, publish StopMove')
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
