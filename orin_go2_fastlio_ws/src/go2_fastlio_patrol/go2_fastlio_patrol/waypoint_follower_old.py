#!/usr/bin/env python3
import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('route_file', '/home/unitree/go2_fastlio_ws/src/go2_fastlio_patrol/routes/route_demo.csv')

        self.declare_parameter('v_base', 0.15)
        self.declare_parameter('max_vx', 0.20)
        self.declare_parameter('k_yaw', 1.0)
        self.declare_parameter('max_yaw_rate', 0.30)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('reach_distance', 0.2)
        self.declare_parameter('loop_mode', 'pingpong')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.route_file = self.get_parameter('route_file').value

        self.v_base = float(self.get_parameter('v_base').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.reach_distance = float(self.get_parameter('reach_distance').value)
        self.loop_mode = self.get_parameter('loop_mode').value

        self.route = self.load_route(self.route_file)
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.target_index = 0
        self.direction = 1
        self.initialized = False

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('waypoint_follower started')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'route points: {len(self.route)}')
        self.get_logger().info(f'cmd_topic: {self.cmd_topic}')

    def load_route(self, path):
        route = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                route.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'v': float(row['v'])
                })

        if len(route) < 2:
            raise RuntimeError('route file must contain at least 2 points')

        return route

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if not self.initialized:
            self.target_index = self.find_nearest_index()
            self.initialized = True
            self.get_logger().info(f'nearest route index: {self.target_index}')

    def find_nearest_index(self):
        best_i = 0
        best_d = 1e9

        for i, p in enumerate(self.route):
            d = math.hypot(p['x'] - self.current_x, p['y'] - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i

        return best_i

    def publish_stop(self):
        cmd = Twist()
        self.pub.publish(cmd)

    def control_loop(self):
        if self.current_x is None:
            self.publish_stop()
            return

        # 如果当前目标点已经接近，则切换到下一个点
        tx = self.route[self.target_index]['x']
        ty = self.route[self.target_index]['y']
        dist_to_target = math.hypot(tx - self.current_x, ty - self.current_y)

        if dist_to_target < self.reach_distance:
            self.target_index += self.direction

            if self.target_index >= len(self.route):
                if self.loop_mode == 'pingpong':
                    self.direction = -1
                    self.target_index = len(self.route) - 2
                    self.get_logger().info('reach end, switch to backward')
                else:
                    self.publish_stop()
                    return

            elif self.target_index < 0:
                if self.loop_mode == 'pingpong':
                    self.direction = 1
                    self.target_index = 1
                    self.get_logger().info('reach start, switch to forward')
                else:
                    self.publish_stop()
                    return

        # lookahead：尽量选前方更远一点的点，减少抖动
        look_i = self.target_index
        while True:
            next_i = look_i + self.direction
            if next_i < 0 or next_i >= len(self.route):
                break

            px = self.route[next_i]['x']
            py = self.route[next_i]['y']
            d = math.hypot(px - self.current_x, py - self.current_y)

            if d >= self.lookahead_distance:
                look_i = next_i
                break

            look_i = next_i

        target = self.route[look_i]
        dx = target['x'] - self.current_x
        dy = target['y'] - self.current_y

        target_angle = math.atan2(dy, dx)
        alpha = normalize_angle(target_angle - self.current_yaw)

        vx = min(self.v_base, self.max_vx)
        yaw_rate = self.k_yaw * alpha

        if yaw_rate > self.max_yaw_rate:
            yaw_rate = self.max_yaw_rate
        elif yaw_rate < -self.max_yaw_rate:
            yaw_rate = -self.max_yaw_rate

        # 角度误差太大时，降低前进速度，防止冲出去
        if abs(alpha) > 1.0:
            vx = min(vx, 0.05)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = 0.0
        cmd.angular.z = yaw_rate

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('waypoint_follower stopped by Ctrl+C')
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
