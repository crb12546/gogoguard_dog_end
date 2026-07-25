#!/usr/bin/env python3
"""Low-overhead, cross-layer telemetry for route recording and patrol.

This recorder intentionally does not subscribe to either point-cloud topic.
Deserializing full clouds in Python can change the timing that an experiment is
trying to measure.  Livox and FAST-LIO queue/latency evidence is collected from
their in-process logs instead.

The JSONL output joins the clocks and state needed to compare:

* FAST-LIO pose and twist;
* Livox IMU samples;
* Go2 sport-state position, velocity and attitude;
* battery, motor temperatures, foot forces and body IMU;
* wireless-controller input used while manually recording a route;
* patrol, safety-output and Unitree API Move commands.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState, WirelessController


def finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def float_list(values):
    return [finite_or_none(value) for value in values]


def integer_list(values):
    return [int(value) for value in values]


def ros_stamp(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        stamp = getattr(message, "stamp", None)
    if stamp is None:
        return None
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None


def quaternion_dict(values):
    if hasattr(values, "x"):
        ordered = [values.x, values.y, values.z, values.w]
    else:
        ordered = list(values)
        if len(ordered) == 4:
            # Unitree IMUState uses [w, x, y, z].
            ordered = [ordered[1], ordered[2], ordered[3], ordered[0]]
    while len(ordered) < 4:
        ordered.append(0.0)
    qx, qy, qz, qw = [float(value) for value in ordered[:4]]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(
        math.pi / 2.0,
        sinp,
    ) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return {
        "qx": finite_or_none(qx),
        "qy": finite_or_none(qy),
        "qz": finite_or_none(qz),
        "qw": finite_or_none(qw),
        "roll": finite_or_none(roll),
        "pitch": finite_or_none(pitch),
        "yaw": finite_or_none(yaw),
    }


def imu_state_dict(message):
    if message is None:
        return None
    return {
        "orientation": quaternion_dict(message.quaternion),
        "gyroscope": float_list(message.gyroscope),
        "accelerometer": float_list(message.accelerometer),
        "rpy": float_list(message.rpy),
        "temperature_c": int(message.temperature),
    }


class JsonlWriter(object):
    def __init__(self, path):
        self.path = path
        self.handle = open(path, "w", buffering=1)
        self.last_fsync = time.monotonic()

    def write(self, payload):
        self.handle.write(
            json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        )
        now = time.monotonic()
        if now - self.last_fsync >= 1.0:
            self.handle.flush()
            os.fsync(self.handle.fileno())
            self.last_fsync = now

    def close(self):
        try:
            self.handle.flush()
            os.fsync(self.handle.fileno())
        finally:
            self.handle.close()


class ExperimentTelemetry(Node):
    def __init__(self, args):
        super().__init__("go2_experiment_telemetry")
        self.args = args
        self.writer = JsonlWriter(args.output)
        self.started_mono = time.monotonic()
        self.stopping = False
        self.sequence = {}
        self.received = {}
        self.written = {}
        self.last_receive_mono = {}
        self.last_write_mono = {}
        self.ready_written = False
        self.required_streams = [
            "odom",
            "livox_imu",
            "sport",
            "low",
        ]
        self.controller_input_streams = (
            "wireless",
            "wireless_raw",
        )
        self.periods = {
            "odom": 1.0 / max(1.0, args.odom_hz),
            "livox_imu": 1.0 / max(1.0, args.imu_hz),
            "sport": 1.0 / max(1.0, args.sport_hz),
            "low": 1.0 / max(0.2, args.low_hz),
            "wireless": 1.0 / max(1.0, args.wireless_hz),
            "wireless_raw": 1.0 / max(
                1.0,
                args.wireless_hz,
            ),
            "patrol_cmd": 1.0 / max(1.0, args.command_hz),
            "cmd_vel": 1.0 / max(1.0, args.command_hz),
            "sport_request": 1.0 / max(1.0, args.command_hz),
        }

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry,
            "/Odometry",
            lambda msg: self.on_message("odom", msg, self.odom_payload),
            qos,
        )
        self.create_subscription(
            Imu,
            "/livox/imu",
            lambda msg: self.on_message(
                "livox_imu",
                msg,
                self.livox_imu_payload,
            ),
            qos,
        )
        self.create_subscription(
            SportModeState,
            "/lf/sportmodestate",
            lambda msg: self.on_message("sport", msg, self.sport_payload),
            qos,
        )
        self.create_subscription(
            LowState,
            "/lf/lowstate",
            self.on_low_message,
            qos,
        )
        self.create_subscription(
            WirelessController,
            "/lf/wirelesscontroller",
            lambda msg: self.on_message(
                "wireless",
                msg,
                self.wireless_payload,
            ),
            qos,
        )
        # Unitree ROS bridges in the field have exposed this DDS topic both
        # with and without the /lf namespace.  Subscribe to both names and
        # merge them into one stream; KEEP_LAST depth=1 keeps this cheap.
        self.create_subscription(
            WirelessController,
            "/wirelesscontroller",
            lambda msg: self.on_message(
                "wireless",
                msg,
                self.wireless_payload,
            ),
            qos,
        )
        self.create_subscription(
            Twist,
            "/patrol_cmd",
            lambda msg: self.on_message(
                "patrol_cmd",
                msg,
                self.twist_payload,
            ),
            qos,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda msg: self.on_message(
                "cmd_vel",
                msg,
                self.twist_payload,
            ),
            qos,
        )
        self.create_subscription(
            Request,
            "/api/sport/request",
            lambda msg: self.on_message(
                "sport_request",
                msg,
                self.request_payload,
            ),
            qos,
        )
        self.create_timer(1.0, self.write_health)
        self.write_record(
            "recorder_start",
            {
                "pid": os.getpid(),
                "profile": args.profile,
                "sampling_hz": {
                    key: round(1.0 / value, 3)
                    for key, value in self.periods.items()
                },
            },
        )
        self.get_logger().info(
            "experiment telemetry started: %s" % args.output
        )

    def write_record(self, kind, payload, source_stamp=None, receive_mono=None):
        now_wall = time.time()
        now_mono = (
            time.monotonic()
            if receive_mono is None
            else float(receive_mono)
        )
        sequence = self.sequence.get(kind, 0) + 1
        self.sequence[kind] = sequence
        record = {
            "kind": kind,
            "sequence": sequence,
            "wall_time": now_wall,
            "monotonic_s": now_mono,
            "elapsed_s": now_mono - self.started_mono,
            "source_stamp": finite_or_none(source_stamp),
            "data": payload,
        }
        self.writer.write(record)

    def on_message(self, kind, message, payload_builder):
        receive_mono = time.monotonic()
        self.received[kind] = self.received.get(kind, 0) + 1
        self.last_receive_mono[kind] = receive_mono
        if (
            receive_mono - self.last_write_mono.get(kind, 0.0)
            < self.periods[kind]
        ):
            return
        self.last_write_mono[kind] = receive_mono
        try:
            payload = payload_builder(message)
            self.write_record(
                kind,
                payload,
                source_stamp=ros_stamp(message),
                receive_mono=receive_mono,
            )
            self.written[kind] = self.written.get(kind, 0) + 1
        except Exception as exc:  # noqa: BLE001
            self.write_record(
                "serialization_error",
                {"source_kind": kind, "error": repr(exc)},
                receive_mono=receive_mono,
            )

    def on_low_message(self, message):
        self.on_message("low", message, self.low_payload)
        if self.args.profile == "recording":
            self.on_message(
                "wireless_raw",
                message,
                self.wireless_raw_payload,
            )

    @staticmethod
    def odom_payload(message):
        pose = message.pose.pose
        twist = message.twist.twist
        return {
            "frame_id": str(message.header.frame_id),
            "child_frame_id": str(message.child_frame_id),
            "position": {
                "x": finite_or_none(pose.position.x),
                "y": finite_or_none(pose.position.y),
                "z": finite_or_none(pose.position.z),
            },
            "orientation": quaternion_dict(pose.orientation),
            "linear_velocity": {
                "x": finite_or_none(twist.linear.x),
                "y": finite_or_none(twist.linear.y),
                "z": finite_or_none(twist.linear.z),
            },
            "angular_velocity": {
                "x": finite_or_none(twist.angular.x),
                "y": finite_or_none(twist.angular.y),
                "z": finite_or_none(twist.angular.z),
            },
            "pose_covariance_diagonal": [
                finite_or_none(message.pose.covariance[index])
                for index in (0, 7, 14, 21, 28, 35)
            ],
            "twist_covariance_diagonal": [
                finite_or_none(message.twist.covariance[index])
                for index in (0, 7, 14, 21, 28, 35)
            ],
        }

    @staticmethod
    def livox_imu_payload(message):
        return {
            "orientation": quaternion_dict(message.orientation),
            "angular_velocity": {
                "x": finite_or_none(message.angular_velocity.x),
                "y": finite_or_none(message.angular_velocity.y),
                "z": finite_or_none(message.angular_velocity.z),
            },
            "linear_acceleration": {
                "x": finite_or_none(message.linear_acceleration.x),
                "y": finite_or_none(message.linear_acceleration.y),
                "z": finite_or_none(message.linear_acceleration.z),
            },
        }

    @staticmethod
    def sport_payload(message):
        return {
            "error_code": int(message.error_code),
            "mode": int(message.mode),
            "progress": finite_or_none(message.progress),
            "gait_type": int(message.gait_type),
            "foot_raise_height": finite_or_none(message.foot_raise_height),
            "position": float_list(message.position),
            "body_height": finite_or_none(message.body_height),
            "velocity": float_list(message.velocity),
            "yaw_speed": finite_or_none(message.yaw_speed),
            "range_obstacle": float_list(message.range_obstacle),
            "foot_force": integer_list(message.foot_force),
            "foot_position_body": float_list(message.foot_position_body),
            "foot_speed_body": float_list(message.foot_speed_body),
            "imu": imu_state_dict(message.imu_state),
        }

    @staticmethod
    def low_payload(message):
        motors = []
        for index, motor in enumerate(message.motor_state[:12]):
            motors.append(
                {
                    "index": index,
                    "mode": int(motor.mode),
                    "q": finite_or_none(motor.q),
                    "dq": finite_or_none(motor.dq),
                    "ddq": finite_or_none(motor.ddq),
                    "tau_est": finite_or_none(motor.tau_est),
                    "temperature_c": int(motor.temperature),
                    "lost": int(motor.lost),
                }
            )
        bms = message.bms_state
        return {
            "tick": int(message.tick),
            "bandwidth": int(message.bandwidth),
            "power_v": finite_or_none(message.power_v),
            "power_a": finite_or_none(message.power_a),
            "temperature_ntc1_c": int(message.temperature_ntc1),
            "temperature_ntc2_c": int(message.temperature_ntc2),
            "foot_force": integer_list(message.foot_force),
            "foot_force_est": integer_list(message.foot_force_est),
            "fan_frequency": integer_list(message.fan_frequency),
            "wireless_remote_raw": integer_list(
                message.wireless_remote
            ),
            "imu": imu_state_dict(message.imu_state),
            "bms": {
                "status": int(bms.status),
                "soc": int(bms.soc),
                "current": int(bms.current),
                "cycle": int(bms.cycle),
                "bq_ntc": integer_list(bms.bq_ntc),
                "mcu_ntc": integer_list(bms.mcu_ntc),
                "cell_voltages": integer_list(bms.cell_vol),
            },
            "motors": motors,
        }

    @staticmethod
    def wireless_payload(message):
        return {
            "lx": finite_or_none(message.lx),
            "ly": finite_or_none(message.ly),
            "rx": finite_or_none(message.rx),
            "ry": finite_or_none(message.ry),
            "keys": int(message.keys),
        }

    @staticmethod
    def wireless_raw_payload(message):
        return {
            "source": "LowState.wireless_remote",
            "wireless_remote_raw": integer_list(
                message.wireless_remote
            ),
        }

    @staticmethod
    def twist_payload(message):
        return {
            "linear": {
                "x": finite_or_none(message.linear.x),
                "y": finite_or_none(message.linear.y),
                "z": finite_or_none(message.linear.z),
            },
            "angular": {
                "x": finite_or_none(message.angular.x),
                "y": finite_or_none(message.angular.y),
                "z": finite_or_none(message.angular.z),
            },
        }

    @staticmethod
    def request_payload(message):
        parameter = str(message.parameter)
        try:
            decoded = json.loads(parameter)
        except (TypeError, ValueError):
            decoded = None
        return {
            "identity_id": int(message.header.identity.id),
            "api_id": int(message.header.identity.api_id),
            "parameter": parameter,
            "parameter_json": decoded,
            "binary_length": len(message.binary),
        }

    def write_health(self):
        now = time.monotonic()
        kinds = sorted(set(self.periods).union(self.received))
        self.write_record(
            "recorder_health",
            {
                "received": {
                    kind: self.received.get(kind, 0)
                    for kind in kinds
                },
                "written": {
                    kind: self.written.get(kind, 0)
                    for kind in kinds
                },
                "receive_age_s": {
                    kind: (
                        round(
                            now - self.last_receive_mono[kind],
                            6,
                        )
                        if kind in self.last_receive_mono
                        else None
                    )
                    for kind in kinds
                },
            },
            receive_mono=now,
        )
        if (
            not self.ready_written
            and all(
                self.received.get(kind, 0) > 0
                for kind in self.required_streams
            )
            and (
                self.args.profile != "recording"
                or any(
                    self.received.get(kind, 0) > 0
                    for kind in self.controller_input_streams
                )
            )
        ):
            self.ready_written = True
            self.write_record(
                "recorder_ready",
                {
                    "profile": self.args.profile,
                    "required_streams": self.required_streams,
                    "controller_input_alternatives": (
                        list(self.controller_input_streams)
                        if self.args.profile == "recording"
                        else []
                    ),
                    "controller_input_sources_received": [
                        kind
                        for kind in self.controller_input_streams
                        if self.received.get(kind, 0) > 0
                    ],
                    "received": {
                        kind: self.received.get(kind, 0)
                        for kind in self.required_streams
                    },
                },
                receive_mono=now,
            )
            self.get_logger().info(
                "EXPERIMENT_TELEMETRY_READY profile=%s streams=%s"
                % (
                    self.args.profile,
                    ",".join(self.required_streams),
                )
            )

    def close(self):
        if self.stopping:
            return
        self.stopping = True
        self.write_health()
        self.write_record(
            "recorder_stop",
            {
                "received": self.received,
                "written": self.written,
            },
        )
        self.writer.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--profile",
        choices=("recording", "patrol"),
        default="recording",
    )
    parser.add_argument("--odom-hz", type=float, default=20.0)
    parser.add_argument("--imu-hz", type=float, default=20.0)
    parser.add_argument("--sport-hz", type=float, default=20.0)
    parser.add_argument("--low-hz", type=float, default=1.0)
    parser.add_argument("--wireless-hz", type=float, default=20.0)
    parser.add_argument("--command-hz", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    rclpy.init()
    node = ExperimentTelemetry(args)

    def stop(_signum, _frame):
        node.stopping = True
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while rclpy.ok() and not node.stopping:
            # KEEP_LAST depth=1 plus bounded spin frequency deliberately drops
            # redundant high-rate state samples before Python deserialization
            # can compete with FAST-LIO.
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stopping = False
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
