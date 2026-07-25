#!/usr/bin/env python3
"""Trace the deployed Go2_2 follower without changing its control math.

This module subclasses the already installed ``WaypointFollower`` and calls
its odometry callback and control loop unchanged.  The only interception is a
publisher proxy that records the exact Twist handed to rclpy.  It is kept as a
separate executable so a diagnostic deployment does not replace the controller
source that produced the earlier patrols.
"""

import hashlib
import inspect
import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy

from go2_fastlio_patrol import waypoint_follower_go2_2 as base_module


BaseFollower = getattr(base_module, "WaypointFollower", None)
if BaseFollower is None:
    raise RuntimeError(
        "BASE_FOLLOWER_CLASS_MISSING: expected deployed "
        "go2_fastlio_patrol.waypoint_follower_go2_2.WaypointFollower"
    )


def finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def source_stamp_seconds(message):
    try:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if value > 0.0 and math.isfinite(value) else None


def sha256_file(path):
    digest = hashlib.sha256()
    try:
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError:
        return None
    return digest.hexdigest()


def nearest_local_projection(route, nearest_index, x, y, window):
    if len(route) < 2:
        return {}
    first = max(0, int(nearest_index) - int(window) - 1)
    last = min(len(route) - 2, int(nearest_index) + int(window) + 1)
    best = None
    for index in range(first, last + 1):
        start = route[index]
        end = route[index + 1]
        dx = float(end["x"]) - float(start["x"])
        dy = float(end["y"]) - float(start["y"])
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            fraction = 0.0
        else:
            fraction = max(
                0.0,
                min(
                    1.0,
                    (
                        (x - float(start["x"])) * dx
                        + (y - float(start["y"])) * dy
                    )
                    / length_squared,
                ),
            )
        projected_x = float(start["x"]) + fraction * dx
        projected_y = float(start["y"]) + fraction * dy
        error_x = x - projected_x
        error_y = y - projected_y
        distance = math.hypot(error_x, error_y)
        segment_length = math.sqrt(length_squared)
        signed = (
            (dx * error_y - dy * error_x) / segment_length
            if segment_length > 1e-9
            else 0.0
        )
        candidate = {
            "segment": index,
            "fraction": fraction,
            "x": projected_x,
            "y": projected_y,
            "heading": math.atan2(dy, dx),
            "distance_m": distance,
            "signed_cross_track_m": signed,
        }
        if best is None or distance < best["distance_m"]:
            best = candidate
    return best or {}


class RecordingPublisher:
    """Forward every publish unchanged and retain the exact Twist values."""

    def __init__(self, publisher, owner):
        self._publisher = publisher
        self._owner = owner

    def publish(self, message):
        self._owner.capture_published_command(message)
        return self._publisher.publish(message)

    def __getattr__(self, name):
        return getattr(self._publisher, name)


class TracedWaypointFollower(BaseFollower):
    def __init__(self):
        self.trace_handle = None
        self.trace_closed = False
        self.trace_write_error_reported = False
        self.odom_callback_sequence = 0
        self.control_sequence = 0
        self.last_odom_receive_wall = None
        self.last_odom_receive_mono = None
        self.last_odom_source_stamp = None
        self.last_odom_callback_ms = None
        self.current_z = None
        self.last_published_command = None
        self.publish_count_in_cycle = 0
        super().__init__()

        original_publisher = self.pub
        self.pub = RecordingPublisher(original_publisher, self)
        if not self.has_parameter("trace_file"):
            self.declare_parameter("trace_file", "")
        self.trace_file = str(
            self.get_parameter("trace_file").value
        ).strip()
        if not self.trace_file:
            raise RuntimeError("TRACE_FILE_REQUIRED")
        trace_path = Path(self.trace_file)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.trace_handle = trace_path.open("w", buffering=1)

        base_source = inspect.getsourcefile(BaseFollower) or ""
        self.write_trace(
            {
                "kind": "trace_start",
                "wall_time": time.time(),
                "monotonic_s": time.monotonic(),
                "base_class": "%s.%s"
                % (BaseFollower.__module__, BaseFollower.__name__),
                "base_source": base_source,
                "base_source_sha256": sha256_file(base_source),
                "trace_pid": os.getpid(),
                "control_period_s": 0.05,
            }
        )
        self.get_logger().info(
            "FOLLOWER_EXACT_TRACE_READY file=%s base_sha256=%s"
            % (self.trace_file, sha256_file(base_source) or "unavailable")
        )

    def write_trace(self, record):
        if self.trace_handle is None or self.trace_closed:
            return
        try:
            self.trace_handle.write(
                json.dumps(
                    record,
                    separators=(",", ":"),
                    sort_keys=True,
                    allow_nan=False,
                )
                + "\n"
            )
        except (OSError, TypeError, ValueError) as exc:
            if not self.trace_write_error_reported:
                self.trace_write_error_reported = True
                self.get_logger().error(
                    "FOLLOWER_TRACE_WRITE_FAILED %s" % exc
                )

    def capture_published_command(self, message):
        self.publish_count_in_cycle += 1
        self.last_published_command = {
            "vx": finite_or_none(message.linear.x),
            "vy": finite_or_none(message.linear.y),
            "vz": finite_or_none(message.linear.z),
            "roll_rate": finite_or_none(message.angular.x),
            "pitch_rate": finite_or_none(message.angular.y),
            "yaw_rate": finite_or_none(message.angular.z),
        }

    def odom_callback(self, message):
        callback_start = time.monotonic()
        self.odom_callback_sequence += 1
        self.last_odom_receive_wall = time.time()
        self.last_odom_receive_mono = callback_start
        self.last_odom_source_stamp = source_stamp_seconds(message)
        self.current_z = finite_or_none(message.pose.pose.position.z)
        super().odom_callback(message)
        self.last_odom_callback_ms = (
            time.monotonic() - callback_start
        ) * 1000.0

    def control_loop(self):
        control_start_mono = time.monotonic()
        control_start_wall = time.time()
        self.control_sequence += 1
        self.last_published_command = None
        self.publish_count_in_cycle = 0

        super().control_loop()
        compute_ms = (time.monotonic() - control_start_mono) * 1000.0

        pose_available = (
            self.current_x is not None
            and self.current_y is not None
            and self.current_yaw is not None
        )
        projection = {}
        target = {}
        nearest = {}
        selected_alpha = None
        nearest_distance = None
        target_distance = None
        if pose_available:
            try:
                nearest_point = self.route[self.nearest_index]
                nearest_distance = math.hypot(
                    float(nearest_point["x"]) - float(self.current_x),
                    float(nearest_point["y"]) - float(self.current_y),
                )
                nearest = {
                    "index": int(self.nearest_index),
                    "x": finite_or_none(nearest_point["x"]),
                    "y": finite_or_none(nearest_point["y"]),
                    "distance_m": finite_or_none(nearest_distance),
                }
                target_point = self.route[self.target_index]
                target_dx = (
                    float(target_point["x"]) - float(self.current_x)
                )
                target_dy = (
                    float(target_point["y"]) - float(self.current_y)
                )
                target_distance = math.hypot(target_dx, target_dy)
                target_angle = math.atan2(target_dy, target_dx)
                selected_alpha = base_module.normalize_angle(
                    target_angle - float(self.current_yaw)
                )
                target = {
                    "index": int(self.target_index),
                    "x": finite_or_none(target_point["x"]),
                    "y": finite_or_none(target_point["y"]),
                    "distance_m": finite_or_none(target_distance),
                    "angle": finite_or_none(target_angle),
                }
                projection = nearest_local_projection(
                    self.route,
                    self.nearest_index,
                    float(self.current_x),
                    float(self.current_y),
                    self.search_window,
                )
            except (IndexError, KeyError, TypeError, ValueError):
                pass

        command = self.last_published_command or {
            "vx": None,
            "vy": None,
            "vz": None,
            "roll_rate": None,
            "pitch_rate": None,
            "yaw_rate": None,
        }
        is_stop = all(
            value is not None and abs(value) <= 1e-9
            for value in (
                command["vx"],
                command["vy"],
                command["yaw_rate"],
            )
        )
        odom_to_control_ms = (
            (control_start_mono - self.last_odom_receive_mono)
            * 1000.0
            if self.last_odom_receive_mono is not None
            else None
        )
        odom_stamp_age_ms = (
            (control_start_wall - self.last_odom_source_stamp)
            * 1000.0
            if self.last_odom_source_stamp is not None
            else None
        )
        self.write_trace(
            {
                "kind": "control",
                "wall_time": control_start_wall,
                "monotonic_s": control_start_mono,
                "control_sequence": self.control_sequence,
                "odom_callback_sequence": self.odom_callback_sequence,
                "odom_source_stamp": self.last_odom_source_stamp,
                "odom_to_control_ms": finite_or_none(
                    odom_to_control_ms
                ),
                "odom_stamp_age_ms": finite_or_none(
                    odom_stamp_age_ms
                ),
                "odom_callback_compute_ms": finite_or_none(
                    self.last_odom_callback_ms
                ),
                "compute_to_trace_ms": finite_or_none(compute_ms),
                "pose": {
                    "x": finite_or_none(self.current_x),
                    "y": finite_or_none(self.current_y),
                    "z": finite_or_none(self.current_z),
                    "yaw": finite_or_none(self.current_yaw),
                },
                "nearest": nearest,
                "target": target,
                "projection": projection,
                "control": {
                    "source": "deployed_go2_2_nearest_lookahead",
                    "cross_track_m": finite_or_none(
                        projection.get("distance_m")
                    ),
                    "signed_cross_track_m": finite_or_none(
                        projection.get("signed_cross_track_m")
                    ),
                    "selected_alpha": finite_or_none(
                        selected_alpha
                    ),
                    "cmd_vx": command["vx"],
                    "cmd_vy": command["vy"],
                    "cmd_yaw_rate": command["yaw_rate"],
                    "is_stop": bool(is_stop),
                    "finished": bool(self.finished),
                    "direction": int(self.direction),
                    "publish_count": self.publish_count_in_cycle,
                    "nearest_distance_m": finite_or_none(
                        nearest_distance
                    ),
                    "target_distance_m": finite_or_none(
                        target_distance
                    ),
                },
            }
        )

    def close_trace(self, reason):
        if self.trace_handle is None or self.trace_closed:
            return
        self.write_trace(
            {
                "kind": "trace_stop",
                "wall_time": time.time(),
                "monotonic_s": time.monotonic(),
                "reason": reason,
                "control_sequence": self.control_sequence,
                "odom_callback_sequence": self.odom_callback_sequence,
            }
        )
        try:
            self.trace_handle.flush()
            os.fsync(self.trace_handle.fileno())
            self.trace_handle.close()
        except OSError:
            pass
        self.trace_closed = True

    def destroy_node(self):
        self.close_trace("destroy_node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TracedWaypointFollower()

    def request_shutdown(signum, _frame):
        node.close_trace("signal_%s" % signum)
        try:
            node.publish_stop()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "traced waypoint_follower stopped by Ctrl+C"
        )
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
