#!/usr/bin/env python3
"""Trace Go2_2 with a startup-interlocked, gravity-level control frame.

This module subclasses the already installed ``WaypointFollower`` and calls
its odometry callback and control loop unchanged after a startup interlock is
released.  Before release, the wrapper publishes zero without calling the base
control loop, allowing ROS discovery and odometry subscriptions to settle
without advancing the controller's stuck/reverse state.  It is kept as a
separate executable so the deployed controller math remains auditable.

In horizontal-frame mode, the wrapper transforms both FAST-LIO position and
its complete orientation through one frozen map-to-ground rotation.  The
recorded route is already expressed in the same gravity-level convention and
is rigidly anchored once at startup.  Unitree IMU data is used only to prove
the startup plane; its yaw never replaces FAST-LIO yaw during motion.
"""

import copy
import hashlib
import inspect
import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState

from go2_fastlio_patrol import waypoint_follower_go2_2 as base_module
from body_yaw_alignment import BodyYawAlignment
from horizontal_frame import (
    HorizontalFrameEstimator,
    align_route_to_pose,
    quaternion_yaw,
)


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
    """Forward every post-interlock publish and retain exact Twist values."""

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
        self.odom_ready_consecutive = 0
        self.odom_ready_reported = False
        self.motion_release_reported = False
        self.raw_lio_yaw = None
        self.body_yaw = None
        self.body_yaw_receive_mono = None
        self.body_yaw_receive_wall = None
        self.body_yaw_hold_reported = False
        self.body_acceleration = None
        self.body_gyro = None
        self.lidar_acceleration = None
        self.lidar_gyro = None
        self.lidar_imu_receive_mono = None
        self.use_horizontal_frame = False
        self.horizontal_estimator = None
        self.horizontal_metadata = None
        self.horizontal_route_anchored = False
        self.horizontal_anchor_rotation = None
        self.horizontal_ready_reported = False
        self.raw_lio_position = None
        self.raw_lio_quaternion = None
        super().__init__()

        original_publisher = self.pub
        self.pub = RecordingPublisher(original_publisher, self)
        if not self.has_parameter("trace_file"):
            self.declare_parameter("trace_file", "")
        if not self.has_parameter("motion_enable_file"):
            self.declare_parameter("motion_enable_file", "")
        if not self.has_parameter("use_body_yaw_alignment"):
            self.declare_parameter("use_body_yaw_alignment", False)
        if not self.has_parameter("body_yaw_topic"):
            self.declare_parameter(
                "body_yaw_topic",
                "/lf/sportmodestate",
            )
        if not self.has_parameter("body_yaw_alignment_samples"):
            self.declare_parameter("body_yaw_alignment_samples", 10)
        if not self.has_parameter("body_yaw_alignment_max_spread_deg"):
            self.declare_parameter(
                "body_yaw_alignment_max_spread_deg",
                2.0,
            )
        if not self.has_parameter("body_yaw_max_age"):
            self.declare_parameter("body_yaw_max_age", 0.25)
        if not self.has_parameter("use_horizontal_frame"):
            self.declare_parameter("use_horizontal_frame", False)
        if not self.has_parameter("horizontal_frame_metadata"):
            self.declare_parameter("horizontal_frame_metadata", "")
        if not self.has_parameter("horizontal_imu_topic"):
            self.declare_parameter("horizontal_imu_topic", "/livox/imu")
        if not self.has_parameter("horizontal_frame_samples"):
            self.declare_parameter("horizontal_frame_samples", 15)
        if not self.has_parameter("horizontal_frame_max_spread_deg"):
            self.declare_parameter(
                "horizontal_frame_max_spread_deg", 1.5
            )
        if not self.has_parameter(
            "horizontal_frame_max_source_disagreement_deg"
        ):
            self.declare_parameter(
                "horizontal_frame_max_source_disagreement_deg", 3.0
            )
        if not self.has_parameter("horizontal_frame_max_gyro"):
            self.declare_parameter("horizontal_frame_max_gyro", 0.08)
        if not self.has_parameter("horizontal_imu_max_age"):
            self.declare_parameter("horizontal_imu_max_age", 0.20)
        self.trace_file = str(
            self.get_parameter("trace_file").value
        ).strip()
        self.motion_enable_file = str(
            self.get_parameter("motion_enable_file").value
        ).strip()
        self.use_body_yaw_alignment = bool(
            self.get_parameter("use_body_yaw_alignment").value
        )
        self.use_horizontal_frame = bool(
            self.get_parameter("use_horizontal_frame").value
        )
        if self.use_horizontal_frame and self.use_body_yaw_alignment:
            raise RuntimeError(
                "HORIZONTAL_FRAME_AND_BODY_YAW_ADAPTER_ARE_MUTUALLY_EXCLUSIVE"
            )
        self.body_yaw_topic = str(
            self.get_parameter("body_yaw_topic").value
        ).strip()
        self.body_yaw_max_age = max(
            0.05,
            float(self.get_parameter("body_yaw_max_age").value),
        )
        self.body_yaw_alignment = BodyYawAlignment(
            minimum_samples=int(
                self.get_parameter(
                    "body_yaw_alignment_samples"
                ).value
            ),
            max_spread_rad=math.radians(
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "body_yaw_alignment_max_spread_deg"
                        ).value
                    ),
                )
            ),
        )
        self.horizontal_frame_metadata_path = str(
            self.get_parameter("horizontal_frame_metadata").value
        ).strip()
        self.horizontal_imu_topic = str(
            self.get_parameter("horizontal_imu_topic").value
        ).strip()
        self.horizontal_imu_max_age = max(
            0.05,
            float(self.get_parameter("horizontal_imu_max_age").value),
        )
        self.canonical_route = [dict(point) for point in self.route]
        if self.use_horizontal_frame:
            if not self.horizontal_frame_metadata_path:
                raise RuntimeError("HORIZONTAL_FRAME_METADATA_REQUIRED")
            metadata_path = Path(self.horizontal_frame_metadata_path)
            try:
                self.horizontal_metadata = json.loads(
                    metadata_path.read_text()
                )
            except (OSError, ValueError) as exc:
                raise RuntimeError(
                    "HORIZONTAL_FRAME_METADATA_UNREADABLE: %s" % exc
                )
            if (
                self.horizontal_metadata.get("schema")
                != "go2.horizontal_route.v1"
            ):
                raise RuntimeError("HORIZONTAL_FRAME_METADATA_SCHEMA_INVALID")
            expected_route_sha = str(
                self.horizontal_metadata.get(
                    "horizontal_route_sha256", ""
                )
            )
            actual_route_sha = sha256_file(self.route_file)
            if (
                not expected_route_sha
                or expected_route_sha != actual_route_sha
            ):
                raise RuntimeError(
                    "HORIZONTAL_ROUTE_HASH_MISMATCH expected=%s actual=%s"
                    % (expected_route_sha or "missing", actual_route_sha)
                )
            calibration = self.horizontal_metadata.get(
                "mount_and_gravity_calibration", {}
            )
            self.horizontal_estimator = HorizontalFrameEstimator(
                q_sensor_from_body=calibration.get(
                    "q_sensor_from_body_xyzw"
                ),
                q_lidar_gravity_correction=calibration.get(
                    "q_lidar_gravity_correction_xyzw",
                    (0.0, 0.0, 0.0, 1.0),
                ),
                minimum_samples=int(
                    self.get_parameter(
                        "horizontal_frame_samples"
                    ).value
                ),
                maximum_spread_rad=math.radians(
                    max(
                        0.0,
                        float(
                            self.get_parameter(
                                "horizontal_frame_max_spread_deg"
                            ).value
                        ),
                    )
                ),
                maximum_source_disagreement_rad=math.radians(
                    max(
                        0.0,
                        float(
                            self.get_parameter(
                                "horizontal_frame_max_source_disagreement_deg"
                            ).value
                        ),
                    )
                ),
                maximum_gyro_rad_s=max(
                    0.0,
                    float(
                        self.get_parameter(
                            "horizontal_frame_max_gyro"
                        ).value
                    ),
                ),
            )
        if not self.trace_file:
            raise RuntimeError("TRACE_FILE_REQUIRED")
        if not self.motion_enable_file:
            raise RuntimeError("MOTION_ENABLE_FILE_REQUIRED")
        trace_path = Path(self.trace_file)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.trace_handle = trace_path.open("w", buffering=1)
        self.body_yaw_sub = None
        if self.use_body_yaw_alignment or self.use_horizontal_frame:
            if not self.body_yaw_topic:
                raise RuntimeError("BODY_IMU_TOPIC_REQUIRED")
            body_yaw_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.body_yaw_sub = self.create_subscription(
                SportModeState,
                self.body_yaw_topic,
                self.body_yaw_callback,
                body_yaw_qos,
            )
        self.lidar_imu_sub = None
        if self.use_horizontal_frame:
            if not self.horizontal_imu_topic:
                raise RuntimeError("HORIZONTAL_IMU_TOPIC_REQUIRED")
            horizontal_imu_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.lidar_imu_sub = self.create_subscription(
                Imu,
                self.horizontal_imu_topic,
                self.lidar_imu_callback,
                horizontal_imu_qos,
            )

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
                "motion_enable_file": self.motion_enable_file,
                "startup_policy": (
                    "publish_zero_without_calling_base_control_until_enabled"
                ),
                "heading_feedback_policy": (
                    (
                        "full_fast_lio_quaternion_in_frozen_gravity_level_frame"
                        if self.use_horizontal_frame
                        else "unitree_body_yaw_with_startup_lio_offset"
                    )
                    if (
                        self.use_horizontal_frame
                        or self.use_body_yaw_alignment
                    )
                    else "raw_fast_lio_euler_yaw"
                ),
                "body_yaw_topic": (
                    self.body_yaw_topic
                    if (
                        self.use_body_yaw_alignment
                        or self.use_horizontal_frame
                    )
                    else None
                ),
                "body_yaw_max_age_s": self.body_yaw_max_age,
                "body_yaw_qos": "keep_last_1_best_effort",
                "body_yaw_alignment_samples": (
                    self.body_yaw_alignment.minimum_samples
                ),
                "body_yaw_alignment_max_spread_deg": math.degrees(
                    self.body_yaw_alignment.max_spread_rad
                ),
                "horizontal_frame_enabled": self.use_horizontal_frame,
                "horizontal_frame_metadata": (
                    self.horizontal_frame_metadata_path
                    if self.use_horizontal_frame
                    else None
                ),
                "horizontal_frame_metadata_sha256": (
                    sha256_file(self.horizontal_frame_metadata_path)
                    if self.use_horizontal_frame
                    else None
                ),
                "horizontal_imu_topic": (
                    self.horizontal_imu_topic
                    if self.use_horizontal_frame
                    else None
                ),
                "horizontal_imu_max_age_s": (
                    self.horizontal_imu_max_age
                    if self.use_horizontal_frame
                    else None
                ),
                "horizontal_frame_calibration": (
                    self.horizontal_estimator.diagnostics()
                    if self.use_horizontal_frame
                    else None
                ),
            }
        )
        self.get_logger().info(
            "FOLLOWER_EXACT_TRACE_READY file=%s base_sha256=%s"
            % (self.trace_file, sha256_file(base_source) or "unavailable")
        )
        self.get_logger().info(
            "FOLLOWER_MOTION_INTERLOCK_ACTIVE file=%s"
            % self.motion_enable_file
        )
        if self.use_body_yaw_alignment:
            self.get_logger().info(
                "FOLLOWER_BODY_YAW_ALIGNMENT_ACTIVE topic=%s "
                "samples=%d max_spread_deg=%.3f max_age_s=%.3f"
                % (
                    self.body_yaw_topic,
                    self.body_yaw_alignment.minimum_samples,
                    math.degrees(
                        self.body_yaw_alignment.max_spread_rad
                    ),
                    self.body_yaw_max_age,
                )
            )
        if self.use_horizontal_frame:
            self.get_logger().info(
                "FOLLOWER_HORIZONTAL_FRAME_CALIBRATING "
                "metadata=%s lidar_imu=%s body_imu=%s samples=%d "
                "max_spread_deg=%.3f max_source_disagreement_deg=%.3f"
                % (
                    self.horizontal_frame_metadata_path,
                    self.horizontal_imu_topic,
                    self.body_yaw_topic,
                    self.horizontal_estimator.minimum_samples,
                    math.degrees(
                        self.horizontal_estimator.maximum_spread_rad
                    ),
                    math.degrees(
                        self.horizontal_estimator
                        .maximum_source_disagreement_rad
                    ),
                )
            )

    @staticmethod
    def body_yaw_from_message(message):
        try:
            value = float(message.imu_state.rpy[2])
        except (AttributeError, IndexError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def body_yaw_callback(self, message):
        value = self.body_yaw_from_message(message)
        try:
            body_acceleration = tuple(
                float(item) for item in message.imu_state.accelerometer
            )
            body_gyro = tuple(
                float(item) for item in message.imu_state.gyroscope
            )
        except (AttributeError, TypeError, ValueError):
            body_acceleration = None
            body_gyro = None
        if value is not None:
            self.body_yaw = base_module.normalize_angle(value)
        if body_acceleration is not None and len(body_acceleration) == 3:
            self.body_acceleration = body_acceleration
        if body_gyro is not None and len(body_gyro) == 3:
            self.body_gyro = body_gyro
        self.body_yaw_receive_mono = time.monotonic()
        self.body_yaw_receive_wall = time.time()
        if (
            self.use_body_yaw_alignment
            and self.body_yaw is not None
            and self.body_yaw_alignment.ready
        ):
            aligned = self.body_yaw_alignment.aligned_yaw(
                self.body_yaw
            )
            if aligned is not None:
                self.current_yaw = aligned

    def lidar_imu_callback(self, message):
        try:
            self.lidar_acceleration = (
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            )
            self.lidar_gyro = (
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            )
        except (AttributeError, TypeError, ValueError):
            return
        self.lidar_imu_receive_mono = time.monotonic()

    def body_yaw_age(self, now_mono=None):
        if self.body_yaw_receive_mono is None:
            return None
        if now_mono is None:
            now_mono = time.monotonic()
        return max(0.0, now_mono - self.body_yaw_receive_mono)

    def body_yaw_usable(self, now_mono=None):
        if not self.use_body_yaw_alignment:
            return True
        age = self.body_yaw_age(now_mono)
        return (
            self.body_yaw_alignment.ready
            and self.body_yaw is not None
            and age is not None
            and age <= self.body_yaw_max_age
        )

    def horizontal_frame_usable(self):
        if not self.use_horizontal_frame:
            return True
        return (
            self.horizontal_estimator is not None
            and self.horizontal_estimator.ready
            and self.horizontal_route_anchored
        )

    def horizontal_inputs_fresh(self, now_mono=None):
        if not self.use_horizontal_frame:
            return True
        if now_mono is None:
            now_mono = time.monotonic()
        body_age = self.body_yaw_age(now_mono)
        lidar_age = (
            now_mono - self.lidar_imu_receive_mono
            if self.lidar_imu_receive_mono is not None
            else None
        )
        return (
            self.body_acceleration is not None
            and self.body_gyro is not None
            and self.lidar_acceleration is not None
            and self.lidar_gyro is not None
            and body_age is not None
            and 0.0 <= body_age <= self.horizontal_imu_max_age
            and lidar_age is not None
            and 0.0 <= lidar_age <= self.horizontal_imu_max_age
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

    @staticmethod
    def odometry_quaternion(message):
        orientation = message.pose.pose.orientation
        return (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    @staticmethod
    def odometry_position(message):
        position = message.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def horizontal_odometry(self, message):
        """Return one fully transformed message, anchoring the route once."""
        q_map_from_sensor = self.odometry_quaternion(message)
        position_map = self.odometry_position(message)
        position_ground = self.horizontal_estimator.transform_position(
            position_map
        )
        q_ground_from_body = (
            self.horizontal_estimator.transform_body_orientation(
                q_map_from_sensor
            )
        )
        if position_ground is None or q_ground_from_body is None:
            return None
        body_yaw = quaternion_yaw(q_ground_from_body)
        if body_yaw is None:
            return None
        if not self.horizontal_route_anchored:
            self.route, self.horizontal_anchor_rotation = (
                align_route_to_pose(
                    self.canonical_route,
                    position_ground[0],
                    position_ground[1],
                    body_yaw,
                )
            )
            self.horizontal_route_anchored = True
            self.get_logger().info(
                "FOLLOWER_HORIZONTAL_ROUTE_ANCHORED "
                "x=%.6f y=%.6f yaw_deg=%.6f rotation_deg=%.6f"
                % (
                    position_ground[0],
                    position_ground[1],
                    math.degrees(body_yaw),
                    math.degrees(self.horizontal_anchor_rotation),
                )
            )
            self.write_trace(
                {
                    "kind": "horizontal_route_anchored",
                    "wall_time": time.time(),
                    "monotonic_s": time.monotonic(),
                    "anchor_x": position_ground[0],
                    "anchor_y": position_ground[1],
                    "anchor_yaw_deg": math.degrees(body_yaw),
                    "route_rotation_deg": math.degrees(
                        self.horizontal_anchor_rotation
                    ),
                    "canonical_start": self.canonical_route[0],
                    "aligned_start": self.route[0],
                }
            )

        transformed = copy.deepcopy(message)
        transformed.pose.pose.position.x = position_ground[0]
        transformed.pose.pose.position.y = position_ground[1]
        transformed.pose.pose.position.z = position_ground[2]
        transformed.pose.pose.orientation.x = q_ground_from_body[0]
        transformed.pose.pose.orientation.y = q_ground_from_body[1]
        transformed.pose.pose.orientation.z = q_ground_from_body[2]
        transformed.pose.pose.orientation.w = q_ground_from_body[3]
        return transformed

    def odom_callback(self, message):
        callback_start = time.monotonic()
        previous_source_stamp = self.last_odom_source_stamp
        self.odom_callback_sequence += 1
        self.last_odom_receive_wall = time.time()
        self.last_odom_receive_mono = callback_start
        self.last_odom_source_stamp = source_stamp_seconds(message)
        self.raw_lio_position = self.odometry_position(message)
        self.raw_lio_quaternion = self.odometry_quaternion(message)
        self.raw_lio_yaw = finite_or_none(
            quaternion_yaw(self.raw_lio_quaternion)
        )
        controller_message = message
        if self.use_horizontal_frame:
            if (
                not self.horizontal_estimator.ready
                and self.horizontal_inputs_fresh(callback_start)
            ):
                locked_now = self.horizontal_estimator.add_sample(
                    self.raw_lio_quaternion,
                    self.lidar_acceleration,
                    self.body_acceleration,
                    self.lidar_gyro,
                    self.body_gyro,
                )
                if locked_now:
                    diagnostics = self.horizontal_estimator.diagnostics()
                    self.horizontal_ready_reported = True
                    self.get_logger().info(
                        "FOLLOWER_HORIZONTAL_FRAME_READY "
                        "samples=%d spread_deg=%.6f "
                        "source_disagreement_deg=%.6f "
                        "map_up=(%.9f,%.9f,%.9f)"
                        % (
                            diagnostics["sample_count"],
                            diagnostics["spread_deg"],
                            diagnostics["source_disagreement_deg"],
                            diagnostics["map_up"][0],
                            diagnostics["map_up"][1],
                            diagnostics["map_up"][2],
                        )
                    )
                    self.write_trace(
                        {
                            "kind": "horizontal_frame_ready",
                            "wall_time": time.time(),
                            "monotonic_s": time.monotonic(),
                            "calibration": diagnostics,
                        }
                    )
                    # The transform is intentionally frozen.  Stop consuming
                    # the 200 Hz calibration IMU so it cannot add runtime
                    # controller load or turn this into a dynamic blend.
                    if self.lidar_imu_sub is not None:
                        self.destroy_subscription(self.lidar_imu_sub)
                        self.lidar_imu_sub = None
                        self.get_logger().info(
                            "FOLLOWER_HORIZONTAL_CALIBRATION_INPUT_FROZEN "
                            "lidar_imu_subscription=closed"
                        )
            if self.horizontal_estimator.ready:
                controller_message = self.horizontal_odometry(message)
            else:
                controller_message = None

        if controller_message is not None:
            super().odom_callback(controller_message)
            self.current_z = finite_or_none(
                controller_message.pose.pose.position.z
            )
        body_yaw_age = self.body_yaw_age(callback_start)
        if (
            self.use_body_yaw_alignment
            and self.raw_lio_yaw is not None
            and self.body_yaw is not None
            and body_yaw_age is not None
            and body_yaw_age <= self.body_yaw_max_age
        ):
            locked_now = self.body_yaw_alignment.add_pair(
                self.raw_lio_yaw,
                self.body_yaw,
            )
            if locked_now:
                self.get_logger().info(
                    "FOLLOWER_BODY_YAW_READY offset_deg=%.6f "
                    "spread_deg=%.6f samples=%d"
                    % (
                        math.degrees(
                            self.body_yaw_alignment.offset_rad
                        ),
                        math.degrees(
                            self.body_yaw_alignment.spread_rad
                        ),
                        self.body_yaw_alignment.sample_count,
                    )
                )
                self.write_trace(
                    {
                        "kind": "body_yaw_alignment_ready",
                        "wall_time": time.time(),
                        "monotonic_s": time.monotonic(),
                        "offset_deg": math.degrees(
                            self.body_yaw_alignment.offset_rad
                        ),
                        "spread_deg": math.degrees(
                            self.body_yaw_alignment.spread_rad
                        ),
                        "sample_count": (
                            self.body_yaw_alignment.sample_count
                        ),
                    }
                )
            aligned = self.body_yaw_alignment.aligned_yaw(
                self.body_yaw
            )
            if aligned is not None:
                self.current_yaw = aligned
        self.last_odom_callback_ms = (
            time.monotonic() - callback_start
        ) * 1000.0
        source_age = (
            time.time() - self.last_odom_source_stamp
            if self.last_odom_source_stamp is not None
            else None
        )
        source_gap = (
            self.last_odom_source_stamp - previous_source_stamp
            if (
                self.last_odom_source_stamp is not None
                and previous_source_stamp is not None
            )
            else None
        )
        fresh = (
            source_age is not None
            and -0.10 <= source_age <= 0.50
            and self.horizontal_frame_usable()
            and controller_message is not None
            and (
                source_gap is None
                or 0.05 <= source_gap <= 0.35
            )
        )
        self.odom_ready_consecutive = (
            self.odom_ready_consecutive + 1 if fresh else 0
        )
        if (
            not self.odom_ready_reported
            and self.odom_ready_consecutive >= 5
        ):
            self.odom_ready_reported = True
            self.get_logger().info(
                "FOLLOWER_ODOM_READY callbacks=%d age_ms=%.3f "
                "source_gap_ms=%.3f"
                % (
                    self.odom_callback_sequence,
                    source_age * 1000.0,
                    (
                        source_gap * 1000.0
                        if source_gap is not None
                        else 0.0
                    ),
                )
            )

    def control_loop(self):
        control_start_mono = time.monotonic()
        control_start_wall = time.time()
        self.control_sequence += 1
        self.last_published_command = None
        self.publish_count_in_cycle = 0

        motion_requested = os.path.isfile(self.motion_enable_file)
        current_receive_age = (
            control_start_mono - self.last_odom_receive_mono
            if self.last_odom_receive_mono is not None
            else None
        )
        current_stamp_age = (
            control_start_wall - self.last_odom_source_stamp
            if self.last_odom_source_stamp is not None
            else None
        )
        release_fresh = (
            self.odom_ready_reported
            and current_receive_age is not None
            and 0.0 <= current_receive_age <= 0.35
            and current_stamp_age is not None
            and -0.10 <= current_stamp_age <= 0.50
            and self.body_yaw_usable(control_start_mono)
            and self.horizontal_frame_usable()
        )
        motion_enabled = (
            motion_requested
            and (
                self.motion_release_reported
                or release_fresh
            )
            and self.body_yaw_usable(control_start_mono)
            and self.horizontal_frame_usable()
        )
        if (
            self.use_body_yaw_alignment
            and motion_requested
            and self.motion_release_reported
            and not motion_enabled
        ):
            if not self.body_yaw_hold_reported:
                self.body_yaw_hold_reported = True
                self.get_logger().error(
                    "FOLLOWER_BODY_YAW_STALE_HOLD age_s=%s; "
                    "publishing zero until fresh body yaw returns"
                    % (
                        "%.3f" % self.body_yaw_age(control_start_mono)
                        if self.body_yaw_age(control_start_mono)
                        is not None
                        else "unavailable"
                    )
                )
        elif self.body_yaw_hold_reported and motion_enabled:
            self.body_yaw_hold_reported = False
            self.get_logger().info(
                "FOLLOWER_BODY_YAW_RECOVERED age_s=%.3f"
                % self.body_yaw_age(control_start_mono)
            )
        if motion_enabled:
            if not self.motion_release_reported:
                self.motion_release_reported = True
                self.get_logger().info(
                    "FOLLOWER_MOTION_INTERLOCK_RELEASED file=%s"
                    " receive_age_ms=%.3f stamp_age_ms=%.3f"
                    % (
                        self.motion_enable_file,
                        current_receive_age * 1000.0,
                        current_stamp_age * 1000.0,
                    )
                )
                self.write_trace(
                    {
                        "kind": "motion_interlock_released",
                        "wall_time": control_start_wall,
                        "monotonic_s": control_start_mono,
                        "odom_callback_sequence": (
                            self.odom_callback_sequence
                        ),
                        "odom_receive_age_ms": (
                            current_receive_age * 1000.0
                        ),
                        "odom_stamp_age_ms": (
                            current_stamp_age * 1000.0
                        ),
                    }
                )
            super().control_loop()
        else:
            # Warm the ROS participant and odometry subscription without
            # advancing the deployed controller's stuck/reverse state.
            self.pub.publish(Twist())
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
                    "raw_lio_yaw": finite_or_none(
                        self.raw_lio_yaw
                    ),
                    "body_yaw": finite_or_none(self.body_yaw),
                    "body_yaw_aligned": finite_or_none(
                        self.body_yaw_alignment.aligned_yaw(
                            self.body_yaw
                        )
                        if self.body_yaw is not None
                        else None
                    ),
                    "body_yaw_offset_deg": finite_or_none(
                        math.degrees(
                            self.body_yaw_alignment.offset_rad
                        )
                        if self.body_yaw_alignment.ready
                        else None
                    ),
                    "body_yaw_age_ms": finite_or_none(
                        self.body_yaw_age(control_start_mono) * 1000.0
                        if self.body_yaw_age(control_start_mono)
                        is not None
                        else None
                    ),
                    "raw_lio_x": finite_or_none(
                        self.raw_lio_position[0]
                        if self.raw_lio_position is not None
                        else None
                    ),
                    "raw_lio_y": finite_or_none(
                        self.raw_lio_position[1]
                        if self.raw_lio_position is not None
                        else None
                    ),
                    "raw_lio_z": finite_or_none(
                        self.raw_lio_position[2]
                        if self.raw_lio_position is not None
                        else None
                    ),
                },
                "horizontal_frame": (
                    {
                        **self.horizontal_estimator.diagnostics(),
                        "route_anchored": self.horizontal_route_anchored,
                        "route_anchor_rotation_deg": finite_or_none(
                            math.degrees(
                                self.horizontal_anchor_rotation
                            )
                            if self.horizontal_anchor_rotation is not None
                            else None
                        ),
                    }
                    if self.use_horizontal_frame
                    else None
                ),
                "nearest": nearest,
                "target": target,
                "projection": projection,
                "control": {
                    "source": (
                        (
                            (
                                "deployed_go2_2_unified_horizontal_frame"
                                if self.use_horizontal_frame
                                else "deployed_go2_2_body_yaw_aligned"
                            )
                            if (
                                self.use_horizontal_frame
                                or self.use_body_yaw_alignment
                            )
                            else "deployed_go2_2_nearest_lookahead"
                        )
                        if motion_enabled
                        else (
                            "body_yaw_stale_hold_zero"
                            if (
                                self.use_body_yaw_alignment
                                and self.motion_release_reported
                                and motion_requested
                            )
                            else "startup_interlock_zero"
                        )
                    ),
                    "motion_enabled": bool(motion_enabled),
                    "motion_requested": bool(motion_requested),
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
