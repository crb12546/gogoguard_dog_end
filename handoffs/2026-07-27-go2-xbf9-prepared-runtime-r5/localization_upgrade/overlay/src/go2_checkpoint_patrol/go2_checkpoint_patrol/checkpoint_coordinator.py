#!/usr/bin/env python3
"""Stop-and-localize adapter around the unchanged CSV waypoint follower.

The node never talks to Unitree's sport API.  When explicitly enabled and
correctly wired, it:

* publishes a frozen-SE2 view of FAST-LIO odometry for the existing follower;
* passes the follower's Twist through unchanged only in RUNNING;
* forces zero Twist during startup/checkpoint localization or any fault;
* activates the expensive map localizer only while the robot is stopped.

Defaults are isolated topics plus ``integration_enabled=false`` so merely
installing or starting this node cannot take over the existing patrol chain.
"""

from __future__ import annotations

import copy
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from go2_nav_interfaces.msg import LocalizationStatus, RouteStatus
from go2_nav_interfaces.srv import GlobalRelocalize, ResetLocalization

from .checkpoint_core import (
    CheckpointLocalizationGate,
    CheckpointTracker,
    GateAction,
    GateState,
    LocalizationEvidence,
    MissedCheckpoint,
    Pose2,
    RoutePoint,
    Transform2,
    derive_map_from_odom,
    evidence_is_ready,
    load_route,
    transform_difference,
    validate_anchor_pose,
)


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _set_yaw(quaternion, yaw: float) -> None:
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def _pose2_from_odometry(message: Odometry) -> Pose2:
    pose = message.pose.pose
    return Pose2(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_yaw_from_quaternion(pose.orientation),
    )


def _control_graph_mismatch_reason(
    *,
    gated_cmd_topic: str,
    gated_publishers: int,
    actuator_command_topic: str,
    actuator_publishers: int,
    actuator_subscribers: int,
    legacy_cmd_topic: str,
    legacy_publishers: int,
) -> Optional[str]:
    """Describe the first mismatch in the one-to-one motion command chain."""

    if gated_publishers != 1:
        return (
            f"graph guard: {gated_cmd_topic} has {gated_publishers} "
            "publishers, expected exactly 1"
        )
    if actuator_publishers != 1:
        return (
            f"graph guard: {actuator_command_topic} has "
            f"{actuator_publishers} publishers, expected exactly 1"
        )
    if actuator_subscribers < 1:
        return (
            f"graph guard: {actuator_command_topic} has "
            f"{actuator_subscribers} subscribers, expected at least 1"
        )
    if legacy_publishers != 0:
        return (
            f"graph guard: legacy {legacy_cmd_topic} has "
            f"{legacy_publishers} publishers, expected exactly 0"
        )
    return None


@dataclass
class _RawSample:
    stamp_ns: int
    received_mono: float
    pose: Pose2


@dataclass
class _AlignmentCandidate:
    transform: Transform2
    corrected_pose: Pose2
    corrected_received_mono: float
    raw_received_mono: float
    stamp_delta_sec: float


class CheckpointLocalizationCoordinator(Node):
    """ROS glue for the pure checkpoint gate and frozen transform."""

    def __init__(self) -> None:
        super().__init__("checkpoint_localization_coordinator")

        self.declare_parameter("integration_enabled", False)
        self.declare_parameter("route_file", "")
        self.declare_parameter("checkpoint_file", "")
        self.declare_parameter("expected_map_id", "")
        self.declare_parameter("expected_map_hash", "")
        self.declare_parameter("expected_source_csv_sha256", "")
        self.declare_parameter("expected_source_pcd_sha256", "")
        self.declare_parameter("route_frame", "map")

        self.declare_parameter("raw_odom_topic", "/Odometry")
        self.declare_parameter(
            "aligned_odom_topic", "/checkpoint_localization/aligned_odometry"
        )
        self.declare_parameter(
            "follower_cmd_topic", "/checkpoint_localization/follower_cmd"
        )
        self.declare_parameter(
            "gated_cmd_topic", "/checkpoint_localization/gated_cmd"
        )
        self.declare_parameter("graph_guard_enabled", False)
        self.declare_parameter("legacy_cmd_topic", "/patrol_cmd")
        self.declare_parameter("actuator_command_topic", "/cmd_vel")
        self.declare_parameter("localization_status_topic", "/localization/status")
        self.declare_parameter(
            "localization_odometry_topic", "/localization/odometry"
        )
        self.declare_parameter(
            "route_status_topic", "/checkpoint_localization/route_status"
        )
        self.declare_parameter(
            "activation_service", "/localization/set_active"
        )
        self.declare_parameter("reset_service", "/localization/reset")
        self.declare_parameter(
            "global_relocalize_service", "/localization/global_relocalize"
        )
        self.declare_parameter(
            "operator_retry_service",
            "/checkpoint_localization/retry_after_fault",
        )

        self.declare_parameter("control_rate_hz", 40.0)
        self.declare_parameter("follower_cmd_timeout_sec", 0.30)
        self.declare_parameter("raw_odom_timeout_sec", 0.50)
        self.declare_parameter("running_input_grace_sec", 1.0)
        self.declare_parameter("settle_before_reset_sec", 2.0)
        self.declare_parameter("collect_after_reset_sec", 2.0)
        self.declare_parameter("relocalize_retry_sec", 0.7)
        self.declare_parameter("service_discovery_retry_sec", 0.5)
        self.declare_parameter("localization_timeout_sec", 90.0)
        self.declare_parameter("status_maximum_age_sec", 0.30)
        self.declare_parameter("corrected_odom_maximum_age_sec", 0.25)
        self.declare_parameter("alignment_sync_slop_sec", 0.08)

        self.declare_parameter("checkpoint_default_radius_m", 0.50)
        self.declare_parameter("checkpoint_default_search_radius_m", 15.0)
        self.declare_parameter("checkpoint_search_window", 8)
        self.declare_parameter("checkpoint_trigger_index_slop", 3)
        self.declare_parameter("checkpoint_missed_index_margin", 5)
        self.declare_parameter("startup_anchor_index", 0)
        self.declare_parameter("startup_maximum_anchor_distance_m", 2.0)
        self.declare_parameter(
            "startup_maximum_anchor_yaw_error_rad", math.radians(35.0)
        )
        self.declare_parameter("checkpoint_maximum_anchor_distance_m", 1.0)
        self.declare_parameter(
            "checkpoint_maximum_anchor_yaw_error_rad", math.radians(20.0)
        )
        self.declare_parameter("maximum_transform_jump_m", 5.0)
        self.declare_parameter(
            "maximum_transform_jump_yaw_rad", math.radians(30.0)
        )

        self.enabled = bool(self.get_parameter("integration_enabled").value)
        self.route_file = str(self.get_parameter("route_file").value)
        self.checkpoint_file = str(
            self.get_parameter("checkpoint_file").value
        )
        self.expected_map_id = str(self.get_parameter("expected_map_id").value)
        self.expected_map_hash = str(
            self.get_parameter("expected_map_hash").value
        )
        self.expected_source_csv_sha256 = str(
            self.get_parameter("expected_source_csv_sha256").value
        )
        self.expected_source_pcd_sha256 = str(
            self.get_parameter("expected_source_pcd_sha256").value
        )
        self.route_frame = str(self.get_parameter("route_frame").value)
        self.gated_cmd_topic = str(
            self.get_parameter("gated_cmd_topic").value
        )
        self.graph_guard_enabled = bool(
            self.get_parameter("graph_guard_enabled").value
        )
        self.legacy_cmd_topic = str(
            self.get_parameter("legacy_cmd_topic").value
        )
        self.actuator_command_topic = str(
            self.get_parameter("actuator_command_topic").value
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_timeout_sec = float(
            self.get_parameter("follower_cmd_timeout_sec").value
        )
        self.raw_odom_timeout_sec = float(
            self.get_parameter("raw_odom_timeout_sec").value
        )
        self.running_input_grace_sec = float(
            self.get_parameter("running_input_grace_sec").value
        )
        self.status_maximum_age_sec = float(
            self.get_parameter("status_maximum_age_sec").value
        )
        self.corrected_odom_maximum_age_sec = float(
            self.get_parameter("corrected_odom_maximum_age_sec").value
        )
        self.alignment_sync_slop_sec = float(
            self.get_parameter("alignment_sync_slop_sec").value
        )
        self.service_discovery_retry_sec = float(
            self.get_parameter("service_discovery_retry_sec").value
        )
        self.startup_anchor_index = int(
            self.get_parameter("startup_anchor_index").value
        )
        self.startup_maximum_anchor_distance_m = float(
            self.get_parameter("startup_maximum_anchor_distance_m").value
        )
        self.startup_maximum_anchor_yaw_error_rad = float(
            self.get_parameter("startup_maximum_anchor_yaw_error_rad").value
        )
        self.checkpoint_maximum_anchor_distance_m = float(
            self.get_parameter("checkpoint_maximum_anchor_distance_m").value
        )
        self.checkpoint_maximum_anchor_yaw_error_rad = float(
            self.get_parameter("checkpoint_maximum_anchor_yaw_error_rad").value
        )
        self.maximum_transform_jump_m = float(
            self.get_parameter("maximum_transform_jump_m").value
        )
        self.maximum_transform_jump_yaw_rad = float(
            self.get_parameter("maximum_transform_jump_yaw_rad").value
        )

        if (
            self.control_rate_hz <= 0.0
            or self.cmd_timeout_sec <= 0.0
            or self.raw_odom_timeout_sec <= 0.0
            or self.running_input_grace_sec <= 0.0
            or self.service_discovery_retry_sec <= 0.0
        ):
            raise RuntimeError(
                "control rate and all timeout/retry values must be positive"
            )
        if self.enabled:
            if not self.route_file:
                raise RuntimeError("enabled integration requires route_file")
            if not self.expected_map_id or not self.expected_map_hash:
                raise RuntimeError(
                    "enabled integration requires exact expected_map_id and "
                    "expected_map_hash"
                )
            if self.checkpoint_file and (
                not self.expected_source_csv_sha256
                or not self.expected_source_pcd_sha256
            ):
                raise RuntimeError(
                    "checkpoint_file requires exact expected source CSV and "
                    "source PCD SHA-256"
                )

        self.route = (
            load_route(
                self.route_file,
                default_checkpoint_radius_m=float(
                    self.get_parameter("checkpoint_default_radius_m").value
                ),
                default_search_radius_m=float(
                    self.get_parameter(
                        "checkpoint_default_search_radius_m"
                    ).value
                ),
                checkpoint_file=self.checkpoint_file,
                expected_source_csv_sha256=(
                    self.expected_source_csv_sha256
                ),
                expected_source_pcd_sha256=(
                    self.expected_source_pcd_sha256
                ),
            )
            if self.enabled
            else []
        )
        if self.enabled and not (
            0 <= self.startup_anchor_index < len(self.route)
        ):
            raise RuntimeError("startup_anchor_index is outside the route")
        self.tracker = (
            CheckpointTracker(
                self.route,
                search_window=int(
                    self.get_parameter("checkpoint_search_window").value
                ),
                trigger_index_slop=int(
                    self.get_parameter(
                        "checkpoint_trigger_index_slop"
                    ).value
                ),
                missed_index_margin=int(
                    self.get_parameter(
                        "checkpoint_missed_index_margin"
                    ).value
                ),
            )
            if self.enabled
            else None
        )
        self.gate = CheckpointLocalizationGate(
            enabled=self.enabled,
            settle_before_reset_sec=float(
                self.get_parameter("settle_before_reset_sec").value
            ),
            collect_after_reset_sec=float(
                self.get_parameter("collect_after_reset_sec").value
            ),
            relocalize_retry_sec=float(
                self.get_parameter("relocalize_retry_sec").value
            ),
            localization_timeout_sec=float(
                self.get_parameter("localization_timeout_sec").value
            ),
        )

        self.alignment: Optional[Transform2] = None
        self._candidate: Optional[_AlignmentCandidate] = None
        self._raw_samples: Deque[_RawSample] = deque(maxlen=500)
        self._last_raw: Optional[_RawSample] = None
        self._last_follower_cmd: Optional[Twist] = None
        self._last_follower_cmd_mono = 0.0
        self._last_status: Optional[LocalizationStatus] = None
        self._last_status_mono = 0.0
        self._evidence_epoch_mono = float("inf")
        self._pending_future = None
        self._pending_action: Optional[GateAction] = None
        self._cleanup_future = None
        self._last_gate_state = self.gate.state
        self._running_since_mono = float("inf")
        self._last_status_log_mono = 0.0
        self._graph_guard_armed = False

        self.aligned_odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("aligned_odom_topic").value),
            10,
        )
        self.gated_cmd_pub = self.create_publisher(
            Twist,
            self.gated_cmd_topic,
            10,
        )
        self.route_status_pub = self.create_publisher(
            RouteStatus,
            str(self.get_parameter("route_status_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("raw_odom_topic").value),
            self._raw_odom_callback,
            50,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("follower_cmd_topic").value),
            self._follower_cmd_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("localization_odometry_topic").value),
            self._corrected_odom_callback,
            20,
        )
        self.create_subscription(
            LocalizationStatus,
            str(self.get_parameter("localization_status_topic").value),
            self._localization_status_callback,
            10,
        )

        self.activation_client = self.create_client(
            SetBool, str(self.get_parameter("activation_service").value)
        )
        self.reset_client = self.create_client(
            ResetLocalization, str(self.get_parameter("reset_service").value)
        )
        self.global_client = self.create_client(
            GlobalRelocalize,
            str(self.get_parameter("global_relocalize_service").value),
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("operator_retry_service").value),
            self._retry_service,
        )
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz, self._timer_callback
        )

        self.gate.begin(time.monotonic())
        if self.enabled:
            checkpoint_count = sum(point.is_checkpoint for point in self.route)
            self.get_logger().warn(
                "CHECKPOINT INTEGRATION enabled: follower commands remain blocked "
                f"until startup localization; route points={len(self.route)}, "
                f"hash-bound checkpoints={checkpoint_count}"
            )
        else:
            self.get_logger().warn(
                "checkpoint integration is DISABLED; isolated gated output stays "
                "at zero and cannot authorize patrol"
            )

    def _clear_evidence(self) -> None:
        self._candidate = None
        self._last_status = None
        self._last_status_mono = 0.0
        self._evidence_epoch_mono = float("inf")

    def _raw_odom_callback(self, message: Odometry) -> None:
        now = time.monotonic()
        sample = _RawSample(
            stamp_ns=_stamp_ns(message),
            received_mono=now,
            pose=_pose2_from_odometry(message),
        )
        self._raw_samples.append(sample)
        self._last_raw = sample
        if self.alignment is None:
            return
        aligned_pose = self.alignment.apply(sample.pose)
        aligned = copy.deepcopy(message)
        aligned.header.frame_id = self.route_frame
        aligned.pose.pose.position.x = aligned_pose.x
        aligned.pose.pose.position.y = aligned_pose.y
        _set_yaw(aligned.pose.pose.orientation, aligned_pose.yaw)
        self.aligned_odom_pub.publish(aligned)

        if self.gate.state != GateState.RUNNING or self.tracker is None:
            return
        try:
            checkpoint = self.tracker.observe(aligned_pose)
        except MissedCheckpoint as error:
            self._latch_fault(str(error))
            return
        if checkpoint is not None:
            self.gate.checkpoint_reached(
                index=checkpoint.index,
                checkpoint_id=checkpoint.checkpoint_id or str(checkpoint.index),
                stop_timeout_sec=checkpoint.checkpoint_stop_timeout_s,
                required=checkpoint.checkpoint_required,
                now=now,
            )
            self._clear_evidence()
            self._publish_stop()
            self.get_logger().warn(
                f"checkpoint reached: {checkpoint.checkpoint_id} "
                f"(route index {checkpoint.index}, "
                f"timeout={checkpoint.checkpoint_stop_timeout_s}s, "
                f"required={str(checkpoint.checkpoint_required).lower()}); "
                "follower command paused"
            )

    def _corrected_odom_callback(self, message: Odometry) -> None:
        if not self.enabled or self.gate.state not in (
            GateState.LOCALIZING,
            GateState.CAPTURING,
        ):
            return
        corrected_stamp = _stamp_ns(message)
        if not self._raw_samples:
            return
        raw = min(
            self._raw_samples,
            key=lambda sample: abs(sample.stamp_ns - corrected_stamp),
        )
        stamp_delta = abs(raw.stamp_ns - corrected_stamp) * 1.0e-9
        if stamp_delta > self.alignment_sync_slop_sec:
            return
        corrected_pose = _pose2_from_odometry(message)
        now = time.monotonic()
        self._candidate = _AlignmentCandidate(
            transform=derive_map_from_odom(corrected_pose, raw.pose),
            corrected_pose=corrected_pose,
            corrected_received_mono=now,
            raw_received_mono=raw.received_mono,
            stamp_delta_sec=stamp_delta,
        )

    def _localization_status_callback(
        self, message: LocalizationStatus
    ) -> None:
        self._last_status = message
        self._last_status_mono = time.monotonic()

    def _follower_cmd_callback(self, message: Twist) -> None:
        now = time.monotonic()
        self._last_follower_cmd = copy.deepcopy(message)
        self._last_follower_cmd_mono = now
        raw_fresh = (
            self._last_raw is not None
            and now - self._last_raw.received_mono <= self.raw_odom_timeout_sec
        )
        if (
            self.enabled
            and self.gate.state == GateState.RUNNING
            and raw_fresh
        ):
            # Normal route behavior is the original follower's command without
            # gain, rate, or trajectory changes.  The timer only adds timeout.
            self.gated_cmd_pub.publish(message)
        else:
            self._publish_stop()

    def _publish_stop(self) -> None:
        self.gated_cmd_pub.publish(Twist())

    def _graph_guard_reason(self) -> Optional[str]:
        """Require one safe publisher and at least one command consumer."""
        if not self.enabled or not self.graph_guard_enabled:
            return None
        gated_publishers = self.count_publishers(self.gated_cmd_topic)
        actuator_publishers = self.count_publishers(
            self.actuator_command_topic
        )
        actuator_subscribers = self.count_subscribers(
            self.actuator_command_topic
        )
        legacy_publishers = self.count_publishers(self.legacy_cmd_topic)
        mismatch = _control_graph_mismatch_reason(
            gated_cmd_topic=self.gated_cmd_topic,
            gated_publishers=gated_publishers,
            actuator_command_topic=self.actuator_command_topic,
            actuator_publishers=actuator_publishers,
            actuator_subscribers=actuator_subscribers,
            legacy_cmd_topic=self.legacy_cmd_topic,
            legacy_publishers=legacy_publishers,
        )
        if not self._graph_guard_armed:
            # The coordinator starts before the safe command node and UDP sender.
            # Missing downstream endpoints are tolerated only while localization
            # still holds zero. Duplicate publishers and legacy publishers are
            # unsafe immediately; observer subscribers (for example rosbag) are
            # harmless, so only a zero subscriber count is rejected.
            unsafe_while_starting = (
                gated_publishers != 1
                or actuator_publishers > 1
                or legacy_publishers != 0
            )
            if unsafe_while_starting:
                return mismatch
            if mismatch is None:
                self._graph_guard_armed = True
                self.get_logger().info(
                    "graph guard armed: "
                    f"{self.gated_cmd_topic} publisher=1, "
                    f"{self.actuator_command_topic} publisher=1/"
                    f"subscribers={actuator_subscribers}, "
                    f"{self.legacy_cmd_topic} publisher=0"
                )
                return None
            if self.gate.state == GateState.RUNNING:
                return (
                    "graph guard: motion command chain was not ready before "
                    f"RUNNING; {mismatch}"
                )
            return None
        return mismatch

    def _retry_service(self, request, response):
        del request
        now = time.monotonic()
        if self._cleanup_future is not None:
            response.success = False
            response.message = (
                "localizer deactivation is still pending; retry again after it finishes"
            )
            return response
        if not self.gate.retry_from_fault(now):
            response.success = False
            response.message = (
                "retry is accepted only from latched FAULT_HOLD"
            )
            return response
        self._clear_evidence()
        response.success = True
        response.message = (
            "fault latch cleared for one calibration retry; motion remains stopped"
        )
        self.get_logger().warn(response.message)
        return response

    def _phase_anchor(self) -> RoutePoint:
        phase = self.gate.phase
        if phase is None or phase.kind == "startup":
            return self.route[self.startup_anchor_index]
        if phase.checkpoint_index is None:
            raise RuntimeError("checkpoint phase has no route index")
        return self.route[phase.checkpoint_index]

    def _make_global_request(self) -> GlobalRelocalize.Request:
        anchor = self._phase_anchor()
        request = GlobalRelocalize.Request()
        request.use_initial_guess = True
        request.initial_guess = PoseWithCovarianceStamped()
        request.initial_guess.header.frame_id = self.route_frame
        request.initial_guess.pose.pose.position.x = float(anchor.x)
        request.initial_guess.pose.pose.position.y = float(anchor.y)
        _set_yaw(
            request.initial_guess.pose.pose.orientation,
            float(anchor.yaw),
        )
        request.search_radius = float(
            anchor.checkpoint_search_radius_m
            if anchor.is_checkpoint
            else self.get_parameter(
                "checkpoint_default_search_radius_m"
            ).value
        )
        return request

    def _dispatch_action(self, action: GateAction, now: float) -> None:
        if action == GateAction.CAPTURE_ALIGNMENT:
            success, reason = self._capture_alignment(now)
            self.gate.action_result(
                action, success=success, now=now, reason=reason
            )
            if not success:
                self._latch_fault(reason, gate_already_failed=True)
            return

        client = None
        request = None
        if action == GateAction.ACTIVATE:
            client = self.activation_client
            request = SetBool.Request()
            request.data = True
        elif action == GateAction.RESET:
            client = self.reset_client
            request = ResetLocalization.Request()
            request.clear_map = False
        elif action == GateAction.RELOCALIZE:
            client = self.global_client
            request = self._make_global_request()
        elif action == GateAction.DEACTIVATE:
            client = self.activation_client
            request = SetBool.Request()
            request.data = False
        else:
            raise RuntimeError(f"unsupported gate action: {action}")

        if not client.service_is_ready():
            self.gate.defer_action(
                action,
                now=now,
                retry_after_sec=self.service_discovery_retry_sec,
            )
            return
        self._pending_action = action
        self._pending_future = client.call_async(request)

    def _poll_future(self, now: float) -> None:
        if self._pending_future is None or not self._pending_future.done():
            return
        action = self._pending_action
        future = self._pending_future
        self._pending_action = None
        self._pending_future = None
        if self.gate.state == GateState.FAULT_HOLD:
            self.get_logger().warn(
                "discarded a late localization service result after FAULT_HOLD"
            )
            return
        if action is None:
            self._latch_fault("service future completed without gate action")
            return
        try:
            result = future.result()
            if action == GateAction.RELOCALIZE:
                success = bool(result.accepted)
                reason = str(result.message)
            else:
                success = bool(result.success)
                reason = str(result.message)
        except Exception as error:  # rclpy service transport exception
            success = False
            reason = f"{action.value} service failed: {error}"

        self.gate.action_result(
            action, success=success, now=now, reason=reason
        )
        if action == GateAction.DEACTIVATE and self.gate.state == GateState.RUNNING:
            self._running_since_mono = now
        if action == GateAction.RESET and success:
            self._candidate = None
            self._last_status = None
            self._last_status_mono = 0.0
            self._evidence_epoch_mono = now
        if action == GateAction.RELOCALIZE and not success:
            self.get_logger().warn(
                f"global relocalization not accepted yet: {reason}"
            )
        if self.gate.state == GateState.FAULT_HOLD:
            self._latch_fault(
                self.gate.fault_reason, gate_already_failed=True
            )

    def _evaluate_evidence(self, now: float) -> None:
        if (
            self.gate.state != GateState.LOCALIZING
            or self._last_status is None
            or self._last_status_mono < self._evidence_epoch_mono
        ):
            return
        corrected_age = (
            now - self._candidate.corrected_received_mono
            if self._candidate is not None
            and self._candidate.corrected_received_mono
            >= self._evidence_epoch_mono
            else float("inf")
        )
        status = self._last_status
        evidence = LocalizationEvidence(
            state=int(status.state),
            map_valid=bool(status.map_valid),
            pose_valid=bool(status.pose_valid),
            safe_to_move=bool(status.safe_to_move),
            startup_precision_verified=bool(
                status.startup_precision_verified
            ),
            global_confirmation_pending=bool(
                status.global_confirmation_pending
            ),
            map_id=str(status.map_id),
            map_hash=str(status.map_hash),
            status_receive_age_sec=now - self._last_status_mono,
            corrected_odom_receive_age_sec=corrected_age,
        )
        ready, reason = evidence_is_ready(
            evidence,
            expected_map_id=self.expected_map_id,
            expected_map_hash=self.expected_map_hash,
            maximum_status_age_sec=self.status_maximum_age_sec,
            maximum_corrected_odom_age_sec=(
                self.corrected_odom_maximum_age_sec
            ),
        )
        lost = int(status.state) == int(LocalizationStatus.STATE_LOST)
        self.gate.localization_evidence(
            ready=ready,
            lost=lost,
            now=now,
            reason=str(status.reason or reason),
        )
        if lost:
            self._latch_fault(
                str(status.reason or reason), gate_already_failed=True
            )

    def _capture_alignment(self, now: float) -> Tuple[bool, str]:
        candidate = self._candidate
        if candidate is None:
            return False, "no synchronized corrected odometry to capture"
        if candidate.corrected_received_mono < self._evidence_epoch_mono:
            return False, "corrected odometry predates localization reset"
        if now - candidate.corrected_received_mono > (
            self.corrected_odom_maximum_age_sec
        ):
            return False, "corrected odometry became stale before capture"

        anchor = self._phase_anchor()
        phase = self.gate.phase
        startup = phase is None or phase.kind == "startup"
        valid, reason = validate_anchor_pose(
            candidate.corrected_pose,
            anchor,
            maximum_distance_m=(
                self.startup_maximum_anchor_distance_m
                if startup
                else self.checkpoint_maximum_anchor_distance_m
            ),
            maximum_yaw_error_rad=(
                self.startup_maximum_anchor_yaw_error_rad
                if startup
                else self.checkpoint_maximum_anchor_yaw_error_rad
            ),
        )
        if not valid:
            return False, reason
        if self.alignment is not None:
            translation_jump, yaw_jump = transform_difference(
                self.alignment, candidate.transform
            )
            if translation_jump > self.maximum_transform_jump_m:
                return (
                    False,
                    f"map<-odom translation jump {translation_jump:.3f} m "
                    f"exceeds {self.maximum_transform_jump_m:.3f} m",
                )
            if yaw_jump > self.maximum_transform_jump_yaw_rad:
                return (
                    False,
                    f"map<-odom yaw jump {math.degrees(yaw_jump):.3f} deg "
                    f"exceeds "
                    f"{math.degrees(self.maximum_transform_jump_yaw_rad):.3f} deg",
                )

        # Python object assignment is atomic under the GIL.  Every subsequent
        # raw odometry message observes either the complete old mapping or the
        # complete new mapping; no partially-updated tx/ty/yaw is published.
        self.alignment = candidate.transform
        # A command computed from the old mapping must never leak through after
        # handoff.  Wait for the original follower to consume a subsequent
        # aligned odometry sample and publish a fresh command.
        self._last_follower_cmd = None
        self._last_follower_cmd_mono = 0.0
        if startup and self.tracker is not None:
            nearest, _ = self.tracker.nearest_global(
                candidate.corrected_pose
            )
            self.tracker.start_at(nearest)
        self.get_logger().info(
            f"captured frozen map<-odom: tx={candidate.transform.tx:.3f}, "
            f"ty={candidate.transform.ty:.3f}, "
            f"yaw={math.degrees(candidate.transform.yaw):.4f} deg, "
            f"stamp_delta={candidate.stamp_delta_sec * 1000.0:.2f} ms"
        )
        return True, ""

    def _best_effort_deactivate(self) -> None:
        if self._cleanup_future is not None:
            return
        if not self.activation_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        self._cleanup_future = self.activation_client.call_async(request)

    def _poll_cleanup_future(self) -> None:
        if self._cleanup_future is None or not self._cleanup_future.done():
            return
        future = self._cleanup_future
        self._cleanup_future = None
        try:
            response = future.result()
            if not bool(response.success):
                self.get_logger().error(
                    "FAULT_HOLD cleanup could not deactivate localizer: "
                    f"{response.message}"
                )
        except Exception as error:
            self.get_logger().error(
                f"FAULT_HOLD cleanup service failed: {error}"
            )

    def _latch_fault(
        self, reason: str, *, gate_already_failed: bool = False
    ) -> None:
        if not gate_already_failed:
            self.gate.fail(reason)
        self._publish_stop()
        self._best_effort_deactivate()
        self.get_logger().error(
            f"FAULT_HOLD (manual retry required): {reason}"
        )
        # Publish the latched reason in the same callback/timer cycle that
        # forces zero; operators must not wait for the one-second status timer
        # to learn why the command graph was rejected.
        self._publish_route_status(time.monotonic())

    def _publish_route_status(self, now: float) -> None:
        status = RouteStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mission_id = "xbf-checkpoint-localization-r2"
        status.route_id = self.route_file
        status.waypoint_count = len(self.route)
        status.nearest_index = (
            int(self.tracker.progress_index) if self.tracker else 0
        )
        status.target_index = status.nearest_index
        status.localization_ready = (
            self.gate.state == GateState.RUNNING and self.alignment is not None
        )
        state_map = {
            GateState.DISABLED: RouteStatus.IDLE,
            GateState.ACTIVATING: RouteStatus.WAITING_LOCALIZATION,
            GateState.SETTLING: RouteStatus.WAITING_LOCALIZATION,
            GateState.RESETTING: RouteStatus.WAITING_LOCALIZATION,
            GateState.LOCALIZING: RouteStatus.WAITING_LOCALIZATION,
            GateState.CAPTURING: RouteStatus.WAITING_LOCALIZATION,
            GateState.DEACTIVATING: RouteStatus.WAITING_LOCALIZATION,
            GateState.RUNNING: RouteStatus.RUNNING,
            GateState.FAULT_HOLD: RouteStatus.FAULT,
        }
        status.state = state_map[self.gate.state]
        status.reason = (
            self.gate.fault_reason
            if self.gate.state == GateState.FAULT_HOLD
            else self.gate.state.value
        )
        if self._last_raw is not None and self.alignment is not None:
            current = self.alignment.apply(self._last_raw.pose)
            nearest = self.route[status.nearest_index]
            status.cross_track_error = float(
                math.hypot(current.x - nearest.x, current.y - nearest.y)
            )
            status.target_distance = status.cross_track_error
        self.route_status_pub.publish(status)

    def _timer_callback(self) -> None:
        now = time.monotonic()
        self._poll_cleanup_future()
        self._poll_future(now)
        self._evaluate_evidence(now)
        self.gate.tick(now)
        graph_guard_reason = self._graph_guard_reason()
        if (
            graph_guard_reason is not None
            and self.gate.state != GateState.FAULT_HOLD
        ):
            self._latch_fault(graph_guard_reason)
        if self.gate.state == GateState.FAULT_HOLD:
            self._publish_stop()
            self._best_effort_deactivate()
        elif self.gate.state == GateState.RUNNING:
            in_grace = (
                now - self._running_since_mono
                <= self.running_input_grace_sec
            )
            raw_stale = (
                self._last_raw is None
                or now - self._last_raw.received_mono
                > self.raw_odom_timeout_sec
            )
            cmd_stale = (
                self._last_follower_cmd is None
                or now - self._last_follower_cmd_mono > self.cmd_timeout_sec
            )
            if in_grace and (raw_stale or cmd_stale):
                self._publish_stop()
            elif raw_stale:
                self._latch_fault("raw /Odometry receive timeout")
            elif cmd_stale:
                self._latch_fault("waypoint follower command timeout")
            else:
                self.gated_cmd_pub.publish(self._last_follower_cmd)
        else:
            self._publish_stop()

        if self._pending_future is None:
            action = self.gate.take_action(now)
            if action is not None:
                self._dispatch_action(action, now)

        completed_phase = self.gate.last_completed_phase
        if completed_phase is not None:
            if (
                completed_phase.kind == "checkpoint"
                and self.tracker is not None
                and completed_phase.checkpoint_index is not None
            ):
                self.tracker.complete(completed_phase.checkpoint_index)
            self.gate.last_completed_phase = None

        if self.gate.state != self._last_gate_state:
            self.get_logger().info(
                f"checkpoint gate: {self._last_gate_state.value} -> "
                f"{self.gate.state.value}"
            )
            self._last_gate_state = self.gate.state
        if now - self._last_status_log_mono >= 1.0:
            self._last_status_log_mono = now
            self._publish_route_status(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckpointLocalizationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("checkpoint coordinator stopped by operator")
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
