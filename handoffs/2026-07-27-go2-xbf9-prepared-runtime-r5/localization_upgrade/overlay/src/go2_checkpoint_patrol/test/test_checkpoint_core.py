import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

from go2_checkpoint_patrol.checkpoint_core import (
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


class TransformTests(unittest.TestCase):
    def test_derive_and_apply_recovers_map_pose(self):
        raw = Pose2(12.0, -4.0, math.radians(70.0))
        expected = Pose2(-3.0, 8.0, math.radians(-15.0))
        transform = derive_map_from_odom(expected, raw)
        actual = transform.apply(raw)
        self.assertAlmostEqual(actual.x, expected.x, places=9)
        self.assertAlmostEqual(actual.y, expected.y, places=9)
        self.assertAlmostEqual(actual.yaw, expected.yaw, places=9)

    def test_transform_difference_wraps_yaw(self):
        first = Transform2(0.0, 0.0, math.radians(179.0))
        second = Transform2(3.0, 4.0, math.radians(-179.0))
        translation, yaw = transform_difference(first, second)
        self.assertAlmostEqual(translation, 5.0)
        self.assertAlmostEqual(math.degrees(yaw), 2.0, places=9)


class RouteContractTests(unittest.TestCase):
    SOURCE_CSV_HASH = "1" * 64
    SOURCE_PCD_HASH = "2" * 64
    ROUTE = (
        "id,x,y,yaw,v\r\n"
        "0,0.000000,0.000000,0.000000000,0.200\r\n"
        "1,0.400000,0.000000,0.000000000,0.200\r\n"
        "2,0.800000,0.000000,0.000000000,0.200\r\n"
    )

    def _load(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.csv"
            path.write_text(content, encoding="utf-8")
            return load_route(
                str(path),
                default_checkpoint_radius_m=0.5,
                default_search_radius_m=12.0,
            )

    def _sidecar_document(self, route_bytes, checkpoints=None):
        return {
            "schema": "go2.route_checkpoints/v1",
            "source_pcd_sha256": self.SOURCE_PCD_HASH,
            "source_csv_sha256": self.SOURCE_CSV_HASH,
            "route_csv_sha256": hashlib.sha256(route_bytes).hexdigest(),
            "route_revision": 3,
            "checkpoints": (
                checkpoints
                if checkpoints is not None
                else [
                {
                    "waypoint_id": "1",
                    "waypoint_index": 1,
                    "checkpoint": {
                        "mode": "relocalize",
                        "stop_timeout_s": 60,
                        "required": True,
                    },
                }
                ]
            ),
        }

    def _load_with_sidecar(self, document):
        with tempfile.TemporaryDirectory() as directory:
            route_path = Path(directory) / "route.edited.csv"
            route_path.write_bytes(self.ROUTE.encode("utf-8"))
            sidecar_path = Path(directory) / "route.checkpoints.json"
            serialized = (
                document
                if isinstance(document, str)
                else json.dumps(document, ensure_ascii=False)
            )
            sidecar_path.write_text(serialized, encoding="utf-8")
            return load_route(
                str(route_path),
                default_checkpoint_radius_m=0.5,
                default_search_radius_m=12.0,
                checkpoint_file=str(sidecar_path),
                expected_source_csv_sha256=self.SOURCE_CSV_HASH,
                expected_source_pcd_sha256=self.SOURCE_PCD_HASH,
            )

    def test_existing_follower_csv_has_no_implicit_checkpoints(self):
        route = self._load(self.ROUTE)
        self.assertEqual(len(route), 3)
        self.assertFalse(any(point.is_checkpoint for point in route))

    def test_empty_sidecar_keeps_startup_calibration_without_mid_route_stops(self):
        document = self._sidecar_document(
            self.ROUTE.encode("utf-8"),
            checkpoints=[],
        )
        route = self._load_with_sidecar(document)
        self.assertEqual(len(route), 3)
        self.assertFalse(any(point.is_checkpoint for point in route))

    def test_hash_bound_sidecar_is_merged_without_changing_csv(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        route = self._load_with_sidecar(document)
        checkpoint = route[1]
        self.assertTrue(checkpoint.is_checkpoint)
        self.assertEqual(checkpoint.checkpoint_id, "waypoint-1")
        self.assertEqual(checkpoint.checkpoint_stop_timeout_s, 60)
        self.assertTrue(checkpoint.checkpoint_required)
        self.assertAlmostEqual(checkpoint.checkpoint_radius_m, 0.5)
        self.assertAlmostEqual(checkpoint.checkpoint_search_radius_m, 12.0)

    def test_exact_route_csv_hash_is_required(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["route_csv_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "exact route_file bytes"):
            self._load_with_sidecar(document)

    def test_source_hashes_must_match_reviewed_configuration(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["source_pcd_sha256"] = "3" * 64
        with self.assertRaisesRegex(ValueError, "source PCD hash"):
            self._load_with_sidecar(document)

    def test_waypoint_id_and_index_must_bind_same_route_row(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["checkpoints"][0]["waypoint_id"] = "2"
        with self.assertRaisesRegex(ValueError, "id/index binding"):
            self._load_with_sidecar(document)

    def test_unknown_sidecar_field_is_rejected(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["unexpected"] = True
        with self.assertRaisesRegex(ValueError, "unknown"):
            self._load_with_sidecar(document)

    def test_duplicate_checkpoint_index_is_rejected(self):
        checkpoint = {
            "waypoint_id": "1",
            "waypoint_index": 1,
            "checkpoint": {
                "mode": "relocalize",
                "stop_timeout_s": 60,
                "required": True,
            },
        }
        document = self._sidecar_document(
            self.ROUTE.encode("utf-8"),
            checkpoints=[checkpoint, dict(checkpoint)],
        )
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self._load_with_sidecar(document)

    def test_checkpoint_policy_fields_are_strict(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["checkpoints"][0]["checkpoint"]["required"] = 1
        with self.assertRaisesRegex(ValueError, "required must be boolean"):
            self._load_with_sidecar(document)

    def test_required_false_and_per_point_timeout_are_preserved(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        document["checkpoints"][0]["checkpoint"] = {
            "mode": "relocalize",
            "stop_timeout_s": 17,
            "required": False,
        }
        checkpoint = self._load_with_sidecar(document)[1]
        self.assertEqual(checkpoint.checkpoint_stop_timeout_s, 17)
        self.assertFalse(checkpoint.checkpoint_required)

    def test_duplicate_json_key_is_rejected(self):
        document = self._sidecar_document(self.ROUTE.encode("utf-8"))
        serialized = json.dumps(document).replace(
            '"route_revision": 3',
            '"route_revision": 3, "route_revision": 4',
        )
        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            self._load_with_sidecar(serialized)


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.route = [
            RoutePoint(index, str(index), float(index), 0.0, 0.0)
            for index in range(12)
        ]
        self.route[5] = RoutePoint(
            5,
            "5",
            5.0,
            0.0,
            0.0,
            checkpoint_id="cp-five",
            checkpoint_radius_m=0.4,
            checkpoint_search_radius_m=10.0,
        )

    def test_checkpoint_requires_route_progress_and_geometry(self):
        tracker = CheckpointTracker(
            self.route, search_window=8, trigger_index_slop=1
        )
        tracker.start_at(0)
        # Being geometrically near the checkpoint is not enough while progress
        # is still at the route start (protects crossings).
        self.assertIsNone(tracker.observe(Pose2(5.0, 0.0, 0.0)))
        event = None
        for index in range(1, 7):
            event = tracker.observe(Pose2(float(index), 0.0, 0.0))
            if event is not None:
                break
        self.assertIsNotNone(event)
        self.assertEqual(event.checkpoint_id, "cp-five")

    def test_passed_checkpoint_latches_as_missed(self):
        tracker = CheckpointTracker(
            self.route,
            search_window=8,
            trigger_index_slop=0,
            missed_index_margin=1,
        )
        tracker.start_at(4)
        with self.assertRaises(MissedCheckpoint):
            for index in range(6, 11):
                tracker.observe(Pose2(float(index), 2.0, 0.0))


class GateTests(unittest.TestCase):
    def _gate(self):
        return CheckpointLocalizationGate(
            enabled=True,
            settle_before_reset_sec=1.0,
            collect_after_reset_sec=1.0,
            relocalize_retry_sec=0.5,
            localization_timeout_sec=10.0,
        )

    def _reach_running(self, gate):
        gate.begin(0.0)
        self.assertIsNone(gate.take_action(0.0))
        gate.tick(1.0)
        self.assertEqual(gate.take_action(1.0), GateAction.ACTIVATE)
        gate.action_result(
            GateAction.ACTIVATE, success=True, now=1.0
        )
        self.assertEqual(gate.take_action(1.0), GateAction.RESET)
        gate.action_result(GateAction.RESET, success=True, now=1.0)
        self.assertEqual(gate.take_action(2.0), GateAction.RELOCALIZE)
        gate.action_result(
            GateAction.RELOCALIZE, success=True, now=2.0
        )
        gate.localization_evidence(
            ready=True, lost=False, now=2.1
        )
        self.assertEqual(
            gate.take_action(2.1), GateAction.CAPTURE_ALIGNMENT
        )
        gate.action_result(
            GateAction.CAPTURE_ALIGNMENT, success=True, now=2.1
        )
        self.assertEqual(gate.take_action(2.1), GateAction.DEACTIVATE)
        gate.action_result(
            GateAction.DEACTIVATE, success=True, now=2.1
        )
        self.assertEqual(gate.state, GateState.RUNNING)

    def test_default_disabled_never_authorizes_motion(self):
        gate = CheckpointLocalizationGate()
        gate.begin(0.0)
        self.assertEqual(gate.state, GateState.DISABLED)
        self.assertTrue(gate.hold_required)
        self.assertIsNone(gate.take_action(100.0))

    def test_startup_handoff_requires_full_sequence(self):
        gate = self._gate()
        self._reach_running(gate)
        self.assertFalse(gate.hold_required)

    def test_checkpoint_immediately_holds_then_repeats_sequence(self):
        gate = self._gate()
        self._reach_running(gate)
        gate.checkpoint_reached(
            index=40,
            checkpoint_id="wall-40",
            stop_timeout_sec=60,
            required=True,
            now=3.0,
        )
        self.assertTrue(gate.hold_required)
        self.assertEqual(gate.state, GateState.SETTLING)
        self.assertEqual(gate.phase.checkpoint_id, "wall-40")

    def test_per_checkpoint_timeout_overrides_default_and_always_holds(self):
        for required in (True, False):
            gate = self._gate()
            self._reach_running(gate)
            gate.checkpoint_reached(
                index=40,
                checkpoint_id="wall-40",
                stop_timeout_sec=5,
                required=required,
                now=3.0,
            )
            gate.tick(8.0)
            self.assertEqual(gate.state, GateState.FAULT_HOLD)
            self.assertTrue(gate.hold_required)
            self.assertIn("automatic bypass is unsupported", gate.fault_reason)
            self.assertIn(
                f"required={str(required).lower()}", gate.fault_reason
            )

    def test_lost_latches_and_status_recovery_cannot_auto_resume(self):
        gate = self._gate()
        gate.begin(0.0)
        gate.tick(1.0)
        gate.take_action(1.0)
        gate.action_result(
            GateAction.ACTIVATE, success=True, now=1.0
        )
        gate.take_action(1.0)
        gate.action_result(GateAction.RESET, success=True, now=1.0)
        gate.localization_evidence(
            ready=False, lost=True, now=1.1, reason="LOST test"
        )
        self.assertEqual(gate.state, GateState.FAULT_HOLD)
        gate.localization_evidence(
            ready=True, lost=False, now=1.2
        )
        self.assertEqual(gate.state, GateState.FAULT_HOLD)
        self.assertTrue(gate.retry_from_fault(2.0))
        self.assertEqual(gate.state, GateState.SETTLING)

    def test_timeout_latches_until_manual_retry(self):
        gate = self._gate()
        gate.begin(0.0)
        gate.tick(11.0)
        self.assertEqual(gate.state, GateState.FAULT_HOLD)
        self.assertIsNone(gate.take_action(12.0))
        self.assertTrue(gate.retry_from_fault(13.0))

    def test_service_discovery_can_defer_without_authorizing_motion(self):
        gate = self._gate()
        gate.begin(0.0)
        gate.tick(1.0)
        action = gate.take_action(1.0)
        self.assertEqual(action, GateAction.ACTIVATE)
        gate.defer_action(action, now=1.0, retry_after_sec=0.5)
        self.assertTrue(gate.hold_required)
        self.assertIsNone(gate.take_action(1.4))
        self.assertEqual(gate.take_action(1.5), GateAction.ACTIVATE)


class EvidenceTests(unittest.TestCase):
    def _evidence(self, **changes):
        values = {
            "state": 2,
            "map_valid": True,
            "pose_valid": True,
            "safe_to_move": True,
            "startup_precision_verified": True,
            "global_confirmation_pending": False,
            "map_id": "reviewed-map-r1",
            "map_hash": "abc123",
            "status_receive_age_sec": 0.05,
            "corrected_odom_receive_age_sec": 0.05,
        }
        values.update(changes)
        return LocalizationEvidence(**values)

    def test_ready_requires_exact_map_identity_and_fresh_pose(self):
        ready, reason = evidence_is_ready(
            self._evidence(),
            expected_map_id="reviewed-map-r1",
            expected_map_hash="abc123",
            maximum_status_age_sec=0.3,
            maximum_corrected_odom_age_sec=0.25,
        )
        self.assertTrue(ready, reason)
        ready, reason = evidence_is_ready(
            self._evidence(map_hash="different"),
            expected_map_id="reviewed-map-r1",
            expected_map_hash="abc123",
            maximum_status_age_sec=0.3,
            maximum_corrected_odom_age_sec=0.25,
        )
        self.assertFalse(ready)
        self.assertIn("hash", reason)

    def test_anchor_checks_position_and_heading(self):
        anchor = RoutePoint(0, "0", 1.0, 2.0, 0.0)
        valid, _ = validate_anchor_pose(
            Pose2(1.2, 2.1, math.radians(4.0)),
            anchor,
            maximum_distance_m=0.5,
            maximum_yaw_error_rad=math.radians(5.0),
        )
        self.assertTrue(valid)
        valid, reason = validate_anchor_pose(
            Pose2(1.2, 2.1, math.radians(6.0)),
            anchor,
            maximum_distance_m=0.5,
            maximum_yaw_error_rad=math.radians(5.0),
        )
        self.assertFalse(valid)
        self.assertIn("yaw", reason)


if __name__ == "__main__":
    unittest.main()
