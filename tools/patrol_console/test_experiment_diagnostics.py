#!/usr/bin/env python3
import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load_script(name):
    path = ROOT / "orin_go2_fastlio_ws/scripts" / name
    spec = importlib.util.spec_from_file_location(
        name.replace(".py", "") + "_tested",
        str(path),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_route(path, points):
    with Path(path).open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["id", "x", "y", "yaw", "v"])
        for index, point in enumerate(points):
            writer.writerow(
                [
                    index,
                    "%.6f" % point[0],
                    "%.6f" % point[1],
                    "%.6f" % point[2],
                    "0.200",
                ]
            )


def telemetry_record(kind, timestamp, data):
    return {
        "kind": kind,
        "sequence": 1,
        "wall_time": timestamp,
        "monotonic_s": timestamp,
        "elapsed_s": timestamp - 1000.0,
        "source_stamp": timestamp if kind == "odom" else None,
        "data": data,
    }


def odom_data(x, y, yaw, z=0.0):
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {
            "qx": 0.0,
            "qy": 0.0,
            "qz": math.sin(yaw / 2.0),
            "qw": math.cos(yaw / 2.0),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw,
        },
        "linear_velocity": {"x": 0.2, "y": 0.0, "z": 0.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def twist_data(vx, vy=0.0, yaw_rate=0.0):
    return {
        "linear": {"x": vx, "y": vy, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": yaw_rate},
    }


class ExperimentDiagnosticsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recording = load_script("route_recording_blackbox.py")
        cls.audit = load_script("go2_experiment_audit.py")

    def test_route_recording_audit_reproduces_distance_sampler(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            route = root / "route.csv"
            telemetry = root / "telemetry.jsonl"
            points = [
                (0.0, 0.0, 0.0),
                (0.4, 0.0, 0.0),
                (0.8, 0.0, 0.0),
            ]
            write_route(route, points)
            odom_points = [
                (-1.0, 0.0),
                (-0.5, 0.0),
                (0.0, 0.0),
                (0.2, 0.0),
                (0.4, 0.0),
                (0.6, 0.0),
                (0.8, 0.0),
            ]
            records = [
                telemetry_record(
                    "odom",
                    1000.0 + index * 0.1,
                    odom_data(x, y, 0.0),
                )
                for index, (x, y) in enumerate(odom_points)
            ]
            telemetry.write_text(
                "".join(json.dumps(record) + "\n" for record in records)
            )

            result = self.recording.build_route_audit(
                str(route),
                str(telemetry),
                0.4,
            )

            reproduction = result["recorder_reproduction"]
            self.assertTrue(
                reproduction["exact_within_2mm_0_002rad"]
            )
            self.assertEqual(
                reproduction["reconstructed_waypoint_count"],
                3,
            )

    def test_base_evidence_contains_only_current_experiment_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            workspace = root / "workspace"
            run_dir = root / "run"
            source = workspace / "patrol_logs" / "livox.log"
            source.parent.mkdir(parents=True)
            source.write_bytes(b"historical-run\n")

            offsets = self.recording.capture_log_offsets(workspace)
            with source.open("ab") as handle:
                handle.write(b"current-run-a\ncurrent-run-b\n")

            copied = self.recording.copy_base_evidence(
                workspace,
                run_dir,
                offsets=offsets,
            )

            self.assertEqual(len(copied), 1)
            destination = run_dir / copied[0]["copy"]
            self.assertEqual(
                destination.read_bytes(),
                b"current-run-a\ncurrent-run-b\n",
            )
            self.assertEqual(
                copied[0]["start_offset_bytes"],
                len(b"historical-run\n"),
            )
            self.assertFalse(
                copied[0]["source_truncated_since_start"]
            )

    def test_localization_configuration_records_runtime_extrinsics(self):
        snapshot = {
            "commands": {
                "ros_params_fastlio": {
                    "stdout": (
                        "/laser_mapping:\n"
                        "  ros__parameters:\n"
                        "    extrinsic_est_en: false\n"
                        "    extrinsic_T: [-0.011, -0.02329, 0.04412]\n"
                        "    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                        "    lidar_qos_depth: 2\n"
                        "    imu_qos_depth: 400\n"
                    )
                },
                "ros_params_livox": {"stdout": "publish_freq: 10.0\n"},
            },
            "configuration_files": {
                "install/livox_ros_driver2/share/livox_ros_driver2/"
                "config/MID360s_config.json": {
                    "text": json.dumps(
                        {
                            "lidar_configs": [
                                {
                                    "extrinsic_parameter": {
                                        "roll": 0.0,
                                        "pitch": 0.0,
                                        "yaw": 0.0,
                                    }
                                }
                            ]
                        }
                    )
                }
            },
        }

        result = self.audit.localization_configuration_audit(
            snapshot
        )

        self.assertFalse(result["extrinsic_estimation_enabled"])
        self.assertEqual(result["lidar_qos_depth"], 2)
        self.assertEqual(result["imu_qos_depth"], 400)
        self.assertEqual(
            result["lidar_to_imu_rotation_identity_max_error"],
            0.0,
        )
        self.assertEqual(
            result["livox_configured_extrinsic"]["pitch"],
            0.0,
        )

    def test_patrol_audit_proves_transform_and_tracking_chain(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            original = [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (2.0, 0.0, 0.0),
            ]
            delta = 0.20
            anchor_x = 10.0
            anchor_y = 5.0
            runtime = [
                (
                    anchor_x + math.cos(delta) * point[0],
                    anchor_y + math.sin(delta) * point[0],
                    delta,
                )
                for point in original
            ]
            write_route(run_dir / "route_original.csv", original)
            write_route(run_dir / "route_runtime.csv", runtime)
            (run_dir / "manual_anchor.json").write_text(
                json.dumps(
                    {
                        "transform": {
                            "source_start": {
                                "x": 0.0,
                                "y": 0.0,
                                "yaw": 0.0,
                            },
                            "current_start": {
                                "x": anchor_x,
                                "y": anchor_y,
                                "yaw": delta,
                            },
                        }
                    }
                )
            )
            (run_dir / "route_recording.json").write_text(
                json.dumps(
                    {
                        "route_sha256": "test",
                        "recording_run_dir": "/recording/test",
                        "status": "complete",
                    }
                )
            )

            records = []
            control_trace = []
            timestamp = 1000.0
            for index in range(21):
                fraction = index / 10.0
                x = anchor_x + math.cos(delta) * fraction
                y = anchor_y + math.sin(delta) * fraction
                records.append(
                    telemetry_record(
                        "odom",
                        timestamp,
                        odom_data(x, y, delta),
                    )
                )
                records.append(
                    telemetry_record(
                        "patrol_cmd",
                        timestamp + 0.001,
                        twist_data(0.2),
                    )
                )
                records.append(
                    telemetry_record(
                        "cmd_vel",
                        timestamp + 0.002,
                        twist_data(0.2),
                    )
                )
                records.append(
                    telemetry_record(
                        "sport",
                        timestamp + 0.025,
                        {
                            "velocity": [0.2, 0.0, 0.0],
                            "yaw_speed": 0.0,
                            "position": [fraction, 0.0, 0.0],
                            "imu": {
                                "orientation": {
                                    "qx": 0.0,
                                    "qy": 0.0,
                                    "qz": math.sin(delta / 2.0),
                                    "qw": math.cos(delta / 2.0),
                                }
                            },
                        },
                    )
                )
                control_trace.append(
                    {
                        "kind": "control",
                        "wall_time": timestamp + 0.001,
                        "monotonic_s": timestamp + 0.001,
                        "control_sequence": index + 1,
                        "odom_callback_sequence": index // 2 + 1,
                        "odom_source_stamp": timestamp,
                        "odom_to_control_ms": 5.0,
                        "odom_stamp_age_ms": 25.0,
                        "compute_to_trace_ms": 0.5,
                        "control": {
                            "source": "course_straight",
                            "cross_track_m": 0.0,
                            "selected_alpha": 0.0,
                            "is_stop": False,
                            "cmd_vx": 0.2,
                            "cmd_yaw_rate": 0.0,
                        },
                    }
                )
                timestamp += 0.1
            for kind in ("low", "livox_imu"):
                records.append(
                    telemetry_record(kind, 1000.0, {})
                )
            records.append(
                telemetry_record(
                    "recorder_health",
                    1002.1,
                    {
                        "received": {
                            "odom": 21,
                            "sport": 21,
                            "low": 1,
                            "livox_imu": 1,
                            "patrol_cmd": 21,
                            "cmd_vel": 21,
                        },
                        "written": {},
                        "receive_age_s": {},
                    },
                )
            )
            records.sort(key=lambda item: item["wall_time"])
            (run_dir / "experiment_telemetry.jsonl").write_text(
                "".join(json.dumps(record) + "\n" for record in records)
            )
            (run_dir / "follower_control_trace.jsonl").write_text(
                "".join(
                    json.dumps(record) + "\n"
                    for record in control_trace
                )
                + json.dumps(
                    {
                        "kind": "trace_stop",
                        "wall_time": timestamp,
                    }
                )
                + "\n"
            )
            rosbag_dir = run_dir / "rosbag"
            rosbag_dir.mkdir()
            (rosbag_dir / "run_0.db3").write_bytes(b"sqlite")
            (run_dir / "rosbag_info.txt").write_text(
                "\n".join(
                    "Topic: %s | Type: test/msg/Test | Count: 21 | "
                    "Serialization Format: cdr"
                    % topic
                    for topic in (
                        "/Odometry",
                        "/livox/imu",
                        "/lf/sportmodestate",
                        "/patrol_cmd",
                        "/cmd_vel",
                        "/api/sport/request",
                    )
                )
                + "\n"
            )

            report = self.audit.build_report(run_dir)

            self.assertTrue(
                report["coordinate_transform"][
                    "exact_within_csv_precision"
                ]
            )
            self.assertLess(
                report["route_tracking"]["cross_track_error_m"][
                    "max"
                ],
                2e-6,
            )
            self.assertEqual(
                report["command_chain"][
                    "nonzero_to_zero_override_records"
                ],
                0,
            )
            self.assertEqual(
                report["follower_control_trace"][
                    "consecutive_control_cycles_per_odom"
                ]["max"],
                2.0,
            )
            self.assertLess(
                report["follower_control_trace"][
                    "trace_to_patrol_cmd_absolute_error"
                ]["vx"]["max"],
                1e-9,
            )
            self.assertTrue(
                report["rosbag"]["critical_topics_complete"]
            )
            self.assertTrue(
                report["sensor_body_alignment"]["available"]
            )
            self.assertLess(
                report["sensor_body_alignment"][
                    "absolute_relative_rotation_change_from_first_deg"
                ]["yaw"]["max"],
                1e-9,
            )
            self.assertTrue(
                report["evidence_health"][
                    "complete_for_onboard_chain"
                ]
            )
            self.assertTrue(
                any(
                    finding["stage"] == "physical_ground_truth"
                    for finding in report["findings"]
                )
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
