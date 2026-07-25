import csv
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = (
    ROOT / "orin_go2_fastlio_ws/scripts/manual_route_anchor.py"
)
SPEC = importlib.util.spec_from_file_location("manual_route_anchor", MODULE_PATH)
ANCHOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANCHOR)


class ManualRouteAnchorTest(unittest.TestCase):
    def test_rigid_transform_preserves_route_and_extra_columns(self):
        fieldnames = ["id", "x", "y", "yaw", "v", "camera"]
        rows = [
            {
                "id": "0",
                "x": "3.0",
                "y": "-4.0",
                "yaw": "2.9",
                "v": "0.2",
                "camera": "front",
            },
            {
                "id": "1",
                "x": "4.0",
                "y": "-4.0",
                "yaw": "3.0",
                "v": "0.3",
                "camera": "left",
            },
            {
                "id": "2",
                "x": "4.0",
                "y": "-2.0",
                "yaw": "-2.8",
                "v": "0.4",
                "camera": "right",
            },
        ]

        output, transform = ANCHOR.transform_route_rows(
            fieldnames,
            rows,
            anchor_x=-2.0,
            anchor_y=7.0,
            anchor_yaw=-3.0,
        )

        self.assertEqual(float(output[0]["x"]), -2.0)
        self.assertEqual(float(output[0]["y"]), 7.0)
        self.assertAlmostEqual(float(output[0]["yaw"]), -3.0, places=6)
        self.assertEqual(
            [row["camera"] for row in output],
            ["front", "left", "right"],
        )
        self.assertEqual(
            [row["v"] for row in output],
            ["0.2", "0.3", "0.4"],
        )
        self.assertTrue(-math.pi <= transform["delta_yaw"] <= math.pi)

        def segment_lengths(points):
            return [
                math.hypot(
                    float(points[index]["x"])
                    - float(points[index - 1]["x"]),
                    float(points[index]["y"])
                    - float(points[index - 1]["y"]),
                )
                for index in range(1, len(points))
            ]

        for before, after in zip(
            segment_lengths(rows),
            segment_lengths(output),
        ):
            self.assertAlmostEqual(before, after, places=5)

    def test_atomic_csv_keeps_header_order_and_row_count(self):
        fieldnames = ["id", "x", "y", "yaw", "v"]
        rows = [
            {"id": "0", "x": "0.0", "y": "0.0", "yaw": "0.0", "v": "0.2"},
            {"id": "1", "x": "1.0", "y": "0.0", "yaw": "0.0", "v": "0.2"},
        ]
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "runtime.csv"
            ANCHOR._atomic_write_csv(output, fieldnames, rows)
            with output.open(newline="") as handle:
                reader = csv.DictReader(handle)
                loaded = list(reader)
                self.assertEqual(reader.fieldnames, fieldnames)
            self.assertEqual(loaded, rows)

    def test_session_identity_uses_boot_pid_and_process_start(self):
        expected = {"boot_id": "boot-a", "pid": 12, "start_ticks": 345}
        self.assertTrue(
            ANCHOR.sessions_equal(
                expected,
                {"boot_id": "boot-a", "pid": 12, "start_ticks": 345},
            )
        )
        for changed in (
            {"boot_id": "boot-b", "pid": 12, "start_ticks": 345},
            {"boot_id": "boot-a", "pid": 13, "start_ticks": 345},
            {"boot_id": "boot-a", "pid": 12, "start_ticks": 346},
        ):
            self.assertFalse(ANCHOR.sessions_equal(expected, changed))


if __name__ == "__main__":
    unittest.main()
