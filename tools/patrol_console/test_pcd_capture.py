import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = PROJECT_ROOT / "orin_go2_fastlio_ws" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import go2_pcd_capture as CAPTURE  # noqa: E402
from horizontal_frame import (  # noqa: E402
    angle_between,
    quaternion_from_two_vectors,
)


class PcdCaptureTests(unittest.TestCase):
    def test_known_slanted_plane_is_rigidly_leveled(self):
        map_up = np.asarray((-0.41, -0.06, 0.91), dtype=float)
        map_up /= np.linalg.norm(map_up)
        q_ground_from_map = quaternion_from_two_vectors(
            map_up, (0.0, 0.0, 1.0)
        )

        axis_a = np.cross(map_up, (0.0, 1.0, 0.0))
        axis_a /= np.linalg.norm(axis_a)
        axis_b = np.cross(map_up, axis_a)
        points = np.asarray(
            [
                x * axis_a + y * axis_b + 0.7 * map_up
                for x in np.linspace(-4.0, 4.0, 17)
                for y in np.linspace(-3.0, 3.0, 13)
            ]
        )
        leveled = CAPTURE.level_points(points, q_ground_from_map)

        centered = leveled - np.mean(leveled, axis=0)
        _, _, right_vectors = np.linalg.svd(centered)
        fitted_normal = right_vectors[-1]
        if fitted_normal[2] < 0.0:
            fitted_normal *= -1.0
        self.assertLess(
            angle_between(fitted_normal, (0.0, 0.0, 1.0)),
            math.radians(1e-6),
        )
        raw_distances = np.linalg.norm(points - points[0], axis=1)
        level_distances = np.linalg.norm(leveled - leveled[0], axis=1)
        self.assertLess(
            float(np.max(np.abs(raw_distances - level_distances))),
            1e-12,
        )

    def test_writer_preserves_point_count_and_uses_atomic_final_name(self):
        points = np.asarray(
            [
                (1.25, -2.5, 0.75),
                (-3.0, 4.0, 1.5),
                (0.0, 0.0, -0.25),
            ]
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.pcd"
            CAPTURE.write_pcd_xyz(output, points)
            text = output.read_text()
            self.assertIn("FIELDS x y z", text)
            self.assertIn("WIDTH 3", text)
            self.assertIn("POINTS 3", text)
            self.assertEqual(len(text.splitlines()), 14)
            self.assertFalse(list(Path(directory).glob("*.partial.*")))

    def test_calibration_has_provenance_and_valid_quaternions(self):
        path = (
            PROJECT_ROOT
            / "orin_go2_fastlio_ws"
            / "config"
            / "horizontal_frame_calibration.json"
        )
        document, calibration = CAPTURE.load_calibration(path)
        self.assertEqual(
            document["schema"], "go2.horizontal_frame_calibration.v1"
        )
        self.assertEqual(
            document["source_evidence"]["route_sha256"],
            "7f4312a12935c451bc3347b1af4ca6507a88932a4acafb90aebdf1de6bc64c8b",
        )
        self.assertEqual(
            len(calibration["q_sensor_from_body_xyzw"]), 4
        )
        self.assertEqual(
            len(calibration["q_lidar_gravity_correction_xyzw"]), 4
        )
        json.dumps(document, allow_nan=False)

    def test_default_companion_paths_hide_raw_map_from_primary_list(self):
        output = Path("/maps/console/site.pcd")
        raw, metadata = CAPTURE.default_companion_paths(output)
        self.assertEqual(raw, Path("/maps/console/raw/site.pcd"))
        self.assertEqual(
            metadata, Path("/maps/console/site.leveling.json")
        )


if __name__ == "__main__":
    unittest.main()
