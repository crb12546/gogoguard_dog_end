import math
import sys
import unittest
from pathlib import Path


SCRIPTS = (
    Path(__file__).resolve().parents[2]
    / "orin_go2_fastlio_ws"
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS))

from body_yaw_alignment import BodyYawAlignment, normalize_angle  # noqa: E402


class BodyYawAlignmentTests(unittest.TestCase):
    def test_stable_pairs_lock_and_preserve_body_turn(self):
        alignment = BodyYawAlignment(
            minimum_samples=5,
            max_spread_rad=math.radians(1.0),
        )
        for noise_deg in (-0.2, 0.1, 0.0, 0.2, -0.1):
            body = 0.20
            lio = body + math.radians(3.0 + noise_deg)
            alignment.add_pair(lio, body)

        self.assertTrue(alignment.ready)
        body_after_turn = math.radians(95.0)
        expected = normalize_angle(body_after_turn + math.radians(3.0))
        self.assertAlmostEqual(
            alignment.aligned_yaw(body_after_turn),
            expected,
            delta=math.radians(0.1),
        )

    def test_wraparound_pairs_use_circular_mean(self):
        alignment = BodyYawAlignment(
            minimum_samples=3,
            max_spread_rad=math.radians(1.0),
        )
        body = math.radians(179.0)
        for offset_deg in (2.0, 2.1, 1.9):
            lio = normalize_angle(body + math.radians(offset_deg))
            alignment.add_pair(lio, body)

        self.assertTrue(alignment.ready)
        self.assertAlmostEqual(
            alignment.offset_rad,
            math.radians(2.0),
            delta=math.radians(0.1),
        )

    def test_unstable_window_does_not_lock_until_stable(self):
        alignment = BodyYawAlignment(
            minimum_samples=4,
            max_spread_rad=math.radians(1.0),
        )
        for delta_deg in (0.0, 5.0, -4.0, 3.0):
            alignment.add_pair(math.radians(delta_deg), 0.0)
        self.assertFalse(alignment.ready)

        for delta_deg in (2.0, 2.1, 1.9, 2.0):
            alignment.add_pair(math.radians(delta_deg), 0.0)
        self.assertTrue(alignment.ready)


if __name__ == "__main__":
    unittest.main()
