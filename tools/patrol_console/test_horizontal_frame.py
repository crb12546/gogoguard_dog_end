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

from horizontal_frame import (  # noqa: E402
    HorizontalFrameEstimator,
    align_route_to_pose,
    angle_between,
    quaternion_from_two_vectors,
    quaternion_multiply,
    quaternion_rotate,
    quaternion_yaw,
)


class HorizontalFrameTests(unittest.TestCase):
    def test_shortest_rotation_levels_known_map_up(self):
        map_up = (-0.39, -0.03, 0.92)
        ground_from_map = quaternion_from_two_vectors(
            map_up, (0.0, 0.0, 1.0)
        )
        leveled = quaternion_rotate(ground_from_map, map_up)
        self.assertLess(
            angle_between(leveled, (0.0, 0.0, 1.0)),
            math.radians(1e-5),
        )

    def test_estimator_fuses_lidar_and_body_without_yaw_swap(self):
        sensor_from_body = quaternion_from_two_vectors(
            (0.0, 0.0, 1.0), (-0.53, -0.02, 0.85)
        )
        map_from_ground = quaternion_from_two_vectors(
            (0.0, 0.0, 1.0), (-0.39, -0.03, 0.92)
        )
        map_from_sensor = quaternion_multiply(
            map_from_ground,
            (
                -sensor_from_body[0],
                -sensor_from_body[1],
                -sensor_from_body[2],
                sensor_from_body[3],
            ),
        )
        estimator = HorizontalFrameEstimator(
            q_sensor_from_body=sensor_from_body,
            minimum_samples=5,
            maximum_spread_rad=math.radians(0.2),
            maximum_source_disagreement_rad=math.radians(0.2),
        )
        locked = False
        for _ in range(5):
            locked = estimator.add_sample(
                map_from_sensor,
                (-0.53, -0.02, 0.85),
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
        self.assertTrue(locked)
        self.assertTrue(estimator.ready)
        body_orientation = estimator.transform_body_orientation(
            map_from_sensor
        )
        body_up = quaternion_rotate(
            body_orientation, (0.0, 0.0, 1.0)
        )
        self.assertLess(
            angle_between(body_up, (0.0, 0.0, 1.0)),
            math.radians(1e-5),
        )

    def test_route_anchor_is_one_rigid_horizontal_transform(self):
        route = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.2},
            {"x": 2.0, "y": 0.0, "yaw": 0.0, "v": 0.2},
        ]
        aligned, rotation = align_route_to_pose(
            route, 10.0, -3.0, math.pi / 2.0
        )
        self.assertAlmostEqual(rotation, math.pi / 2.0)
        self.assertAlmostEqual(aligned[0]["x"], 10.0)
        self.assertAlmostEqual(aligned[0]["y"], -3.0)
        self.assertAlmostEqual(aligned[1]["x"], 10.0)
        self.assertAlmostEqual(aligned[1]["y"], -1.0)
        self.assertAlmostEqual(
            quaternion_yaw((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))),
            math.pi / 2.0,
        )


if __name__ == "__main__":
    unittest.main()
