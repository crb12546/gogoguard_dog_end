import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from import_platform_preparation import (  # noqa: E402
    PreparationImportError,
    prepare_platform_bundle,
)


def digest(payload):
    return hashlib.sha256(payload).hexdigest()


def json_bytes(document):
    return (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def category_semantics():
    policy = {
        "stable_include": ("stable_layer_roi", "extract_to_stable_layer"),
        "dynamic_exclude": (
            "static_map_exclude_roi",
            "exclude_from_cleaned_map",
        ),
        "vegetation_exclude": (
            "static_map_exclude_roi",
            "exclude_from_cleaned_map",
        ),
        "parking_exclude": (
            "static_map_exclude_roi",
            "exclude_from_cleaned_map",
        ),
        "low_confidence": ("review_finding", "none"),
        "repetitive_geometry": ("review_finding", "none"),
        "ghosting": ("review_finding", "none"),
        "drift_suspect": ("review_finding", "none"),
        "sparse": ("review_finding", "none"),
        "blind_zone": ("review_finding", "none"),
        "manual_review": ("review_finding", "none"),
    }
    return {
        name: {
            "label_zh": name,
            "role": role,
            "offline_map_action": action,
            "description_zh": "测试语义 %s" % name,
        }
        for name, (role, action) in policy.items()
    }


class PlatformPreparationImportTests(unittest.TestCase):
    def make_fixture(
        self,
        root,
        *,
        alignment_document_theta_delta=5e-13,
        aligned_route_x_offset=0.0,
    ):
        pcd_lines = [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            "WIDTH 30",
            "HEIGHT 1",
            "POINTS 30",
            "DATA ascii",
        ]
        for index in range(30):
            pcd_lines.append(
                "%.2f %.2f %.2f"
                % (index * 0.1, 0.02 * (index % 2), 0.2 * (index % 10))
            )
        pcd_bytes = ("\n".join(pcd_lines) + "\n").encode("ascii")
        source_pcd = root / "map.pcd"
        source_pcd.write_bytes(pcd_bytes)
        pcd_sha = digest(pcd_bytes)

        source_csv = (
            "id,x,y,yaw,v\r\n"
            "0,0.000000,0.000000,0.000000000,0.200\r\n"
            "1,0.400000,0.000000,0.000000000,0.200\r\n"
            "2,0.800000,0.000000,0.000000000,0.200\r\n"
            "3,1.200000,0.000000,0.000000000,0.200\r\n"
        ).encode("utf-8")
        route_csv = (
            "id,x,y,yaw,v\r\n"
            "0,%.6f,0.000000,0.000000000,0.200\r\n"
            "1,%.6f,0.000000,0.000000000,0.200\r\n"
            "2,%.6f,0.000000,0.000000000,0.200\r\n"
            "3,%.6f,0.000000,0.000000000,0.200\r\n"
            % tuple(
                value + aligned_route_x_offset
                for value in (0.0, 0.4, 0.8, 1.2)
            )
        ).encode("utf-8")
        source_csv_sha = digest(source_csv)
        route_csv_sha = digest(route_csv)

        preparation = {
            "schema": "go2.patrol_preparation/v1",
            "source": {
                "pcd_file_name": "map.pcd",
                "pcd_sha256": pcd_sha,
                "csv_file_name": "source.csv",
                "csv_sha256": source_csv_sha,
            },
            "alignment": {
                "type": "SE2",
                "theta_rad": alignment_document_theta_delta,
                "translation_m": [0.0, 0.0],
                "confirmed": True,
            },
            "trim": {
                "source_start_index": 0,
                "source_end_index": 3,
                "source_point_count": 4,
                "exported_point_count": 4,
            },
            "landmarks": {
                "approved_ids": ["wall-1"],
                "candidate_ids": [],
                "rejected_ids": [],
            },
            "checkpoints": [],
        }
        alignment = {
            "schema": "go2.route_alignment/v1",
            "status": "reviewed",
            "method": "operator-planar-drag-rotate/v1",
            "source": {
                "pcd_sha256": pcd_sha,
                "csv_sha256": source_csv_sha,
            },
            "transform": {
                "type": "SE2",
                "theta_rad": 0.0,
                "translation_m": [0.0, 0.0],
            },
            "evidence": {"note_zh": "测试"},
        }
        annotations = {
            "schema": "go2.map_review_annotations/v2",
            "revision": "test-review-r1",
            "exported_at_utc": "2026-07-27T00:00:00Z",
            "map": {
                "sha256": pcd_sha,
                "file_name": "map.pcd",
            },
            "coordinate_system": {
                "frame_id": "map",
                "coordinate_mode": "absolute",
                "linear_unit": "metre",
                "handedness": "right_handed",
                "axes": {
                    "x": "map +X",
                    "y": "map +Y",
                    "z": "map +Z",
                },
                "region_geometry": {
                    "type": "roi_geometry_union",
                    "geometry_field": "annotations[].geometry",
                    "supported_types": [
                        "circle",
                        "wall",
                        "box",
                        "cylinder",
                    ],
                },
            },
            "category_semantics": category_semantics(),
            "safety": {
                "source_map_mutation": "forbidden",
                "mask_application_requires_preview": True,
                "filtered_map_requires_new_sha256": True,
                "approved_roi_required_for_map_action": True,
                "localization_constraint_notice_zh": "测试固定物仍需配准质量门。",
            },
            "annotations": [
                {
                    "id": "wall-1",
                    "category": "stable_include",
                    "object_type": "wall",
                    "source": "manual",
                    "review_status": "approved",
                    "legacy_candidate": False,
                    "geometry": {
                        "type": "wall",
                        "start_m": {"x": 0.0, "y": 0.0},
                        "end_m": {"x": 3.0, "y": 0.0},
                        "width_m": 1.0,
                        "z_min_m": -0.5,
                        "z_max_m": 2.5,
                    },
                    "point_count": 30,
                    "note": "测试墙体",
                }
            ],
        }
        checkpoints = {
            "schema": "go2.route_checkpoints/v1",
            "source_pcd_sha256": pcd_sha,
            "source_csv_sha256": source_csv_sha,
            "route_csv_sha256": route_csv_sha,
            "route_revision": None,
            "checkpoints": [],
        }
        bundle = root / "preparation.zip"
        with zipfile.ZipFile(str(bundle), "w", zipfile.ZIP_STORED) as archive:
            archive.writestr("source/source.csv", source_csv)
            archive.writestr("route.aligned.csv", route_csv)
            archive.writestr("route.alignment.json", json_bytes(alignment))
            archive.writestr("map.annotations.json", json_bytes(annotations))
            archive.writestr("route.checkpoints.json", json_bytes(checkpoints))
            archive.writestr("preparation.json", json_bytes(preparation))
        return bundle, source_pcd

    def test_import_builds_one_hash_bound_runtime_asset_set(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, source_pcd = self.make_fixture(root)
            output = root / "prepared"
            report = prepare_platform_bundle(
                bundle,
                source_pcd,
                output,
                map_id="fixture-map-r1",
                minimum_stable_points=1,
            )
            self.assertEqual(report["checkpoint_count"], 0)
            self.assertEqual(report["route_points"], 4)
            self.assertEqual(report["approved_landmark_ids"], ["wall-1"])
            self.assertTrue(
                (output / "maps/fixture-map-r1/manifest.json").is_file()
            )
            self.assertTrue(
                (output / "routes/route.aligned.route.json").is_file()
            )
            self.assertTrue((output / "deployment.env").is_file())
            self.assertFalse(report["real_dog_running_verified"])
            route_metadata = json.loads(
                (output / "routes/route.aligned.route.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertTrue(
                route_metadata["alignment"]["operator_confirmed"]
            )
            self.assertFalse(
                route_metadata["alignment"]["field_truth_verified"]
            )

    def test_wrong_source_pcd_hash_fails_without_publishing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, source_pcd = self.make_fixture(root)
            source_pcd.write_bytes(source_pcd.read_bytes() + b"# changed\n")
            output = root / "must-not-exist"
            with self.assertRaisesRegex(
                PreparationImportError,
                "source PCD SHA-256",
            ):
                prepare_platform_bundle(
                    bundle,
                    source_pcd,
                    output,
                    map_id="fixture-map-r1",
                    minimum_stable_points=1,
                )
            self.assertFalse(output.exists())

    def test_materially_different_alignment_document_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, source_pcd = self.make_fixture(
                root,
                alignment_document_theta_delta=1e-6,
            )
            output = root / "must-not-exist"
            with self.assertRaisesRegex(
                PreparationImportError,
                "alignment document differs",
            ):
                prepare_platform_bundle(
                    bundle,
                    source_pcd,
                    output,
                    map_id="fixture-map-r1",
                    minimum_stable_points=1,
                )
            self.assertFalse(output.exists())

    def test_hash_bound_but_semantically_wrong_route_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle, source_pcd = self.make_fixture(
                root,
                aligned_route_x_offset=0.01,
            )
            output = root / "must-not-exist"
            with self.assertRaisesRegex(
                PreparationImportError,
                r"not the declared SE\(2\)\+trim",
            ):
                prepare_platform_bundle(
                    bundle,
                    source_pcd,
                    output,
                    map_id="fixture-map-r1",
                    minimum_stable_points=1,
                )
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
