"""Command-line entry point for the map asset pipeline."""

import argparse
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

from .annotation_filter import (
    AnnotationFilterError,
    filter_pcd_with_annotations,
)
from .descriptor import (
    DescriptorError,
    PolarDescriptorConfig,
    build_descriptor_index,
    compile_map_bundle,
    compute_polar_descriptor,
    load_descriptor_index,
    verify_map_bundle,
)
from .pcd import PCDDataError, read_pcd, write_pcd_ascii
from .review import ReviewBundleError, build_review_bundle
from .reviewed_publish import (
    REVIEWED_PUBLICATION_FILENAME,
    ReviewedMapPublicationError,
    publish_reviewed_map_bundle,
    verify_reviewed_map_bundle,
)
from .tiles import MapManifestError, build_tiled_map, verify_manifest
from .voxel import voxel_downsample_cloud


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="go2-map", description="Build and verify Go2 PCD map assets."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    info = subparsers.add_parser("pcd-info", help="inspect a PCD file")
    info.add_argument("pcd", type=Path)
    info.add_argument("--allow-nonfinite", action="store_true")

    downsample = subparsers.add_parser("downsample", help="voxel-downsample PCD")
    downsample.add_argument("source", type=Path)
    downsample.add_argument("destination", type=Path)
    downsample.add_argument("--voxel-size", type=float, required=True)

    tile = subparsers.add_parser("tile", help="build deterministic map tiles")
    tile.add_argument("source", type=Path)
    tile.add_argument("output", type=Path)
    tile.add_argument("--map-id")
    tile.add_argument("--frame-id", default="map")
    tile.add_argument("--tile-size", type=float, default=20.0)
    tile.add_argument("--voxel-size", type=float, default=0.20)
    tile.add_argument("--overwrite", action="store_true")

    verify = subparsers.add_parser("verify", help="verify manifest and tile hashes")
    verify.add_argument("manifest", type=Path)
    verify.add_argument(
        "--tiles-only",
        action="store_true",
        help=(
            "verify the intermediate manifest and tiles without requiring " "an index"
        ),
    )

    index = subparsers.add_parser("build-index", help="build polar descriptor index")
    index.add_argument("manifest", type=Path)
    index.add_argument("--output", type=Path)
    _descriptor_arguments(index)

    compile_map = subparsers.add_parser(
        "compile",
        help=("compile one PCD into verified tiles, manifest, and descriptor " "index"),
    )
    compile_map.add_argument("source", type=Path)
    compile_map.add_argument("output", type=Path)
    compile_map.add_argument("--map-id")
    compile_map.add_argument("--frame-id", default="map")
    compile_map.add_argument("--tile-size", type=float, default=20.0)
    compile_map.add_argument("--voxel-size", type=float, default=0.20)
    compile_map.add_argument("--overwrite", action="store_true")
    _descriptor_arguments(compile_map)

    query = subparsers.add_parser("query-index", help="query index with a PCD scan")
    query.add_argument("index", type=Path)
    query.add_argument("scan", type=Path)
    query.add_argument("--limit", type=int, default=5)
    query.add_argument("--center-x", type=float, default=0.0)
    query.add_argument("--center-y", type=float, default=0.0)
    query.add_argument("--center-z", type=float, default=0.0)

    review = subparsers.add_parser(
        "review-bundle",
        help="build a deterministic point-cloud quality review bundle",
    )
    review.add_argument("source", type=Path)
    review.add_argument("output", type=Path)
    review.add_argument("--keyframes", type=Path)
    review.add_argument("--session", type=Path)
    review.add_argument("--max-preview-points", type=int, default=500_000)

    annotation_filter = subparsers.add_parser(
        "filter-annotations",
        help=("apply hash-bound explicit exclusion masks to a new PCD asset"),
    )
    annotation_filter.add_argument("source", type=Path)
    annotation_filter.add_argument("annotations", type=Path)
    annotation_filter.add_argument("output", type=Path)

    publish_reviewed = subparsers.add_parser(
        "publish-reviewed",
        help=(
            "filter annotations, compile cleaned tracking tiles, build global "
            "descriptors from the stable layer, and publish one immutable bundle"
        ),
    )
    publish_reviewed.add_argument("source", type=Path)
    publish_reviewed.add_argument("annotations", type=Path)
    publish_reviewed.add_argument("output", type=Path)
    publish_reviewed.add_argument("--map-id", required=True)
    publish_reviewed.add_argument("--frame-id", default="map")
    publish_reviewed.add_argument("--tile-size", type=float, default=20.0)
    publish_reviewed.add_argument("--voxel-size", type=float, default=0.20)
    publish_reviewed.add_argument("--minimum-stable-points", type=int, default=1000)
    _descriptor_arguments(publish_reviewed)

    verify_reviewed = subparsers.add_parser(
        "verify-reviewed",
        help="verify every reviewed publication hash and cross-file binding",
    )
    verify_reviewed.add_argument(
        "publication",
        type=Path,
        help=("reviewed_map_publication.json or the containing map directory"),
    )
    return parser


def _descriptor_arguments(parser: argparse.ArgumentParser) -> None:
    defaults = PolarDescriptorConfig()
    parser.add_argument("--rings", type=int, default=defaults.rings)
    parser.add_argument("--sectors", type=int, default=defaults.sectors)
    parser.add_argument("--max-radius", type=float, default=defaults.max_radius_m)
    parser.add_argument("--min-z", type=float, default=defaults.min_z_m)
    parser.add_argument("--max-z", type=float, default=defaults.max_z_m)


def _bounds(points):
    if not points:
        return None
    return {
        "min": [min(point[index] for point in points) for index in range(3)],
        "max": [max(point[index] for point in points) for index in range(3)],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "pcd-info":
            cloud = read_pcd(
                arguments.pcd, reject_nonfinite=not arguments.allow_nonfinite
            )
            print(
                json.dumps(
                    {
                        "encoding": cloud.data_encoding,
                        "fields": [field.name for field in cloud.fields],
                        "height": cloud.height,
                        "point_count": cloud.point_count,
                        "width": cloud.width,
                        "xyz_bounds": (
                            _bounds(cloud.xyz_points())
                            if {"x", "y", "z"}.issubset(
                                {field.name for field in cloud.fields}
                            )
                            else None
                        ),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "downsample":
            cloud = read_pcd(arguments.source)
            result = voxel_downsample_cloud(cloud, arguments.voxel_size)
            write_pcd_ascii(result, arguments.destination)
            print(
                json.dumps(
                    {
                        "input_points": cloud.point_count,
                        "output_points": result.point_count,
                        "output": str(arguments.destination),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "tile":
            manifest = build_tiled_map(
                arguments.source,
                arguments.output,
                map_id=arguments.map_id,
                frame_id=arguments.frame_id,
                tile_size_m=arguments.tile_size,
                voxel_size_m=arguments.voxel_size,
                overwrite=arguments.overwrite,
            )
            print(
                json.dumps(
                    {
                        "map_id": manifest.map_id,
                        "tiles": len(manifest.tiles),
                        "points": manifest.tiled_point_count,
                        "manifest": str(arguments.output / "manifest.json"),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "verify":
            if arguments.tiles_only:
                manifest = verify_manifest(arguments.manifest)
                descriptor_count = None
            else:
                manifest, verified_index = verify_map_bundle(arguments.manifest)
                descriptor_count = len(verified_index.entries)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "map_id": manifest.map_id,
                        "tiles": len(manifest.tiles),
                        "descriptor_entries": descriptor_count,
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "build-index":
            config = PolarDescriptorConfig(
                rings=arguments.rings,
                sectors=arguments.sectors,
                max_radius_m=arguments.max_radius,
                min_z_m=arguments.min_z,
                max_z_m=arguments.max_z,
            )
            index = build_descriptor_index(arguments.manifest, arguments.output, config)
            print(
                json.dumps(
                    {
                        "map_id": index.map_id,
                        "entries": len(index.entries),
                        "output": str(
                            arguments.output
                            or (
                                arguments.manifest
                                if arguments.manifest.is_dir()
                                else arguments.manifest.parent
                            )
                            / "descriptor_index.json"
                        ),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "compile":
            config = PolarDescriptorConfig(
                rings=arguments.rings,
                sectors=arguments.sectors,
                max_radius_m=arguments.max_radius,
                min_z_m=arguments.min_z,
                max_z_m=arguments.max_z,
            )
            manifest, index = compile_map_bundle(
                arguments.source,
                arguments.output,
                map_id=arguments.map_id,
                frame_id=arguments.frame_id,
                tile_size_m=arguments.tile_size,
                voxel_size_m=arguments.voxel_size,
                descriptor_config=config,
                overwrite=arguments.overwrite,
            )
            manifest_path = arguments.output / "manifest.json"
            index_path = arguments.output / "descriptor_index.json"
            print(
                json.dumps(
                    {
                        "ok": True,
                        "map_id": manifest.map_id,
                        "source_points": manifest.source_point_count,
                        "tiled_points": manifest.tiled_point_count,
                        "tiles": len(manifest.tiles),
                        "descriptor_entries": len(index.entries),
                        "manifest": str(manifest_path),
                        "descriptor_index": str(index_path),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "query-index":
            index = load_descriptor_index(arguments.index)
            scan = read_pcd(arguments.scan)
            descriptor = compute_polar_descriptor(
                scan.xyz_points(),
                index.config,
                (arguments.center_x, arguments.center_y, arguments.center_z),
            )
            matches = index.query(descriptor, arguments.limit)
            print(
                json.dumps(
                    [
                        {
                            "id": match.id,
                            "tile_id": match.tile_id,
                            "center": list(match.center),
                            "score": match.score,
                            "sector_shift": match.sector_shift,
                            "yaw_offset_rad": match.yaw_offset_rad,
                        }
                        for match in matches
                    ],
                    sort_keys=True,
                )
            )
        elif arguments.command == "review-bundle":
            review = build_review_bundle(
                arguments.source,
                arguments.output,
                keyframes_path=arguments.keyframes,
                session_path=arguments.session,
                max_preview_points=arguments.max_preview_points,
            )
            print(
                json.dumps(
                    {
                        "ok": True,
                        "output": str(arguments.output),
                        "review": str(arguments.output / "review.json"),
                        "review_sha256": review["review_sha256"],
                        "source_points": review["source"]["point_count"],
                        "preview_points": review["preview"]["point_count"],
                        "warning_count": len(review["warnings"]),
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "filter-annotations":
            report = filter_pcd_with_annotations(
                arguments.source,
                arguments.annotations,
                arguments.output,
            )
            print(
                json.dumps(
                    {
                        "ok": True,
                        "output": report["output_directory"],
                        "filtered_pcd": str(
                            Path(report["output_directory"])
                            / report["output"]["filename"]
                        ),
                        "output_sha256": report["output"]["sha256"],
                        "report": str(
                            Path(report["output_directory"]) / "filter_report.json"
                        ),
                        "report_sha256": report["report_sha256"],
                        "source_points": report["source"]["point_count"],
                        "removed_points": report["filtering"]["removed_point_count"],
                        "retained_points": report["filtering"]["retained_point_count"],
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "publish-reviewed":
            config = PolarDescriptorConfig(
                rings=arguments.rings,
                sectors=arguments.sectors,
                max_radius_m=arguments.max_radius,
                min_z_m=arguments.min_z,
                max_z_m=arguments.max_z,
            )
            publication = publish_reviewed_map_bundle(
                arguments.source,
                arguments.annotations,
                arguments.output,
                map_id=arguments.map_id,
                frame_id=arguments.frame_id,
                tile_size_m=arguments.tile_size,
                voxel_size_m=arguments.voxel_size,
                descriptor_config=config,
                minimum_stable_points=arguments.minimum_stable_points,
            )
            print(
                json.dumps(
                    {
                        "ok": True,
                        "output": publication["output_directory"],
                        "publication": str(
                            Path(publication["output_directory"])
                            / REVIEWED_PUBLICATION_FILENAME
                        ),
                        "publication_sha256": publication["publication_sha256"],
                        "map_id": publication["map_id"],
                        "source_pcd_sha256": publication["source_pcd"]["sha256"],
                        "annotation_revision": publication["annotation"]["revision"],
                        "cleaned_map_sha256": publication["cleaned_static_map"][
                            "sha256"
                        ],
                        "stable_layer_sha256": publication["stable_layer"]["sha256"],
                        "stable_layer_points": publication["stable_layer"][
                            "point_count"
                        ],
                        "manifest_sha256": publication["compiled_map"][
                            "manifest_sha256"
                        ],
                        "deployment_ready": publication["deployment_ready"],
                    },
                    sort_keys=True,
                )
            )
        elif arguments.command == "verify-reviewed":
            publication = verify_reviewed_map_bundle(arguments.publication)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "map_id": publication["map_id"],
                        "publication_sha256": publication["publication_sha256"],
                        "manifest_sha256": publication["compiled_map"][
                            "manifest_sha256"
                        ],
                        "cleaned_map_sha256": publication["cleaned_static_map"][
                            "sha256"
                        ],
                        "stable_layer_sha256": publication["stable_layer"]["sha256"],
                        "stable_layer_points": publication["stable_layer"][
                            "point_count"
                        ],
                    },
                    sort_keys=True,
                )
            )
        return 0
    except (
        AnnotationFilterError,
        DescriptorError,
        MapManifestError,
        PCDDataError,
        ReviewBundleError,
        ReviewedMapPublicationError,
        OSError,
        ValueError,
    ) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
