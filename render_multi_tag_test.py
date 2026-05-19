from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.board import A4_HEIGHT_M, A4_WIDTH_M, create_irregular_table_layout, save_board_layout
from src.calibration import calibrate_from_images
from src.detection import detect_tags, draw_detections, load_image
from src.pose import draw_pose_axes, estimate_camera_pose
from src.render import default_camera, look_at_tcw, paper_corners_world, project_points, render_board_texture, render_view
from utils.camera import save_camera
from utils.geometry import invert_transform, rotation_angle_deg, rvec_tvec_to_matrix
from utils.io import ensure_dir, write_json


DEFAULT_TABLE_WIDTH_M = 2.8
DEFAULT_TABLE_HEIGHT_M = 2.8


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render and validate an irregular multi-page AprilTag table layout.")
    parser.add_argument("--output-dir", default="outputs/multi_tag_render_test")
    parser.add_argument("--tag-count", type=int, default=16)
    parser.add_argument("--tag-size-m", type=float, default=0.045)
    parser.add_argument("--table-width-m", type=float, default=DEFAULT_TABLE_WIDTH_M)
    parser.add_argument("--table-height-m", type=float, default=DEFAULT_TABLE_HEIGHT_M)
    parser.add_argument("--paper-width-m", type=float, default=A4_WIDTH_M)
    parser.add_argument("--paper-height-m", type=float, default=A4_HEIGHT_M)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=900)
    parser.add_argument("--focal-px", type=float, default=950.0)
    parser.add_argument("--calibration-views", type=int, default=10)
    parser.add_argument("--min-calibration-views", type=int, default=5)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args(argv)


def _polygon_area(points: np.ndarray) -> float:
    pts = np.asarray(points, dtype=np.float64)
    x = pts[:, 0]
    y = pts[:, 1]
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) * 0.5)


def _eye_from_oblique_view(target: np.ndarray, distance_m: float, tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    tilt = math.radians(float(tilt_deg))
    azimuth = math.radians(float(azimuth_deg))
    direction = np.array(
        [
            math.sin(tilt) * math.cos(azimuth),
            math.sin(tilt) * math.sin(azimuth),
            math.cos(tilt),
        ],
        dtype=np.float64,
    )
    return target + distance_m * direction


def _projected_table_area_ratio(layout, camera, rvec, tvec, image_size: tuple[int, int]) -> float:
    width, height = image_size
    corners = project_points(paper_corners_world(layout), camera, rvec, tvec)
    return _polygon_area(corners) / float(width * height)


def _solve_oblique_pose_for_area(
    layout,
    camera,
    image_size: tuple[int, int],
    tilt_deg: float,
    azimuth_deg: float,
    target_area_ratio: float,
    target: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    target_w = np.asarray(target, dtype=np.float64)
    lo = 0.08
    hi = 5.0
    best = None
    for _ in range(80):
        distance = (lo + hi) * 0.5
        eye = _eye_from_oblique_view(target_w, distance, tilt_deg, azimuth_deg)
        rvec, tvec = look_at_tcw(eye, target_w)
        area_ratio = _projected_table_area_ratio(layout, camera, rvec, tvec, image_size)
        best = (eye, rvec, tvec, area_ratio)
        if area_ratio > target_area_ratio:
            lo = distance
        else:
            hi = distance
    assert best is not None
    return best


def _calibration_specs() -> list[dict[str, Any]]:
    return [
        {"tilt_deg": 35.0, "azimuth_deg": 0.0, "target_area_ratio": 0.36, "noise_std": 0.8},
        {"tilt_deg": 42.0, "azimuth_deg": 33.0, "target_area_ratio": 0.34, "noise_std": 0.8},
        {"tilt_deg": 48.0, "azimuth_deg": 70.0, "target_area_ratio": 0.35, "noise_std": 1.0},
        {"tilt_deg": 54.0, "azimuth_deg": 112.0, "target_area_ratio": 0.32, "noise_std": 1.0},
        {"tilt_deg": 60.0, "azimuth_deg": 150.0, "target_area_ratio": 0.34, "noise_std": 1.2},
        {"tilt_deg": 66.0, "azimuth_deg": 198.0, "target_area_ratio": 0.31, "noise_std": 1.2},
        {"tilt_deg": 70.0, "azimuth_deg": 245.0, "target_area_ratio": 0.33, "noise_std": 1.4},
        {"tilt_deg": 52.0, "azimuth_deg": 292.0, "target_area_ratio": 0.35, "noise_std": 1.0},
        {"tilt_deg": 62.0, "azimuth_deg": 330.0, "target_area_ratio": 0.32, "noise_std": 1.2},
        {"tilt_deg": 72.0, "azimuth_deg": 25.0, "target_area_ratio": 0.30, "noise_std": 1.4},
        {"tilt_deg": 45.0, "azimuth_deg": 210.0, "target_area_ratio": 0.35, "noise_std": 1.0},
        {"tilt_deg": 68.0, "azimuth_deg": 310.0, "target_area_ratio": 0.31, "noise_std": 1.4},
    ]


def _evaluation_specs() -> list[dict[str, Any]]:
    return [
        {"tilt_deg": 46.0, "azimuth_deg": 18.0, "target_area_ratio": 0.40, "target": (0.00, 0.00, 0.0), "hide": [1, 6, 11]},
        {"tilt_deg": 55.0, "azimuth_deg": 76.0, "target_area_ratio": 0.42, "target": (0.04, 0.01, 0.0), "hide": [0, 5, 10, 15]},
        {"tilt_deg": 63.0, "azimuth_deg": 136.0, "target_area_ratio": 0.38, "target": (-0.03, 0.03, 0.0), "hide": [2, 7, 12]},
        {"tilt_deg": 70.0, "azimuth_deg": 202.0, "target_area_ratio": 0.40, "target": (0.05, -0.02, 0.0), "hide": [3, 8, 13]},
        {"tilt_deg": 74.0, "azimuth_deg": 260.0, "target_area_ratio": 0.36, "target": (-0.04, -0.02, 0.0), "hide": [4, 9, 14]},
        {"tilt_deg": 78.0, "azimuth_deg": 318.0, "target_area_ratio": 0.34, "target": (0.02, 0.04, 0.0), "hide": [1, 2, 12, 13]},
    ]


def _hidden_tag_ids(layout, indexes: list[int]) -> list[int]:
    tags = list(layout.tags)
    return [tags[index % len(tags)].tag_id for index in indexes]


def _occlusion_polygons_for_tags(layout, camera, rvec, tvec, hidden_tag_ids: list[int]) -> list[list[tuple[float, float]]]:
    tags = layout.tag_by_id()
    polygons: list[list[tuple[float, float]]] = []
    for tag_id in hidden_tag_ids:
        tag = tags[tag_id]
        pts = project_points(tag.corners, camera, rvec, tvec)
        center = np.mean(pts, axis=0, keepdims=True)
        expanded = center + (pts - center) * 1.25
        polygons.append([(float(x), float(y)) for x, y in expanded])
    return polygons


def _pose_error(est_t_cw: np.ndarray, true_t_cw: np.ndarray) -> dict[str, float]:
    est_t_wc = invert_transform(est_t_cw)
    true_t_wc = invert_transform(true_t_cw)
    center_error = float(np.linalg.norm(est_t_wc[:3, 3] - true_t_wc[:3, 3]))
    rot_error = rotation_angle_deg(est_t_wc[:3, :3], true_t_wc[:3, :3])
    return {
        "camera_center_error_m": center_error,
        "rotation_error_deg": rot_error,
    }


def _localize_eval_images(
    image_paths: list[Path],
    true_poses: dict[str, np.ndarray],
    view_metadata: dict[str, dict[str, Any]],
    layout,
    camera,
    annotate_dir: Path,
    label: str,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    ensure_dir(annotate_dir)
    for path in image_paths:
        image = load_image(path)
        detections = detect_tags(image, layout.family)
        pose = estimate_camera_pose(layout, detections, camera)
        overlay = draw_detections(image, detections)
        overlay = draw_pose_axes(overlay, camera, pose, layout.nominal_tag_size_m)
        cv2.imwrite(str(annotate_dir / f"{path.stem}_{label}.png"), overlay)
        row: dict[str, Any] = {
            "image": str(path),
            "detected_tag_count": len(detections),
            "view": view_metadata[path.name],
            "pose": pose.to_dict(),
        }
        if pose.success and pose.t_cw is not None:
            row.update(_pose_error(pose.t_cw, true_poses[path.name]))
        rows.append(row)
    return rows


def _summarize_detection_quality(rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for role in ("calibration", "evaluation", "overall"):
        subset = rows if role == "overall" else [row for row in rows if row["role"] == role]
        expected = sum(int(row["expected_tag_count"]) for row in subset)
        detected = sum(int(row["detected_expected_tag_count"]) for row in subset)
        missed = sum(int(row["missed_tag_count"]) for row in subset)
        false_count = sum(int(row["false_detection_count"]) for row in subset)
        raw_detected = sum(int(row["raw_detected_tag_count"]) for row in subset)
        summary[role] = {
            "frame_count": len(subset),
            "expected_tag_count": expected,
            "detected_expected_tag_count": detected,
            "missed_tag_count": missed,
            "false_detection_count": false_count,
            "miss_rate": missed / expected if expected else 0.0,
            "false_detection_rate": false_count / raw_detected if raw_detected else 0.0,
        }
    return summary


def _detection_quality_rows(image_paths: list[Path], view_metadata: dict[str, dict[str, Any]], layout) -> list[dict[str, Any]]:
    all_ids = set(layout.tag_ids)
    rows: list[dict[str, Any]] = []
    for path in image_paths:
        metadata = view_metadata[path.name]
        hidden_ids = set(int(tag_id) for tag_id in metadata.get("hidden_tag_ids", []))
        expected_ids = all_ids - hidden_ids
        detected_ids = {det.tag_id for det in detect_tags(load_image(path), layout.family)}
        missed_ids = sorted(expected_ids - detected_ids)
        false_ids = sorted(detected_ids - expected_ids)
        rows.append(
            {
                "image": str(path),
                "role": metadata["role"],
                "expected_tag_count": len(expected_ids),
                "raw_detected_tag_count": len(detected_ids),
                "detected_expected_tag_count": len(detected_ids & expected_ids),
                "missed_tag_count": len(missed_ids),
                "false_detection_count": len(false_ids),
                "miss_rate": len(missed_ids) / len(expected_ids) if expected_ids else 0.0,
                "false_detection_rate": len(false_ids) / len(detected_ids) if detected_ids else 0.0,
                "missed_tag_ids": missed_ids,
                "false_detection_tag_ids": false_ids,
            }
        )
    return rows


def _write_layout_preview(path: Path, layout, paper_size_m: tuple[float, float]) -> None:
    width = 1200
    height = int(round(width * layout.paper_height_m / layout.paper_width_m))
    image = render_board_texture(
        layout,
        width_px=width,
        background=(178, 178, 168),
        per_tag_paper_size_m=paper_size_m,
    )
    cv2.imwrite(str(path), image)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out_dir = ensure_dir(args.output_dir)
    layout_dir = ensure_dir(out_dir / "layout")
    image_dir = ensure_dir(out_dir / "images")
    annotate_dir = ensure_dir(out_dir / "annotated")

    layout = create_irregular_table_layout(
        args.tag_count,
        tag_size_m=args.tag_size_m,
        table_size_m=(args.table_width_m, args.table_height_m),
        seed=args.seed,
    )
    layout_path = save_board_layout(layout_dir / "table_layout.json", layout)
    paper_size_m = (args.paper_width_m, args.paper_height_m)
    _write_layout_preview(layout_dir / "table_layout_preview.png", layout, paper_size_m)

    image_size = (args.image_width, args.image_height)
    camera = default_camera(image_size, args.focal_px)
    camera_path = save_camera(out_dir / "synthetic_camera.json", camera)
    texture = render_board_texture(
        layout,
        width_px=3600,
        background=(178, 178, 168),
        per_tag_paper_size_m=paper_size_m,
    )

    calibration_specs = _calibration_specs()
    if args.calibration_views > len(calibration_specs):
        raise ValueError(f"calibration-views can be at most {len(calibration_specs)}")
    view_specs = [
        {"role": "calibration", **spec}
        for spec in calibration_specs[: args.calibration_views]
    ] + [
        {"role": "evaluation", **spec}
        for spec in _evaluation_specs()
    ]

    calibration_paths: list[Path] = []
    eval_paths: list[Path] = []
    true_poses: dict[str, np.ndarray] = {}
    view_metadata: dict[str, dict[str, Any]] = {}

    for idx, spec in enumerate(view_specs):
        eye, rvec, tvec, table_area_ratio = _solve_oblique_pose_for_area(
            layout,
            camera,
            image_size,
            spec["tilt_deg"],
            spec["azimuth_deg"],
            spec["target_area_ratio"],
            target=spec.get("target", (0.0, 0.0, 0.0)),
        )
        hidden_ids = _hidden_tag_ids(layout, spec.get("hide", []))
        occlusion_polygons = _occlusion_polygons_for_tags(layout, camera, rvec, tvec, hidden_ids)
        name = f"view_{idx:02d}_{spec['role']}.png"
        path = image_dir / name
        image = render_view(
            layout,
            camera,
            rvec,
            tvec,
            texture=texture,
            image_size=image_size,
            background=190,
            occlusion_polygons=occlusion_polygons,
            blur=spec.get("blur", 0.0),
            noise_std=spec.get("noise_std", 0.0),
        )
        cv2.imwrite(str(path), image)
        true_poses[name] = rvec_tvec_to_matrix(rvec, tvec)
        view_metadata[name] = {
            "role": spec["role"],
            "tilt_deg": spec["tilt_deg"],
            "azimuth_deg": spec["azimuth_deg"],
            "target_area_ratio": spec["target_area_ratio"],
            "table_area_ratio": table_area_ratio,
            "camera_distance_m": float(np.linalg.norm(eye - np.asarray(spec.get("target", (0.0, 0.0, 0.0))))),
            "hidden_tag_ids": hidden_ids,
            "noise_std": spec.get("noise_std", 0.0),
        }
        if spec["role"] == "calibration":
            calibration_paths.append(path)
        else:
            eval_paths.append(path)

    known_intrinsics_rows = _localize_eval_images(
        eval_paths,
        true_poses,
        view_metadata,
        layout,
        camera,
        annotate_dir,
        "known_intrinsics",
    )

    calibration = calibrate_from_images(
        layout,
        calibration_paths,
        count=args.calibration_views,
        min_views=args.min_calibration_views,
    )
    self_calib_rows: list[dict[str, Any]] = []
    if calibration.success and calibration.camera is not None:
        self_calib_rows = _localize_eval_images(
            eval_paths,
            true_poses,
            view_metadata,
            layout,
            calibration.camera,
            annotate_dir,
            "self_calibrated",
        )

    detection_quality = _detection_quality_rows(calibration_paths + eval_paths, view_metadata, layout)
    metrics = {
        "success": all(row["pose"]["success"] for row in known_intrinsics_rows)
        and (calibration.success and all(row["pose"]["success"] for row in self_calib_rows)),
        "layout_json": str(layout_path),
        "camera_json": str(camera_path),
        "tag_size_m": args.tag_size_m,
        "table_size_m": [args.table_width_m, args.table_height_m],
        "paper_size_m": [args.paper_width_m, args.paper_height_m],
        "view_metadata": view_metadata,
        "known_intrinsics": known_intrinsics_rows,
        "self_calibration": calibration.to_dict(),
        "self_calibrated": self_calib_rows,
        "detection_quality": detection_quality,
        "detection_summary": _summarize_detection_quality(detection_quality),
    }
    write_json(out_dir / "metrics.json", metrics)

    known_success = sum(row["pose"]["success"] for row in known_intrinsics_rows)
    self_success = sum(row["pose"]["success"] for row in self_calib_rows)
    print(f"wrote {out_dir / 'metrics.json'}")
    print(f"known-intrinsics frames: {known_success}/{len(known_intrinsics_rows)}")
    print(f"self-calibration: {calibration.reason}")
    if calibration.rms_px is not None:
        print(f"self-calibration RMS: {calibration.rms_px:.4f}px")
    if self_calib_rows:
        print(f"self-calibrated frames: {self_success}/{len(self_calib_rows)}")
    return 0 if metrics["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
