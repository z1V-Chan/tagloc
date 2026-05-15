from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.board import create_default_layout, save_board_layout, save_board_pdf_and_preview
from src.calibration import calibrate_from_images
from src.detection import detect_tags, draw_detections, load_image
from src.pose import draw_pose_axes, estimate_camera_pose
from src.render import (
    default_camera,
    look_at_tcw,
    paper_corners_world,
    project_points,
    render_board_texture,
    render_view,
)
from utils.camera import save_camera
from utils.geometry import invert_transform, rotation_angle_deg, rvec_tvec_to_matrix
from utils.io import ensure_dir, write_json


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render synthetic AprilTag-board images and validate localization.")
    parser.add_argument("--output-dir", default="outputs/render_test")
    parser.add_argument("--nominal-tag-size-m", type=float, default=0.045)
    parser.add_argument("--actual-tag-size-m", type=float, default=0.0465)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=900)
    parser.add_argument("--focal-px", type=float, default=950.0)
    parser.add_argument("--calibration-views", type=int, default=10)
    parser.add_argument("--min-calibration-views", type=int, default=5)
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


def _projected_paper_area_ratio(layout, camera, rvec, tvec, image_size: tuple[int, int]) -> float:
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
    lo = 0.05
    hi = 5.0
    best = None
    for _ in range(80):
        distance = (lo + hi) * 0.5
        eye = _eye_from_oblique_view(target_w, distance, tilt_deg, azimuth_deg)
        rvec, tvec = look_at_tcw(eye, target_w)
        area_ratio = _projected_paper_area_ratio(layout, camera, rvec, tvec, image_size)
        best = (eye, rvec, tvec, area_ratio)
        if area_ratio > target_area_ratio:
            lo = distance
        else:
            hi = distance
    assert best is not None
    return best


def _calibration_specs() -> list[dict[str, Any]]:
    return [
        {"tilt_deg": 45.0, "azimuth_deg": 0.0, "target_area_ratio": 1 / 9.4, "noise_std": 1.0},
        {"tilt_deg": 50.0, "azimuth_deg": 35.0, "target_area_ratio": 1 / 10.2, "noise_std": 1.0},
        {"tilt_deg": 55.0, "azimuth_deg": 72.0, "target_area_ratio": 1 / 9.8, "noise_std": 1.2},
        {"tilt_deg": 60.0, "azimuth_deg": 112.0, "target_area_ratio": 1 / 10.6, "noise_std": 1.2},
        {"tilt_deg": 65.0, "azimuth_deg": 150.0, "target_area_ratio": 1 / 9.7, "noise_std": 1.5},
        {"tilt_deg": 70.0, "azimuth_deg": 205.0, "target_area_ratio": 1 / 10.8, "noise_std": 1.5},
        {"tilt_deg": 75.0, "azimuth_deg": 252.0, "target_area_ratio": 1 / 10.5, "noise_std": 1.8},
        {"tilt_deg": 48.0, "azimuth_deg": 294.0, "target_area_ratio": 1 / 9.9, "noise_std": 1.0},
        {"tilt_deg": 58.0, "azimuth_deg": 330.0, "target_area_ratio": 1 / 10.1, "noise_std": 1.2},
        {"tilt_deg": 68.0, "azimuth_deg": 25.0, "target_area_ratio": 1 / 10.9, "noise_std": 1.5},
        {"tilt_deg": 73.0, "azimuth_deg": 315.0, "target_area_ratio": 1 / 10.4, "noise_std": 1.8},
        {"tilt_deg": 62.0, "azimuth_deg": 185.0, "target_area_ratio": 1 / 9.6, "noise_std": 1.4},
    ]


def _irregular_occlusions(index: int) -> list[list[tuple[float, float]]]:
    patterns = [
        [
            [(0.42, 0.34), (0.56, 0.38), (0.53, 0.53), (0.39, 0.50)],
            [(0.50, 0.66), (0.62, 0.70), (0.60, 0.82), (0.47, 0.77)],
        ],
        [
            [(0.45, 0.18), (0.60, 0.24), (0.57, 0.37), (0.43, 0.33)],
            [(0.36, 0.55), (0.49, 0.50), (0.52, 0.64), (0.39, 0.70)],
        ],
        [
            [(0.54, 0.28), (0.67, 0.35), (0.61, 0.50), (0.49, 0.45)],
            [(0.40, 0.73), (0.55, 0.68), (0.58, 0.82), (0.45, 0.88)],
        ],
        [
            [(0.37, 0.30), (0.50, 0.25), (0.55, 0.39), (0.42, 0.45)],
            [(0.55, 0.56), (0.67, 0.61), (0.63, 0.75), (0.50, 0.69)],
        ],
        [
            [(0.44, 0.42), (0.59, 0.39), (0.63, 0.53), (0.47, 0.59)],
        ],
        [
            [(0.49, 0.13), (0.62, 0.21), (0.57, 0.34), (0.43, 0.26)],
            [(0.41, 0.58), (0.54, 0.61), (0.50, 0.74), (0.36, 0.70)],
        ],
        [
            [(0.46, 0.24), (0.58, 0.28), (0.55, 0.43), (0.42, 0.39)],
            [(0.52, 0.62), (0.65, 0.68), (0.60, 0.81), (0.48, 0.76)],
        ],
        [
            [(0.40, 0.47), (0.54, 0.42), (0.59, 0.56), (0.46, 0.64)],
        ],
    ]
    return patterns[index % len(patterns)]


def _evaluation_specs() -> list[dict[str, Any]]:
    tilts = [45.0, 52.0, 60.0, 67.0, 72.0, 75.0, 78.0, 80.0]
    azimuths = [15.0, 75.0, 135.0, 195.0, 255.0, 315.0, 45.0, 225.0]
    target_area_ratios = [1 / 9.2, 1 / 10.0, 1 / 10.8, 1 / 9.7, 1 / 10.6, 1 / 9.9, 1 / 11.0, 1 / 11.0]
    specs: list[dict[str, Any]] = []
    for idx, (tilt, azimuth, area_ratio) in enumerate(zip(tilts, azimuths, target_area_ratios)):
        specs.append(
            {
                "tilt_deg": tilt,
                "azimuth_deg": azimuth,
                "target_area_ratio": area_ratio,
                "occlusion_polygons": _irregular_occlusions(idx),
                "blur": 1.0 + 0.15 * (idx % 3),
                "noise_std": 2.0 + 0.5 * (idx % 2),
            }
        )
    return specs


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


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out_dir = ensure_dir(args.output_dir)
    board_dir = ensure_dir(out_dir / "board")
    image_dir = ensure_dir(out_dir / "images")
    annotate_dir = ensure_dir(out_dir / "annotated")

    nominal_layout = create_default_layout(tag_size_m=args.nominal_tag_size_m)
    actual_layout = nominal_layout.scaled_to_tag_size(args.actual_tag_size_m)
    layout_path = save_board_layout(board_dir / "board_layout.json", nominal_layout)
    save_board_pdf_and_preview(nominal_layout, board_dir / "a4_apriltag_board.pdf", board_dir / "a4_apriltag_board.png")

    camera = default_camera((args.image_width, args.image_height), args.focal_px)
    camera_path = save_camera(out_dir / "synthetic_camera.json", camera)
    texture = render_board_texture(actual_layout)

    calibration_specs = _calibration_specs()
    if args.calibration_views > len(calibration_specs):
        raise ValueError(f"calibration-views can be at most {len(calibration_specs)} for this render test")
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
    image_size = (args.image_width, args.image_height)
    for idx, spec in enumerate(view_specs):
        eye, rvec, tvec, paper_area_ratio = _solve_oblique_pose_for_area(
            actual_layout,
            camera,
            image_size,
            spec["tilt_deg"],
            spec["azimuth_deg"],
            spec["target_area_ratio"],
        )
        name = f"view_{idx:02d}.png"
        path = image_dir / name
        image = render_view(
            actual_layout,
            camera,
            rvec,
            tvec,
            texture=texture,
            image_size=image_size,
            occlusion_polygons=spec.get("occlusion_polygons"),
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
            "paper_area_ratio": paper_area_ratio,
            "camera_distance_m": float(np.linalg.norm(eye)),
            "occlusion_polygon_count": len(spec.get("occlusion_polygons") or []),
            "blur": spec.get("blur", 0.0),
            "noise_std": spec.get("noise_std", 0.0),
        }
        if spec["role"] == "calibration":
            calibration_paths.append(path)
        else:
            eval_paths.append(path)

    measured_layout = nominal_layout.scaled_to_tag_size(args.actual_tag_size_m)
    known_intrinsics_rows = _localize_eval_images(
        eval_paths,
        true_poses,
        view_metadata,
        measured_layout,
        camera,
        annotate_dir,
        "known_intrinsics",
    )

    calibration = calibrate_from_images(
        measured_layout,
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
            measured_layout,
            calibration.camera,
            annotate_dir,
            "self_calibrated",
        )

    metrics = {
        "success": all(row["pose"]["success"] for row in known_intrinsics_rows)
        and (calibration.success and all(row["pose"]["success"] for row in self_calib_rows)),
        "layout_json": str(layout_path),
        "camera_json": str(camera_path),
        "nominal_tag_size_m": args.nominal_tag_size_m,
        "actual_measured_tag_size_m": args.actual_tag_size_m,
        "view_metadata": view_metadata,
        "known_intrinsics": known_intrinsics_rows,
        "self_calibration": calibration.to_dict(),
        "self_calibrated": self_calib_rows,
    }
    write_json(out_dir / "metrics.json", metrics)
    print(f"wrote {out_dir / 'metrics.json'}")
    print(f"known-intrinsics frames: {sum(row['pose']['success'] for row in known_intrinsics_rows)}/{len(known_intrinsics_rows)}")
    print(f"self-calibration: {calibration.reason}")
    if calibration.rms_px is not None:
        print(f"self-calibration RMS: {calibration.rms_px:.4f}px")
    return 0 if metrics["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
