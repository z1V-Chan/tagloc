from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares

from src.board import DEFAULT_FAMILY, BoardLayout, create_layout_from_placements, save_board_layout
from src.detection import TagDetection, detect_tags, load_image
from src.pose import estimate_camera_pose, reprojection_errors
from utils.camera import CameraModel, load_camera, save_camera
from utils.geometry import invert_transform, rvec_tvec_to_matrix
from utils.io import expand_image_paths, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate an irregular coplanar AprilTag table layout from undistorted images."
    )
    parser.add_argument("images", nargs="+", help="Calibration image files, directories, or shell globs.")
    parser.add_argument("--tag-size-m", type=float, required=True, help="Measured tag edge size in meters.")
    parser.add_argument("--family", default=DEFAULT_FAMILY, help="OpenCV AprilTag dictionary name.")
    parser.add_argument("--camera", default=None, help="Optional known camera JSON/YAML.")
    parser.add_argument("--calibration-count", type=int, default=10, help="Number of leading images to use.")
    parser.add_argument("--min-camera-observations", type=int, default=20)
    parser.add_argument("--output", required=True, help="Output estimated layout JSON.")
    parser.add_argument("--save-estimated-camera", default=None, help="Optional camera JSON when --camera is omitted.")
    parser.add_argument("--summary", default=None, help="Optional estimation summary JSON.")
    parser.add_argument("--anchor-id", type=int, default=None, help="Tag id used as the estimated layout origin.")
    parser.add_argument("--table-width-m", type=float, default=None, help="Optional layout canvas width.")
    parser.add_argument("--table-height-m", type=float, default=None, help="Optional layout canvas height.")
    parser.add_argument("--max-optimizer-iterations", type=int, default=200)
    parser.add_argument("--no-joint-optimization", action="store_true", help="Only run the single-tag bootstrap estimator.")
    parser.add_argument(
        "--refine-known-camera",
        action="store_true",
        help="Also refine intrinsics when --camera is provided. Disabled by default.",
    )
    return parser.parse_args()


def _tag_object_corners(tag_size_m: float) -> np.ndarray:
    half = float(tag_size_m) * 0.5
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def _calibrate_camera_from_tags(
    image_paths: list[Path],
    family: str,
    tag_size_m: float,
    min_observations: int,
) -> tuple[CameraModel, dict[str, Any]] | tuple[None, dict[str, Any]]:
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    object_corners = _tag_object_corners(tag_size_m).astype(np.float32)
    observation_images: list[str] = []

    for path in image_paths:
        image = load_image(path)
        h, w = image.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            return None, {"success": False, "reason": "all calibration images must have the same size"}
        for detection in detect_tags(image, family):
            object_points.append(object_corners)
            image_points.append(detection.corners.astype(np.float32))
            observation_images.append(str(path))

    if image_size is None:
        return None, {"success": False, "reason": "no calibration images were readable"}
    if len(object_points) < min_observations:
        return None, {
            "success": False,
            "reason": f"need at least {min_observations} tag observations, got {len(object_points)}",
            "observation_count": len(object_points),
        }

    flags = (
        cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K1
        | cv2.CALIB_FIX_K2
        | cv2.CALIB_FIX_K3
    )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    try:
        rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
            flags=flags,
            criteria=criteria,
        )
    except cv2.error as exc:
        return None, {"success": False, "reason": f"calibrateCamera failed: {exc}"}

    camera = CameraModel(camera_matrix, dist_coeffs, image_size)
    return camera, {
        "success": True,
        "reason": "calibrated_from_individual_tags",
        "rms_px": float(rms),
        "observation_count": len(object_points),
        "used_images": sorted(set(observation_images)),
        "camera": camera.to_dict(),
    }


def _tag_pose_from_detection(
    detection: TagDetection,
    camera: CameraModel,
    object_corners: np.ndarray,
) -> tuple[np.ndarray, float] | None:
    try:
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_corners,
            detection.corners.astype(np.float64),
            camera.camera_matrix,
            camera.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
    except cv2.error:
        return None
    if not ok or len(rvecs) == 0:
        return None

    best: tuple[float, np.ndarray] | None = None
    for rvec, tvec in zip(rvecs, tvecs):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if float(tvec[2, 0]) <= 0:
            continue
        errors = reprojection_errors(object_corners, detection.corners, camera, rvec, tvec)
        rms = float(np.sqrt(np.mean(errors**2)))
        if best is None or rms < best[0]:
            best = (rms, rvec_tvec_to_matrix(rvec, tvec))
    return best


def _yaw_from_transform(transform: np.ndarray) -> float:
    return math.degrees(math.atan2(float(transform[1, 0]), float(transform[0, 0])))


def _average_samples(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    arr = np.asarray([[x, y] for x, y, _ in samples], dtype=np.float64)
    yaws = np.radians([yaw for _, _, yaw in samples])
    yaw = math.degrees(math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws)))))
    xy = np.median(arr, axis=0)
    return float(xy[0]), float(xy[1]), float(yaw)


def _tag_yaw_deg_from_corners(corners: np.ndarray) -> float:
    x_axis = np.asarray(corners, dtype=np.float64)[1] - np.asarray(corners, dtype=np.float64)[0]
    return math.degrees(math.atan2(float(x_axis[1]), float(x_axis[0])))


def _corners_from_xy_yaw_rad(x_m: float, y_m: float, yaw_rad: float, tag_size_m: float) -> np.ndarray:
    half = float(tag_size_m) * 0.5
    x_axis = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0], dtype=np.float64)
    y_axis = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0], dtype=np.float64)
    center = np.array([x_m, y_m, 0.0], dtype=np.float64)
    return np.array(
        [
            center - half * x_axis + half * y_axis,
            center + half * x_axis + half * y_axis,
            center + half * x_axis - half * y_axis,
            center - half * x_axis - half * y_axis,
        ],
        dtype=np.float64,
    )


def _camera_from_params(
    camera_params: np.ndarray | None,
    base_camera: CameraModel,
) -> CameraModel:
    if camera_params is None:
        return base_camera
    fx, fy, cx, cy = [float(v) for v in camera_params]
    camera_matrix = np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return CameraModel(camera_matrix, np.zeros(5, dtype=np.float64), base_camera.image_size)


def _joint_optimize_layout(
    image_paths: list[Path],
    family: str,
    tag_size_m: float,
    initial_layout: BoardLayout,
    initial_camera: CameraModel,
    anchor_id: int,
    table_size_m: tuple[float, float] | None,
    optimize_camera: bool,
    max_nfev: int,
) -> tuple[BoardLayout, CameraModel, dict[str, Any]]:
    tag_by_id = initial_layout.tag_by_id()
    tag_ids = sorted(tag_by_id)
    if anchor_id not in tag_by_id:
        raise ValueError(f"anchor tag {anchor_id} is missing from the initial layout")

    observations: list[tuple[int, int, np.ndarray]] = []
    view_paths: list[str] = []
    initial_view_params: list[np.ndarray] = []
    for path in image_paths:
        detections = [det for det in detect_tags(load_image(path), family) if det.tag_id in tag_by_id]
        if not detections:
            continue
        pose = estimate_camera_pose(initial_layout, detections, initial_camera, max_reprojection_error_px=20.0)
        if not pose.success or pose.rvec is None or pose.tvec is None:
            continue
        view_index = len(view_paths)
        view_paths.append(str(path))
        initial_view_params.append(np.r_[pose.rvec.reshape(3), pose.tvec.reshape(3)])
        for detection in detections:
            observations.append((view_index, detection.tag_id, detection.corners.astype(np.float64)))

    if not observations:
        raise ValueError("no usable multi-tag observations for joint optimization")

    optimized_tag_ids = [tag_id for tag_id in tag_ids if tag_id != anchor_id]
    tag_initial: list[float] = []
    for tag_id in optimized_tag_ids:
        tag = tag_by_id[tag_id]
        tag_initial.extend(
            [
                float(tag.center[0]),
                float(tag.center[1]),
                math.radians(_tag_yaw_deg_from_corners(tag.corners)),
            ]
        )

    camera_initial = np.array(
        [
            initial_camera.fx,
            initial_camera.fy,
            initial_camera.cx,
            initial_camera.cy,
        ],
        dtype=np.float64,
    )
    chunks: list[np.ndarray] = []
    lower_chunks: list[np.ndarray] = []
    upper_chunks: list[np.ndarray] = []
    if optimize_camera:
        chunks.append(camera_initial)
        width, height = initial_camera.image_size or (max(initial_camera.cx * 2.0, 1.0), max(initial_camera.cy * 2.0, 1.0))
        lower_chunks.append(np.array([100.0, 100.0, 0.0, 0.0], dtype=np.float64))
        upper_chunks.append(np.array([10000.0, 10000.0, float(width), float(height)], dtype=np.float64))
    view_initial = np.concatenate(initial_view_params).astype(np.float64)
    chunks.append(view_initial)
    lower_chunks.append(np.full(view_initial.shape, -np.inf, dtype=np.float64))
    upper_chunks.append(np.full(view_initial.shape, np.inf, dtype=np.float64))
    tag_initial_arr = np.asarray(tag_initial, dtype=np.float64)
    chunks.append(tag_initial_arr)
    lower_chunks.append(np.full(tag_initial_arr.shape, -np.inf, dtype=np.float64))
    upper_chunks.append(np.full(tag_initial_arr.shape, np.inf, dtype=np.float64))

    x0 = np.concatenate(chunks)
    lower = np.concatenate(lower_chunks)
    upper = np.concatenate(upper_chunks)

    view_param_count = len(view_paths) * 6

    def unpack(params: np.ndarray) -> tuple[CameraModel, np.ndarray, dict[int, tuple[float, float, float]]]:
        offset = 0
        camera_params = None
        if optimize_camera:
            camera_params = params[offset : offset + 4]
            offset += 4
        camera = _camera_from_params(camera_params, initial_camera)
        view_params = params[offset : offset + view_param_count].reshape(len(view_paths), 6)
        offset += view_param_count
        tag_params = {anchor_id: (0.0, 0.0, 0.0)}
        raw_tags = params[offset:].reshape(len(optimized_tag_ids), 3) if optimized_tag_ids else np.empty((0, 3))
        for tag_id, row in zip(optimized_tag_ids, raw_tags):
            tag_params[tag_id] = (float(row[0]), float(row[1]), float(row[2]))
        return camera, view_params, tag_params

    def residuals(params: np.ndarray) -> np.ndarray:
        camera, view_params, tag_params = unpack(params)
        residual_chunks: list[np.ndarray] = []
        for view_index, tag_id, image_corners in observations:
            rvec = view_params[view_index, :3].reshape(3, 1)
            tvec = view_params[view_index, 3:].reshape(3, 1)
            x_m, y_m, yaw_rad = tag_params[tag_id]
            object_corners = _corners_from_xy_yaw_rad(x_m, y_m, yaw_rad, tag_size_m)
            projected, _ = cv2.projectPoints(
                object_corners,
                rvec,
                tvec,
                camera.camera_matrix,
                camera.dist_coeffs,
            )
            residual_chunks.append((projected.reshape(4, 2) - image_corners).reshape(-1))
        return np.concatenate(residual_chunks)

    initial_residuals = residuals(x0)
    initial_rms = float(np.sqrt(np.mean(initial_residuals**2)))
    result = least_squares(
        residuals,
        x0,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=2.0,
        max_nfev=max_nfev,
        x_scale="jac",
        verbose=0,
    )
    final_residuals = residuals(result.x)
    final_rms = float(np.sqrt(np.mean(final_residuals**2)))
    optimized_camera, _, tag_params = unpack(result.x)
    placements = [
        (tag_id, x_m, y_m, math.degrees(yaw_rad))
        for tag_id, (x_m, y_m, yaw_rad) in sorted(tag_params.items())
    ]
    optimized_layout = create_layout_from_placements(
        placements,
        tag_size_m=tag_size_m,
        family=family,
        table_size_m=table_size_m,
    )
    summary = {
        "success": bool(result.success),
        "reason": str(result.message),
        "optimized_intrinsics": bool(optimize_camera),
        "view_count": len(view_paths),
        "observation_count": len(observations),
        "residual_count": int(len(final_residuals)),
        "initial_reprojection_rms_px": initial_rms,
        "final_reprojection_rms_px": final_rms,
        "optimizer_nfev": int(result.nfev),
        "optimizer_cost": float(result.cost),
        "used_images": view_paths,
    }
    return optimized_layout, optimized_camera, summary


def _estimate_layout(
    image_paths: list[Path],
    family: str,
    tag_size_m: float,
    camera: CameraModel,
    anchor_id: int | None,
    table_size_m: tuple[float, float] | None,
) -> tuple[BoardLayout, dict[str, Any]]:
    object_corners = _tag_object_corners(tag_size_m)
    view_poses: list[dict[int, np.ndarray]] = []
    pose_rms_rows: list[dict[str, Any]] = []
    observed_ids: set[int] = set()

    for path in image_paths:
        detections = detect_tags(load_image(path), family)
        poses: dict[int, np.ndarray] = {}
        for detection in detections:
            pose = _tag_pose_from_detection(detection, camera, object_corners)
            if pose is None:
                continue
            rms, transform = pose
            poses[detection.tag_id] = transform
            observed_ids.add(detection.tag_id)
            pose_rms_rows.append({"image": str(path), "id": detection.tag_id, "single_tag_pose_rms_px": rms})
        if poses:
            view_poses.append(poses)

    if not observed_ids:
        raise ValueError("no tag poses could be estimated")
    anchor = int(anchor_id) if anchor_id is not None else min(observed_ids)
    if anchor not in observed_ids:
        raise ValueError(f"anchor tag {anchor} was not observed")

    samples_by_id: dict[int, list[tuple[float, float, float]]] = {anchor: [(0.0, 0.0, 0.0)]}
    for poses in view_poses:
        if anchor not in poses:
            continue
        t_anchor_camera = invert_transform(poses[anchor])
        for tag_id, t_camera_tag in poses.items():
            t_anchor_tag = t_anchor_camera @ t_camera_tag
            samples_by_id.setdefault(tag_id, []).append(
                (
                    float(t_anchor_tag[0, 3]),
                    float(t_anchor_tag[1, 3]),
                    _yaw_from_transform(t_anchor_tag),
                )
            )

    if set(samples_by_id) != observed_ids:
        known_ids = set(samples_by_id)
        progress = True
        while progress and known_ids != observed_ids:
            progress = False
            averaged = {tag_id: _average_samples(samples) for tag_id, samples in samples_by_id.items()}
            transforms = {}
            for tag_id, (x_m, y_m, yaw_deg) in averaged.items():
                yaw = math.radians(yaw_deg)
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :3] = np.array(
                    [
                        [math.cos(yaw), -math.sin(yaw), 0.0],
                        [math.sin(yaw), math.cos(yaw), 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                )
                transform[:3, 3] = [x_m, y_m, 0.0]
                transforms[tag_id] = transform
            for poses in view_poses:
                known_in_view = [tag_id for tag_id in poses if tag_id in known_ids]
                if not known_in_view:
                    continue
                source_id = known_in_view[0]
                t_source_camera = invert_transform(poses[source_id])
                for tag_id, t_camera_tag in poses.items():
                    if tag_id in known_ids:
                        continue
                    t_source_tag = t_source_camera @ t_camera_tag
                    t_anchor_tag = transforms[source_id] @ t_source_tag
                    samples_by_id.setdefault(tag_id, []).append(
                        (
                            float(t_anchor_tag[0, 3]),
                            float(t_anchor_tag[1, 3]),
                            _yaw_from_transform(t_anchor_tag),
                        )
                    )
                    known_ids.add(tag_id)
                    progress = True

    placements = [
        (tag_id, *_average_samples(samples))
        for tag_id, samples in sorted(samples_by_id.items())
    ]
    layout = create_layout_from_placements(
        placements,
        tag_size_m=tag_size_m,
        family=family,
        table_size_m=table_size_m,
    )
    object_count = len(pose_rms_rows)
    mean_single_tag_rms = float(np.mean([row["single_tag_pose_rms_px"] for row in pose_rms_rows])) if pose_rms_rows else None
    summary = {
        "success": True,
        "anchor_id": anchor,
        "observed_tag_count": len(observed_ids),
        "estimated_tag_count": len(layout.tags),
        "used_image_count": len(image_paths),
        "single_tag_pose_count": object_count,
        "mean_single_tag_pose_rms_px": mean_single_tag_rms,
        "tag_sample_counts": {str(tag_id): len(samples) for tag_id, samples in sorted(samples_by_id.items())},
    }
    return layout, summary


def _table_size_from_args(args: argparse.Namespace) -> tuple[float, float] | None:
    if args.table_width_m is None and args.table_height_m is None:
        return None
    if args.table_width_m is None or args.table_height_m is None:
        raise ValueError("--table-width-m and --table-height-m must be provided together")
    return (float(args.table_width_m), float(args.table_height_m))


def main() -> int:
    args = parse_args()
    image_paths = expand_image_paths(args.images)[: args.calibration_count]
    if not image_paths:
        print("no image files found", file=sys.stderr)
        return 2
    if args.tag_size_m <= 0:
        raise ValueError("--tag-size-m must be positive")

    calibration_summary = None
    if args.camera:
        camera = load_camera(args.camera)
    else:
        camera, calibration_summary = _calibrate_camera_from_tags(
            image_paths,
            args.family,
            args.tag_size_m,
            args.min_camera_observations,
        )
        if camera is None:
            result = {"success": False, "reason": "camera calibration failed", "calibration": calibration_summary}
            if args.summary:
                write_json(args.summary, result)
            print(result, file=sys.stderr)
            return 1
        if args.save_estimated_camera:
            save_camera(args.save_estimated_camera, camera)

    table_size_m = _table_size_from_args(args)
    layout, layout_summary = _estimate_layout(
        image_paths,
        args.family,
        args.tag_size_m,
        camera,
        args.anchor_id,
        table_size_m,
    )
    joint_optimization_summary = None
    if not args.no_joint_optimization:
        anchor_id = int(layout_summary["anchor_id"])
        layout, camera, joint_optimization_summary = _joint_optimize_layout(
            image_paths,
            args.family,
            args.tag_size_m,
            layout,
            camera,
            anchor_id,
            table_size_m,
            optimize_camera=(not args.camera) or args.refine_known_camera,
            max_nfev=args.max_optimizer_iterations,
        )
    layout_path = save_board_layout(args.output, layout)
    if args.save_estimated_camera:
        save_camera(args.save_estimated_camera, camera)
    summary = {
        "success": True,
        "layout": str(layout_path),
        "camera": camera.to_dict(),
        "calibration": calibration_summary,
        "layout_estimation": layout_summary,
        "joint_optimization": joint_optimization_summary,
    }
    if args.summary:
        write_json(args.summary, summary)
    print(f"layout: {layout_path}")
    if args.save_estimated_camera:
        print(f"camera: {Path(args.save_estimated_camera)}")
    if args.summary:
        print(f"summary: {Path(args.summary)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
