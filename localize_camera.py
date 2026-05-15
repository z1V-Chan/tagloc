from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

from src.board import load_board_layout
from src.calibration import calibrate_from_images
from src.detection import detect_tags, draw_detections, load_image
from src.pose import draw_pose_axes, estimate_camera_pose
from utils.camera import load_camera, save_camera
from utils.io import ensure_dir, expand_image_paths, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera pose from an A4 AprilTag board.")
    parser.add_argument("images", nargs="+", help="Image files, directories, or shell globs.")
    parser.add_argument("--board", default="outputs/board/board_layout.json", help="Board layout JSON.")
    parser.add_argument("--tag-size-m", type=float, default=None, help="Measured printed tag edge size in meters.")
    parser.add_argument("--camera", default=None, help="JSON/YAML camera config. If omitted, calibrate from first N images.")
    parser.add_argument("--calibration-count", type=int, default=10, help="Number of leading images for no-intrinsics mode.")
    parser.add_argument("--min-calibration-views", type=int, default=5)
    parser.add_argument("--output", default=None, help="Output JSON path. Defaults to stdout.")
    parser.add_argument("--annotate-dir", default=None, help="Optional directory for detection and pose overlays.")
    parser.add_argument("--save-estimated-camera", default=None, help="Optional path for estimated camera JSON.")
    parser.add_argument("--ransac-error-px", type=float, default=4.0)
    parser.add_argument("--max-reprojection-error-px", type=float, default=8.0)
    parser.add_argument("--axis-length-m", type=float, default=0.05)
    return parser.parse_args()


def localize_one(
    image_path: Path,
    layout,
    camera,
    annotate_dir: Path | None,
    ransac_error_px: float,
    max_reprojection_error_px: float,
    axis_length_m: float,
) -> dict:
    image = load_image(image_path)
    detections = detect_tags(image, layout.family)
    pose = estimate_camera_pose(
        layout,
        detections,
        camera,
        ransac_reprojection_error_px=ransac_error_px,
        max_reprojection_error_px=max_reprojection_error_px,
    )
    if annotate_dir is not None:
        overlay = draw_detections(image, detections)
        overlay = draw_pose_axes(overlay, camera, pose, axis_length_m)
        cv2.imwrite(str(annotate_dir / f"{image_path.stem}_pose.png"), overlay)
    return {
        "image": str(image_path),
        "detections": [det.to_dict() for det in detections],
        "pose": pose.to_dict(),
    }


def main() -> int:
    args = parse_args()
    image_paths = expand_image_paths(args.images)
    if not image_paths:
        print("no image files found", file=sys.stderr)
        return 2

    layout = load_board_layout(args.board).scaled_to_tag_size(args.tag_size_m)
    measured_tag_size = layout.nominal_tag_size_m
    annotate_dir = ensure_dir(args.annotate_dir) if args.annotate_dir else None

    calibration = None
    if args.camera:
        camera = load_camera(args.camera)
    else:
        calibration = calibrate_from_images(
            layout,
            image_paths,
            family=layout.family,
            count=args.calibration_count,
            min_views=args.min_calibration_views,
        )
        if not calibration.success or calibration.camera is None:
            result = {
                "success": False,
                "reason": "camera calibration failed",
                "calibration": calibration.to_dict(),
            }
            if args.output:
                write_json(args.output, result)
            else:
                print(result)
            return 1
        camera = calibration.camera
        if args.save_estimated_camera:
            save_camera(args.save_estimated_camera, camera)

    frames = [
        localize_one(
            path,
            layout,
            camera,
            annotate_dir,
            args.ransac_error_px,
            args.max_reprojection_error_px,
            args.axis_length_m,
        )
        for path in image_paths
    ]
    result = {
        "success": all(frame["pose"]["success"] for frame in frames),
        "board": str(args.board),
        "tag_size_m": measured_tag_size,
        "camera": camera.to_dict(),
        "calibration": calibration.to_dict() if calibration is not None else None,
        "frames": frames,
    }
    if args.output:
        write_json(args.output, result)
    else:
        import json

        print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

