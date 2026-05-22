from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.board import BoardLayout, load_board_layout
from src.detection import TagDetection, draw_detections, make_detector
from src.pose import PoseResult, estimate_camera_pose
from utils.camera import CameraModel, load_camera
from utils.io import ensure_dir, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Annotate a video with AprilTag detections and estimated table-plane coordinate axes."
    )
    parser.add_argument("video", help="Input video path.")
    parser.add_argument("--board", required=True, help="Estimated table layout JSON.")
    parser.add_argument("--camera", required=True, help="Camera JSON.")
    parser.add_argument("--output", required=True, help="Output annotated video path.")
    parser.add_argument("--summary", default=None, help="Optional output summary JSON.")
    parser.add_argument("--tag-size-m", type=float, default=None, help="Measured tag edge size in meters.")
    parser.add_argument("--axis-length-m", type=float, default=0.35, help="Length of table-frame axes to draw.")
    parser.add_argument("--ransac-error-px", type=float, default=6.0)
    parser.add_argument("--max-reprojection-error-px", type=float, default=12.0)
    parser.add_argument("--max-frames", type=int, default=None, help="Optional frame limit for debugging.")
    parser.add_argument("--stride", type=int, default=1, help="Process every Nth input frame.")
    parser.add_argument("--codec", default="mp4v", help="OpenCV fourcc for output video.")
    parser.add_argument("--progress-every", type=int, default=120, help="Print progress every N processed frames.")
    return parser.parse_args()


def _detect_with_detector(detector: cv2.aruco.ArucoDetector, frame: np.ndarray) -> list[TagDetection]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [
        TagDetection(int(tag_id), np.asarray(corner[0], dtype=np.float64))
        for tag_id, corner in zip(ids.reshape(-1), corners)
    ]


def _project_points(camera: CameraModel, pose: PoseResult, object_points: np.ndarray) -> np.ndarray | None:
    if not pose.success or pose.rvec is None or pose.tvec is None:
        return None
    projected, _ = cv2.projectPoints(
        object_points.astype(np.float64),
        pose.rvec.reshape(3, 1),
        pose.tvec.reshape(3, 1),
        camera.camera_matrix,
        camera.dist_coeffs,
    )
    return projected.reshape(-1, 2)


def _draw_projected_layout(frame: np.ndarray, layout: BoardLayout, camera: CameraModel, pose: PoseResult) -> np.ndarray:
    out = frame.copy()
    if not pose.success:
        return out
    for tag in layout.tags:
        projected = _project_points(camera, pose, tag.corners)
        if projected is None:
            continue
        poly = np.rint(projected).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [poly], isClosed=True, color=(180, 180, 180), thickness=1, lineType=cv2.LINE_AA)
    return out


def _draw_table_axes(frame: np.ndarray, camera: CameraModel, pose: PoseResult, axis_length_m: float) -> np.ndarray:
    out = frame.copy()
    axis = float(axis_length_m)
    object_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis, 0.0, 0.0],
            [0.0, axis, 0.0],
            [0.0, 0.0, axis],
        ],
        dtype=np.float64,
    )
    projected = _project_points(camera, pose, object_points)
    if projected is None:
        return out
    pts = np.rint(projected).astype(np.int32)
    origin = tuple(int(v) for v in pts[0])
    axes = [
        (tuple(int(v) for v in pts[1]), (0, 0, 255), "X"),
        (tuple(int(v) for v in pts[2]), (0, 180, 0), "Y"),
        (tuple(int(v) for v in pts[3]), (255, 0, 0), "Z"),
    ]
    for end, color, label in axes:
        cv2.arrowedLine(out, origin, end, color, 4, cv2.LINE_AA, tipLength=0.08)
        cv2.putText(out, label, (end[0] + 8, end[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
    cv2.circle(out, origin, 5, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.putText(out, "table frame", (origin[0] + 10, origin[1] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def _draw_status(frame: np.ndarray, frame_index: int, detections: list[TagDetection], pose: PoseResult) -> np.ndarray:
    out = frame.copy()
    used = ",".join(str(tag_id) for tag_id in pose.used_tag_ids) if pose.success else "-"
    if pose.success and pose.reprojection_rms_px is not None:
        status = f"frame {frame_index}  tags {len(detections)}  used {used}  rms {pose.reprojection_rms_px:.2f}px"
        color = (60, 220, 60)
    else:
        status = f"frame {frame_index}  tags {len(detections)}  pose failed: {pose.reason}"
        color = (40, 40, 255)
    cv2.rectangle(out, (12, 12), (min(out.shape[1] - 12, 980), 52), (0, 0, 0), -1)
    cv2.putText(out, status, (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
    return out


def _open_writer(path: Path, codec: str, fps: float, size: tuple[int, int]) -> cv2.VideoWriter:
    ensure_dir(path.parent)
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(str(path), fourcc, fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer: {path}")
    return writer


def annotate_video(
    video_path: str | Path,
    layout: BoardLayout,
    camera: CameraModel,
    output_path: str | Path,
    axis_length_m: float,
    ransac_error_px: float,
    max_reprojection_error_px: float,
    max_frames: int | None,
    stride: int,
    codec: str,
    progress_every: int,
) -> dict[str, Any]:
    if stride <= 0:
        raise ValueError("--stride must be positive")
    input_path = Path(video_path)
    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"could not open video: {input_path}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    input_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    output_fps = input_fps / float(stride) if input_fps > 0 else 30.0
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    writer = _open_writer(Path(output_path), codec, output_fps, (width, height))
    detector = make_detector(layout.family)

    processed = 0
    written = 0
    success_count = 0
    detection_counts: list[int] = []
    used_counts: list[int] = []
    rms_values: list[float] = []
    failed_frames: list[dict[str, Any]] = []
    started = time.monotonic()

    try:
        input_index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if input_index % stride != 0:
                input_index += 1
                continue
            if max_frames is not None and processed >= max_frames:
                break

            detections = _detect_with_detector(detector, frame)
            pose = estimate_camera_pose(
                layout,
                detections,
                camera,
                ransac_reprojection_error_px=ransac_error_px,
                max_reprojection_error_px=max_reprojection_error_px,
            )
            overlay = draw_detections(frame, detections)
            overlay = _draw_projected_layout(overlay, layout, camera, pose)
            overlay = _draw_table_axes(overlay, camera, pose, axis_length_m)
            overlay = _draw_status(overlay, input_index, detections, pose)
            writer.write(overlay)

            detection_counts.append(len(detections))
            used_counts.append(len(pose.used_tag_ids) if pose.success else 0)
            if pose.success:
                success_count += 1
                if pose.reprojection_rms_px is not None:
                    rms_values.append(float(pose.reprojection_rms_px))
            else:
                failed_frames.append({"frame_index": input_index, "reason": pose.reason, "detected_tags": len(detections)})

            processed += 1
            written += 1
            if progress_every > 0 and processed % progress_every == 0:
                elapsed = max(time.monotonic() - started, 1e-6)
                print(f"processed {processed} frames ({processed / elapsed:.2f} fps)")
            input_index += 1
    finally:
        cap.release()
        writer.release()

    elapsed = time.monotonic() - started
    summary: dict[str, Any] = {
        "input_video": str(input_path),
        "output_video": str(output_path),
        "input_frame_count": frame_count,
        "input_fps": input_fps,
        "output_fps": output_fps,
        "frame_size": [width, height],
        "stride": int(stride),
        "processed_frames": processed,
        "written_frames": written,
        "pose_success_frames": success_count,
        "pose_failed_frames": processed - success_count,
        "failed_frames": failed_frames[:100],
        "elapsed_sec": elapsed,
    }
    if detection_counts:
        summary["detected_tags_per_frame"] = {
            "min": int(min(detection_counts)),
            "max": int(max(detection_counts)),
            "mean": float(np.mean(detection_counts)),
            "total": int(sum(detection_counts)),
        }
    if used_counts:
        summary["used_tags_per_frame"] = {
            "min": int(min(used_counts)),
            "max": int(max(used_counts)),
            "mean": float(np.mean(used_counts)),
        }
    if rms_values:
        summary["reprojection_rms_px"] = {
            "min": float(np.min(rms_values)),
            "max": float(np.max(rms_values)),
            "mean": float(np.mean(rms_values)),
            "median": float(np.median(rms_values)),
        }
    return summary


def main() -> int:
    args = parse_args()
    layout = load_board_layout(args.board).scaled_to_tag_size(args.tag_size_m)
    camera = load_camera(args.camera)
    summary = annotate_video(
        args.video,
        layout,
        camera,
        args.output,
        args.axis_length_m,
        args.ransac_error_px,
        args.max_reprojection_error_px,
        args.max_frames,
        args.stride,
        args.codec,
        args.progress_every,
    )
    if args.summary:
        write_json(args.summary, summary)
    print(f"wrote {args.output}")
    if args.summary:
        print(f"summary {args.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
