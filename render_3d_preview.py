from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

from src.board import load_board_layout
from utils.io import ensure_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a static PNG preview of a tag layout and camera poses.")
    parser.add_argument("--localization", required=True, help="localize_camera.py output JSON.")
    parser.add_argument("--board", default=None, help="Board layout JSON. Defaults to the path in localization JSON.")
    parser.add_argument("--tag-size-m", type=float, default=None, help="Measured printed tag size in meters.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument("--camera-depth-m", type=float, default=0.12, help="Depth of each camera frustum in meters.")
    parser.add_argument("--max-cameras", type=int, default=80, help="Maximum number of camera frustums to draw.")
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def _transform(t_wc: np.ndarray, points_c: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_c, dtype=np.float64)
    return pts @ t_wc[:3, :3].T + t_wc[:3, 3]


def _camera_corners(camera: dict, depth: float) -> np.ndarray:
    width, height = camera["image_size"]
    matrix = np.asarray(camera["camera_matrix"], dtype=np.float64)
    fx = matrix[0, 0]
    fy = matrix[1, 1]
    cx = matrix[0, 2]
    cy = matrix[1, 2]
    px = np.array(
        [
            [0.0, 0.0],
            [float(width), 0.0],
            [float(width), float(height)],
            [0.0, float(height)],
        ],
        dtype=np.float64,
    )
    return np.column_stack(
        [
            (px[:, 0] - cx) / fx * depth,
            (px[:, 1] - cy) / fy * depth,
            np.full(4, depth, dtype=np.float64),
        ]
    )


def _set_equal_aspect(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    radius = max(float((maxs - mins).max()) * 0.55, 0.1)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def render_3d_preview(localization_path: str | Path, board_path: str | Path | None, tag_size_m: float | None, output_path: str | Path, camera_depth_m: float, max_cameras: int, dpi: int) -> Path:
    with Path(localization_path).open("r", encoding="utf-8") as f:
        localization = json.load(f)
    resolved_board = board_path or localization.get("board")
    if resolved_board is None:
        raise ValueError("board path is required")
    resolved_tag_size = tag_size_m if tag_size_m is not None else localization.get("tag_size_m")
    layout = load_board_layout(resolved_board).scaled_to_tag_size(resolved_tag_size)
    camera = localization["camera"]

    fig = plt.figure(figsize=(9.5, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Estimated AprilTag Layout and Camera Poses")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    all_points: list[np.ndarray] = []
    tag_faces = []
    tag_edges = []
    for tag in layout.tags:
        corners = np.asarray(tag.corners, dtype=np.float64)
        tag_faces.append(corners)
        for p0, p1 in zip(corners, np.roll(corners, -1, axis=0)):
            tag_edges.append([p0, p1])
        all_points.append(corners)
        center = np.asarray(tag.center, dtype=np.float64)
        ax.text(center[0], center[1], 0.01, str(tag.tag_id), ha="center", va="center", fontsize=8)

    ax.add_collection3d(Poly3DCollection(tag_faces, facecolors=(0.92, 0.92, 0.92, 0.8), edgecolors="black", linewidths=0.7))
    ax.add_collection3d(Line3DCollection(tag_edges, colors="black", linewidths=0.8))

    origin = np.zeros(3, dtype=np.float64)
    axis_len = max(layout.paper_width_m, layout.paper_height_m) * 0.15
    ax.plot([0, axis_len], [0, 0], [0, 0], color="red", linewidth=2)
    ax.plot([0, 0], [0, axis_len], [0, 0], color="green", linewidth=2)
    ax.plot([0, 0], [0, 0], [0, axis_len], color="blue", linewidth=2)
    all_points.append(np.array([origin, [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]], dtype=np.float64))

    successful_frames = [frame for frame in localization.get("frames", []) if frame.get("pose", {}).get("success") and "T_wc" in frame.get("pose", {})]
    if max_cameras > 0 and len(successful_frames) > max_cameras:
        indices = np.linspace(0, len(successful_frames) - 1, max_cameras).round().astype(int)
        frames_to_draw = [successful_frames[int(index)] for index in indices]
    else:
        frames_to_draw = successful_frames

    frustum_c = _camera_corners(camera, camera_depth_m)
    camera_edges = []
    camera_centers = []
    for frame in frames_to_draw:
        t_wc = np.asarray(frame["pose"]["T_wc"], dtype=np.float64)
        center = t_wc[:3, 3]
        corners_w = _transform(t_wc, frustum_c)
        camera_centers.append(center)
        for corner in corners_w:
            camera_edges.append([center, corner])
        for p0, p1 in zip(corners_w, np.roll(corners_w, -1, axis=0)):
            camera_edges.append([p0, p1])
        all_points.append(np.vstack([center, corners_w]))

    if camera_edges:
        ax.add_collection3d(Line3DCollection(camera_edges, colors=(0.1, 0.35, 0.95, 0.35), linewidths=0.6))
    if camera_centers:
        centers = np.asarray(camera_centers, dtype=np.float64)
        ax.scatter(centers[:, 0], centers[:, 1], centers[:, 2], s=8, c="#1f77b4", alpha=0.85, label="camera centers")
        ax.plot(centers[:, 0], centers[:, 1], centers[:, 2], color="#1f77b4", alpha=0.45, linewidth=1.0)

    _set_equal_aspect(ax, np.vstack(all_points))
    ax.view_init(elev=24, azim=-55)
    ax.legend(loc="upper right")
    fig.tight_layout()

    output = Path(output_path)
    ensure_dir(output.parent)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)
    return output


def main() -> int:
    args = parse_args()
    output = render_3d_preview(
        args.localization,
        args.board,
        args.tag_size_m,
        args.output,
        args.camera_depth_m,
        args.max_cameras,
        args.dpi,
    )
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
