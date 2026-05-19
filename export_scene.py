from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from src.board import BoardLayout, load_board_layout, marker_image
from utils.camera import CameraModel
from utils.io import ensure_dir


Color = tuple[int, int, int]


@dataclass
class PlyScene:
    vertices: list[tuple[float, float, float, Color]]
    faces: list[tuple[list[int], Color]]
    edges: list[tuple[int, int, Color]]

    def add_vertex(self, point: Iterable[float], color: Color) -> int:
        p = np.asarray(list(point), dtype=np.float64).reshape(3)
        self.vertices.append((float(p[0]), float(p[1]), float(p[2]), color))
        return len(self.vertices) - 1

    def add_face(self, points: Iterable[Iterable[float]], color: Color) -> list[int]:
        indices = [self.add_vertex(p, color) for p in points]
        self.faces.append((indices, color))
        return indices

    def add_edge(self, p0: Iterable[float], p1: Iterable[float], color: Color) -> tuple[int, int]:
        i0 = self.add_vertex(p0, color)
        i1 = self.add_vertex(p1, color)
        self.edges.append((i0, i1, color))
        return i0, i1

    def write_ply(self, path: str | Path) -> Path:
        p = Path(path)
        ensure_dir(p.parent)
        with p.open("w", encoding="utf-8") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(self.vertices)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write(f"element face {len(self.faces)}\n")
            f.write("property list uchar int vertex_indices\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write(f"element edge {len(self.edges)}\n")
            f.write("property int vertex1\n")
            f.write("property int vertex2\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            for x, y, z, color in self.vertices:
                f.write(f"{x:.9f} {y:.9f} {z:.9f} {color[0]} {color[1]} {color[2]}\n")
            for indices, color in self.faces:
                joined = " ".join(str(i) for i in indices)
                f.write(f"{len(indices)} {joined} {color[0]} {color[1]} {color[2]}\n")
            for i0, i1, color in self.edges:
                f.write(f"{i0} {i1} {color[0]} {color[1]} {color[2]}\n")
        return p


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export board markers and camera poses to an ASCII PLY scene.")
    parser.add_argument("--localization", required=True, help="localize_camera.py output JSON.")
    parser.add_argument("--board", default=None, help="Board layout JSON. Defaults to the path in localization JSON.")
    parser.add_argument("--tag-size-m", type=float, default=None, help="Measured printed tag size in meters.")
    parser.add_argument("--output", required=True, help="Output PLY path.")
    parser.add_argument("--camera-depth-m", type=float, default=0.08, help="Depth of each camera frustum in meters.")
    parser.add_argument("--axis-length-m", type=float, default=0.06, help="Camera axis length in meters.")
    parser.add_argument(
        "--tag-grid",
        type=int,
        default=16,
        help="Number of colored mesh tiles per tag side for visualizing the AprilTag pattern.",
    )
    return parser.parse_args()


def _camera_from_localization(data: dict) -> CameraModel:
    camera = data["camera"]
    image_size = camera.get("image_size")
    if image_size is not None:
        image_size = (int(image_size[0]), int(image_size[1]))
    return CameraModel(camera["camera_matrix"], camera.get("dist_coeffs", [0, 0, 0, 0, 0]), image_size)


def _transform(t_wc: np.ndarray, points_c: np.ndarray) -> np.ndarray:
    pts = np.asarray(points_c, dtype=np.float64)
    return pts @ t_wc[:3, :3].T + t_wc[:3, 3]


def _point_on_tag(tag_corners: np.ndarray, u: float, v: float) -> np.ndarray:
    top = (1.0 - u) * tag_corners[0] + u * tag_corners[1]
    bottom = (1.0 - u) * tag_corners[3] + u * tag_corners[2]
    return (1.0 - v) * top + v * bottom


def _add_tag_pattern(scene: PlyScene, layout: BoardLayout, tag_id: int, tag_corners: np.ndarray, grid_size: int) -> None:
    marker_px = 256
    marker = marker_image(layout.family, tag_id, marker_px)
    small = cv2.resize(marker, (grid_size, grid_size), interpolation=cv2.INTER_AREA)
    corners = tag_corners.copy()
    corners[:, 2] = 0.0005
    for row in range(grid_size):
        for col in range(grid_size):
            value = int(small[row, col])
            color = (245, 245, 245) if value >= 128 else (8, 8, 8)
            u0 = col / grid_size
            u1 = (col + 1) / grid_size
            v0 = row / grid_size
            v1 = (row + 1) / grid_size
            scene.add_face(
                [
                    _point_on_tag(corners, u0, v0),
                    _point_on_tag(corners, u1, v0),
                    _point_on_tag(corners, u1, v1),
                    _point_on_tag(corners, u0, v1),
                ],
                color,
            )


def _add_board(scene: PlyScene, layout: BoardLayout, tag_grid: int) -> None:
    paper_w = layout.paper_width_m
    paper_h = layout.paper_height_m
    paper = np.array(
        [
            [-paper_w / 2.0, paper_h / 2.0, -0.0005],
            [paper_w / 2.0, paper_h / 2.0, -0.0005],
            [paper_w / 2.0, -paper_h / 2.0, -0.0005],
            [-paper_w / 2.0, -paper_h / 2.0, -0.0005],
        ],
        dtype=np.float64,
    )
    paper_indices = scene.add_face(paper, (245, 245, 245))
    for i0, i1 in zip(paper_indices, paper_indices[1:] + paper_indices[:1]):
        scene.edges.append((i0, i1, (180, 180, 180)))

    for tag in layout.tags:
        tag_points = tag.corners.copy()
        tag_points[:, 2] = 0.0005
        _add_tag_pattern(scene, layout, tag.tag_id, tag_points, tag_grid)
        tag_indices = [scene.add_vertex(p, (0, 0, 0)) for p in tag_points]
        for i0, i1 in zip(tag_indices, tag_indices[1:] + tag_indices[:1]):
            scene.edges.append((i0, i1, (0, 0, 0)))

    origin = np.zeros(3, dtype=np.float64)
    axis = max(layout.paper_width_m, layout.paper_height_m) * 0.18
    scene.add_edge(origin, [axis, 0, 0], (255, 0, 0))
    scene.add_edge(origin, [0, axis, 0], (0, 180, 0))
    scene.add_edge(origin, [0, 0, axis], (0, 0, 255))


def _add_camera(scene: PlyScene, camera: CameraModel, t_wc: np.ndarray, name_index: int, depth: float, axis_length: float) -> None:
    if camera.image_size is None:
        raise ValueError("camera image_size is required to export frustums")
    width, height = camera.image_size
    fx = camera.fx
    fy = camera.fy
    cx = camera.cx
    cy = camera.cy
    corners_px = np.array(
        [
            [0.0, 0.0],
            [float(width), 0.0],
            [float(width), float(height)],
            [0.0, float(height)],
        ],
        dtype=np.float64,
    )
    corners_c = np.column_stack(
        [
            (corners_px[:, 0] - cx) / fx * depth,
            (corners_px[:, 1] - cy) / fy * depth,
            np.full(4, depth, dtype=np.float64),
        ]
    )
    center_w = t_wc[:3, 3]
    corners_w = _transform(t_wc, corners_c)
    palette = [
        (230, 60, 60),
        (60, 160, 240),
        (245, 170, 40),
        (155, 90, 220),
        (60, 190, 120),
        (230, 80, 170),
        (120, 120, 40),
        (40, 170, 170),
    ]
    color = palette[name_index % len(palette)]
    corner_indices = [scene.add_vertex(p, color) for p in corners_w]
    center_idx = scene.add_vertex(center_w, color)
    scene.faces.append((corner_indices, tuple(max(0, int(c * 0.65)) for c in color)))
    for i0, i1 in zip(corner_indices, corner_indices[1:] + corner_indices[:1]):
        scene.faces.append(([center_idx, i0, i1], tuple(max(0, int(c * 0.45)) for c in color)))
    for idx in corner_indices:
        scene.edges.append((center_idx, idx, color))
    for i0, i1 in zip(corner_indices, corner_indices[1:] + corner_indices[:1]):
        scene.edges.append((i0, i1, color))

    axes_c = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float64,
    )
    axes_w = _transform(t_wc, axes_c)
    scene.add_edge(axes_w[0], axes_w[1], (255, 0, 0))
    scene.add_edge(axes_w[0], axes_w[2], (0, 180, 0))
    scene.add_edge(axes_w[0], axes_w[3], (0, 0, 255))


def export_scene(
    localization_path: str | Path,
    board_path: str | Path | None,
    tag_size_m: float | None,
    output_path: str | Path,
    camera_depth_m: float,
    axis_length_m: float,
    tag_grid: int,
) -> Path:
    with Path(localization_path).open("r", encoding="utf-8") as f:
        data = json.load(f)
    resolved_board = board_path or data.get("board")
    if resolved_board is None:
        raise ValueError("board path is required")
    resolved_tag_size = tag_size_m if tag_size_m is not None else data.get("tag_size_m")
    layout = load_board_layout(resolved_board).scaled_to_tag_size(resolved_tag_size)
    camera = _camera_from_localization(data)

    scene = PlyScene(vertices=[], faces=[], edges=[])
    _add_board(scene, layout, tag_grid)
    camera_count = 0
    for frame in data.get("frames", []):
        pose = frame.get("pose", {})
        if not pose.get("success") or "T_wc" not in pose:
            continue
        _add_camera(
            scene,
            camera,
            np.asarray(pose["T_wc"], dtype=np.float64),
            camera_count,
            camera_depth_m,
            axis_length_m,
        )
        camera_count += 1
    if camera_count == 0:
        raise ValueError("no successful camera poses found")
    return scene.write_ply(output_path)


def main() -> int:
    args = parse_args()
    path = export_scene(
        args.localization,
        args.board,
        args.tag_size_m,
        args.output,
        args.camera_depth_m,
        args.axis_length_m,
        args.tag_grid,
    )
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
