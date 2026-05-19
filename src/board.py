from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import matplotlib
import numpy as np

from utils.io import write_json


matplotlib.use("Agg")
import matplotlib.pyplot as plt

A4_WIDTH_M = 0.210
A4_HEIGHT_M = 0.297
DEFAULT_FAMILY = "DICT_APRILTAG_36h11"


@dataclass(frozen=True)
class TagSpec:
    tag_id: int
    center: np.ndarray
    corners: np.ndarray

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": int(self.tag_id),
            "center": self.center.astype(float).tolist(),
            "corners": self.corners.astype(float).tolist(),
        }


@dataclass(frozen=True)
class BoardLayout:
    family: str
    paper_size_m: tuple[float, float]
    nominal_tag_size_m: float
    tags: tuple[TagSpec, ...]
    world_frame: str = "paper_center_x_right_y_up_z_out"

    @property
    def paper_width_m(self) -> float:
        return float(self.paper_size_m[0])

    @property
    def paper_height_m(self) -> float:
        return float(self.paper_size_m[1])

    @property
    def tag_ids(self) -> list[int]:
        return [tag.tag_id for tag in self.tags]

    def tag_by_id(self) -> dict[int, TagSpec]:
        return {tag.tag_id: tag for tag in self.tags}

    def scaled_to_tag_size(self, measured_tag_size_m: float | None) -> "BoardLayout":
        if measured_tag_size_m is None:
            return self
        measured = float(measured_tag_size_m)
        if measured <= 0:
            raise ValueError("measured tag size must be positive")
        if self.world_frame.startswith("table_"):
            return self.with_tag_size(measured)
        scale = measured / self.nominal_tag_size_m
        return self.scaled(scale, nominal_tag_size_m=measured)

    def with_tag_size(self, tag_size_m: float) -> "BoardLayout":
        measured = float(tag_size_m)
        if measured <= 0:
            raise ValueError("tag size must be positive")
        half = measured * 0.5
        tags: list[TagSpec] = []
        for tag in self.tags:
            center = np.asarray(tag.center, dtype=np.float64).reshape(3)
            corners = np.asarray(tag.corners, dtype=np.float64)
            x_axis = corners[1] - corners[0]
            y_axis = corners[0] - corners[3]
            x_axis /= np.linalg.norm(x_axis)
            y_axis /= np.linalg.norm(y_axis)
            resized_corners = np.array(
                [
                    center - half * x_axis + half * y_axis,
                    center + half * x_axis + half * y_axis,
                    center + half * x_axis - half * y_axis,
                    center - half * x_axis - half * y_axis,
                ],
                dtype=np.float64,
            )
            tags.append(TagSpec(tag.tag_id, center, resized_corners))
        return BoardLayout(
            family=self.family,
            paper_size_m=self.paper_size_m,
            nominal_tag_size_m=measured,
            tags=tuple(tags),
            world_frame=self.world_frame,
        )

    def scaled(self, scale: float, nominal_tag_size_m: float | None = None) -> "BoardLayout":
        tags = tuple(
            TagSpec(tag.tag_id, tag.center * scale, tag.corners * scale)
            for tag in self.tags
        )
        return BoardLayout(
            family=self.family,
            paper_size_m=(self.paper_width_m * scale, self.paper_height_m * scale),
            nominal_tag_size_m=float(nominal_tag_size_m or self.nominal_tag_size_m * scale),
            tags=tags,
            world_frame=self.world_frame,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "family": self.family,
            "paper_size_m": [self.paper_width_m, self.paper_height_m],
            "nominal_tag_size_m": float(self.nominal_tag_size_m),
            "world_frame": self.world_frame,
            "tags": [tag.to_dict() for tag in self.tags],
        }


def create_default_layout(tag_size_m: float = 0.045, family: str = DEFAULT_FAMILY) -> BoardLayout:
    xs = [-0.060, 0.0, 0.060]
    ys = [0.090, 0.030, -0.030, -0.090]
    half = float(tag_size_m) * 0.5
    tags: list[TagSpec] = []
    tag_id = 0
    for y in ys:
        for x in xs:
            center = np.array([x, y, 0.0], dtype=np.float64)
            corners = np.array(
                [
                    [x - half, y + half, 0.0],
                    [x + half, y + half, 0.0],
                    [x + half, y - half, 0.0],
                    [x - half, y - half, 0.0],
                ],
                dtype=np.float64,
            )
            tags.append(TagSpec(tag_id, center, corners))
            tag_id += 1
    return BoardLayout(
        family=family,
        paper_size_m=(A4_WIDTH_M, A4_HEIGHT_M),
        nominal_tag_size_m=float(tag_size_m),
        tags=tuple(tags),
    )


def tag_corners_from_center(
    center_xy: tuple[float, float] | np.ndarray,
    tag_size_m: float,
    yaw_deg: float = 0.0,
) -> np.ndarray:
    center_2d = np.asarray(center_xy, dtype=np.float64).reshape(2)
    center = np.array([center_2d[0], center_2d[1], 0.0], dtype=np.float64)
    half = float(tag_size_m) * 0.5
    yaw = math.radians(float(yaw_deg))
    x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
    y_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float64)
    return np.array(
        [
            center - half * x_axis + half * y_axis,
            center + half * x_axis + half * y_axis,
            center + half * x_axis - half * y_axis,
            center - half * x_axis - half * y_axis,
        ],
        dtype=np.float64,
    )


def create_single_tag_layout(
    tag_id: int,
    tag_size_m: float = 0.045,
    family: str = DEFAULT_FAMILY,
    paper_size_m: tuple[float, float] = (A4_WIDTH_M, A4_HEIGHT_M),
) -> BoardLayout:
    center = np.zeros(3, dtype=np.float64)
    return BoardLayout(
        family=family,
        paper_size_m=paper_size_m,
        nominal_tag_size_m=float(tag_size_m),
        tags=(TagSpec(int(tag_id), center, tag_corners_from_center((0.0, 0.0), tag_size_m)),),
    )


def create_layout_from_placements(
    placements: list[tuple[int, float, float, float]],
    tag_size_m: float = 0.045,
    family: str = DEFAULT_FAMILY,
    table_size_m: tuple[float, float] | None = None,
    margin_m: float = 0.05,
) -> BoardLayout:
    if tag_size_m <= 0:
        raise ValueError("tag_size_m must be positive")
    if not placements:
        raise ValueError("at least one tag placement is required")

    seen: set[int] = set()
    tags: list[TagSpec] = []
    for tag_id, x_m, y_m, yaw_deg in placements:
        tag_id = int(tag_id)
        if tag_id in seen:
            raise ValueError(f"duplicate tag id: {tag_id}")
        seen.add(tag_id)
        center = np.array([float(x_m), float(y_m), 0.0], dtype=np.float64)
        tags.append(
            TagSpec(
                tag_id,
                center,
                tag_corners_from_center((float(x_m), float(y_m)), tag_size_m, float(yaw_deg)),
            )
        )

    if table_size_m is None:
        all_corners = np.concatenate([tag.corners[:, :2] for tag in tags], axis=0)
        half_w = float(np.max(np.abs(all_corners[:, 0]))) + float(margin_m)
        half_h = float(np.max(np.abs(all_corners[:, 1]))) + float(margin_m)
        table_size_m = (max(2.0 * half_w, tag_size_m), max(2.0 * half_h, tag_size_m))

    return BoardLayout(
        family=family,
        paper_size_m=(float(table_size_m[0]), float(table_size_m[1])),
        nominal_tag_size_m=float(tag_size_m),
        tags=tuple(tags),
        world_frame="table_center_x_right_y_up_z_out",
    )


def create_irregular_table_layout(
    tag_count: int,
    tag_size_m: float = 0.045,
    family: str = DEFAULT_FAMILY,
    table_size_m: tuple[float, float] = (0.62, 0.44),
    start_id: int = 0,
    seed: int = 7,
) -> BoardLayout:
    if tag_count <= 0:
        raise ValueError("tag_count must be positive")
    table_w, table_h = float(table_size_m[0]), float(table_size_m[1])
    margin = max(float(tag_size_m) * 1.25, 0.035)
    usable_w = table_w - 2.0 * margin
    usable_h = table_h - 2.0 * margin
    if usable_w <= tag_size_m or usable_h <= tag_size_m:
        raise ValueError("table_size_m is too small for the requested tag size")

    aspect = table_w / table_h
    cols = max(1, int(math.ceil(math.sqrt(tag_count * aspect))))
    rows = int(math.ceil(tag_count / cols))
    cell_w = usable_w / cols
    cell_h = usable_h / rows
    rng = np.random.default_rng(seed)
    placements: list[tuple[int, float, float, float]] = []
    for index in range(tag_count):
        row, col = divmod(index, cols)
        x = -usable_w / 2.0 + (col + 0.5) * cell_w
        y = usable_h / 2.0 - (row + 0.5) * cell_h
        x += float(rng.uniform(-0.22, 0.22) * cell_w)
        y += float(rng.uniform(-0.22, 0.22) * cell_h)
        x = float(np.clip(x, -table_w / 2.0 + margin, table_w / 2.0 - margin))
        y = float(np.clip(y, -table_h / 2.0 + margin, table_h / 2.0 - margin))
        yaw = float(rng.uniform(-70.0, 70.0))
        placements.append((int(start_id) + index, x, y, yaw))
    return create_layout_from_placements(
        placements,
        tag_size_m=tag_size_m,
        family=family,
        table_size_m=(table_w, table_h),
    )


def load_board_layout(path: str | Path) -> BoardLayout:
    import json

    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)
    tags = tuple(
        TagSpec(
            int(item["id"]),
            np.asarray(item["center"], dtype=np.float64),
            np.asarray(item["corners"], dtype=np.float64),
        )
        for item in data["tags"]
    )
    paper = data.get("paper_size_m", [A4_WIDTH_M, A4_HEIGHT_M])
    return BoardLayout(
        family=data.get("family", DEFAULT_FAMILY),
        paper_size_m=(float(paper[0]), float(paper[1])),
        nominal_tag_size_m=float(data["nominal_tag_size_m"]),
        tags=tags,
        world_frame=data.get("world_frame", "paper_center_x_right_y_up_z_out"),
    )


def save_board_layout(path: str | Path, layout: BoardLayout) -> Path:
    return write_json(path, layout.to_dict())


def aruco_dictionary(family: str):
    if not hasattr(cv2.aruco, family):
        raise ValueError(f"OpenCV has no ArUco/AprilTag dictionary named {family}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))


def marker_image(family: str, tag_id: int, size_px: int) -> np.ndarray:
    return cv2.aruco.generateImageMarker(aruco_dictionary(family), int(tag_id), int(size_px))


def save_board_pdf_and_preview(
    layout: BoardLayout,
    pdf_path: str | Path,
    png_path: str | Path,
    dpi: int = 300,
    draw_axes: bool = False,
) -> tuple[Path, Path]:
    pdf = Path(pdf_path)
    png = Path(png_path)
    pdf.parent.mkdir(parents=True, exist_ok=True)
    png.parent.mkdir(parents=True, exist_ok=True)

    width_in = layout.paper_width_m / 0.0254
    height_in = layout.paper_height_m / 0.0254
    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi, facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-layout.paper_width_m / 2.0, layout.paper_width_m / 2.0)
    ax.set_ylim(-layout.paper_height_m / 2.0, layout.paper_height_m / 2.0)
    ax.set_aspect("equal")
    ax.axis("off")

    if draw_axes:
        ax.plot(
            [-layout.paper_width_m / 2.0, layout.paper_width_m / 2.0],
            [0, 0],
            color="0.88",
            linewidth=0.3,
        )
        ax.plot(
            [0, 0],
            [-layout.paper_height_m / 2.0, layout.paper_height_m / 2.0],
            color="0.88",
            linewidth=0.3,
        )

    marker_px = 800
    for tag in layout.tags:
        image = marker_image(layout.family, tag.tag_id, marker_px)
        x0, y0, _ = tag.corners[3]
        x1, y1, _ = tag.corners[1]
        ax.imshow(
            image,
            cmap="gray",
            vmin=0,
            vmax=255,
            extent=(x0, x1, y0, y1),
            origin="upper",
            interpolation="nearest",
        )
        ax.text(
            float(tag.center[0]),
            float(tag.corners[2, 1] - 0.006),
            str(tag.tag_id),
            ha="center",
            va="top",
            fontsize=5,
            color="0.35",
        )

    fig.savefig(pdf, format="pdf")
    fig.savefig(png, format="png", dpi=dpi)
    plt.close(fig)
    return pdf, png
