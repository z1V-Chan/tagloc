from __future__ import annotations

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
        scale = measured / self.nominal_tag_size_m
        return self.scaled(scale, nominal_tag_size_m=measured)

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
