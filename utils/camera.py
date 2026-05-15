from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from utils.io import read_data, write_json


@dataclass(frozen=True)
class CameraModel:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        k = np.asarray(self.camera_matrix, dtype=np.float64)
        d = np.asarray(self.dist_coeffs, dtype=np.float64).reshape(-1, 1)
        if k.shape != (3, 3):
            raise ValueError(f"camera_matrix must be 3x3, got {k.shape}")
        object.__setattr__(self, "camera_matrix", k)
        object.__setattr__(self, "dist_coeffs", d)

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1, 2])

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.reshape(-1).tolist(),
        }
        if self.image_size is not None:
            data["image_size"] = [int(self.image_size[0]), int(self.image_size[1])]
        return data


def load_camera(path: str | Path) -> CameraModel:
    data = read_data(path)
    if "camera_matrix" not in data:
        raise ValueError(f"{path} is missing camera_matrix")
    dist = data.get("dist_coeffs", data.get("distortion_coefficients", [0, 0, 0, 0, 0]))
    image_size = data.get("image_size")
    if image_size is not None:
        if len(image_size) != 2:
            raise ValueError("image_size must be [width, height]")
        image_size = (int(image_size[0]), int(image_size[1]))
    return CameraModel(np.asarray(data["camera_matrix"], dtype=np.float64), np.asarray(dist), image_size)


def save_camera(path: str | Path, camera: CameraModel) -> Path:
    return write_json(path, camera.to_dict())

