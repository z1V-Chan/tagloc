from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.board import BoardLayout
from src.detection import collect_correspondences, detect_tags, load_image
from utils.camera import CameraModel


@dataclass(frozen=True)
class CalibrationResult:
    success: bool
    reason: str
    camera: CameraModel | None = None
    rms_px: float | None = None
    valid_view_count: int = 0
    used_images: tuple[str, ...] = ()
    rvecs: tuple[np.ndarray, ...] = ()
    tvecs: tuple[np.ndarray, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "success": self.success,
            "reason": self.reason,
            "valid_view_count": int(self.valid_view_count),
            "used_images": list(self.used_images),
        }
        if self.rms_px is not None:
            data["rms_px"] = float(self.rms_px)
        if self.camera is not None:
            data["camera"] = self.camera.to_dict()
        return data


def calibrate_from_images(
    layout: BoardLayout,
    image_paths: list[str | Path],
    family: str | None = None,
    count: int = 10,
    min_views: int = 5,
) -> CalibrationResult:
    selected_paths = [Path(p) for p in image_paths[:count]]
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used_images: list[str] = []
    image_size: tuple[int, int] | None = None
    detector_family = family or layout.family

    for path in selected_paths:
        image = load_image(path)
        h, w = image.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            return CalibrationResult(False, "all calibration images must have the same size")
        detections = detect_tags(image, detector_family)
        obj, img, _ = collect_correspondences(layout, detections)
        if len(obj) >= 4:
            object_points.append(obj.astype(np.float32))
            image_points.append(img.astype(np.float32))
            used_images.append(str(path))

    if image_size is None:
        return CalibrationResult(False, "no calibration images were readable")
    if len(object_points) < min_views:
        return CalibrationResult(
            False,
            f"need at least {min_views} valid calibration views, got {len(object_points)}",
            valid_view_count=len(object_points),
            used_images=tuple(used_images),
        )

    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-7,
    )
    try:
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
            flags=flags,
            criteria=criteria,
        )
    except cv2.error as exc:
        return CalibrationResult(False, f"calibrateCamera failed: {exc}", valid_view_count=len(object_points))

    camera = CameraModel(camera_matrix, dist_coeffs, image_size)
    return CalibrationResult(
        True,
        "calibrated",
        camera=camera,
        rms_px=float(rms),
        valid_view_count=len(object_points),
        used_images=tuple(used_images),
        rvecs=tuple(np.asarray(r, dtype=np.float64).reshape(3, 1) for r in rvecs),
        tvecs=tuple(np.asarray(t, dtype=np.float64).reshape(3, 1) for t in tvecs),
    )

