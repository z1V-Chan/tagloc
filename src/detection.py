from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from src.board import BoardLayout, aruco_dictionary


@dataclass(frozen=True)
class TagDetection:
    tag_id: int
    corners: np.ndarray

    def to_dict(self) -> dict:
        return {"id": int(self.tag_id), "corners": self.corners.astype(float).tolist()}


def load_image(path: str | Path, grayscale: bool = False) -> np.ndarray:
    flag = cv2.IMREAD_GRAYSCALE if grayscale else cv2.IMREAD_COLOR
    image = cv2.imread(str(path), flag)
    if image is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return image


def make_detector(family: str) -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    if hasattr(params, "aprilTagQuadDecimate"):
        params.aprilTagQuadDecimate = 1.0
    return cv2.aruco.ArucoDetector(aruco_dictionary(family), params)


def detect_tags(image: np.ndarray, family: str) -> list[TagDetection]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    detector = make_detector(family)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    detections: list[TagDetection] = []
    for tag_id, corner in zip(ids.reshape(-1), corners):
        detections.append(TagDetection(int(tag_id), np.asarray(corner[0], dtype=np.float64)))
    return detections


def collect_correspondences(
    layout: BoardLayout,
    detections: list[TagDetection],
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    tags = layout.tag_by_id()
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    corner_tag_ids: list[int] = []
    for det in detections:
        tag = tags.get(det.tag_id)
        if tag is None:
            continue
        object_points.append(tag.corners)
        image_points.append(det.corners)
        corner_tag_ids.extend([det.tag_id] * 4)
    if not object_points:
        return (
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 2), dtype=np.float64),
            [],
        )
    return (
        np.concatenate(object_points, axis=0).astype(np.float64),
        np.concatenate(image_points, axis=0).astype(np.float64),
        corner_tag_ids,
    )


def draw_detections(image: np.ndarray, detections: list[TagDetection]) -> np.ndarray:
    out = image.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if not detections:
        return out
    corners = [det.corners.reshape(1, 4, 2).astype(np.float32) for det in detections]
    ids = np.asarray([[det.tag_id] for det in detections], dtype=np.int32)
    cv2.aruco.drawDetectedMarkers(out, corners, ids)
    return out

