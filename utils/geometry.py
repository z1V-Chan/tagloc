from __future__ import annotations

import math

import cv2
import numpy as np


def rvec_tvec_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rmat, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rmat
    mat[:3, 3] = t
    return mat


def invert_transform(mat: np.ndarray) -> np.ndarray:
    mat = np.asarray(mat, dtype=np.float64)
    inv = np.eye(4, dtype=np.float64)
    r = mat[:3, :3]
    t = mat[:3, 3]
    inv[:3, :3] = r.T
    inv[:3, 3] = -r.T @ t
    return inv


def rotation_angle_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    r_delta = np.asarray(r_a, dtype=np.float64) @ np.asarray(r_b, dtype=np.float64).T
    cos_theta = (float(np.trace(r_delta)) - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def transform_points(t_cw: np.ndarray, points_w: np.ndarray) -> np.ndarray:
    points = np.asarray(points_w, dtype=np.float64)
    return points @ t_cw[:3, :3].T + t_cw[:3, 3]

