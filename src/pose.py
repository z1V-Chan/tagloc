from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from src.board import BoardLayout
from src.detection import TagDetection, collect_correspondences
from utils.camera import CameraModel
from utils.geometry import invert_transform, rvec_tvec_to_matrix, transform_points


@dataclass(frozen=True)
class PoseResult:
    success: bool
    reason: str
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    t_cw: np.ndarray | None = None
    t_wc: np.ndarray | None = None
    reprojection_mean_px: float | None = None
    reprojection_rms_px: float | None = None
    reprojection_max_px: float | None = None
    used_tag_ids: tuple[int, ...] = ()
    inlier_count: int = 0
    point_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "success": self.success,
            "reason": self.reason,
            "used_tag_ids": list(self.used_tag_ids),
            "inlier_count": int(self.inlier_count),
            "point_count": int(self.point_count),
        }
        if self.rvec is not None:
            data["rvec"] = self.rvec.reshape(3).astype(float).tolist()
        if self.tvec is not None:
            data["tvec"] = self.tvec.reshape(3).astype(float).tolist()
        if self.t_cw is not None:
            data["T_cw"] = self.t_cw.astype(float).tolist()
        if self.t_wc is not None:
            data["T_wc"] = self.t_wc.astype(float).tolist()
        if self.reprojection_mean_px is not None:
            data["reprojection_mean_px"] = float(self.reprojection_mean_px)
            data["reprojection_rms_px"] = float(self.reprojection_rms_px)
            data["reprojection_max_px"] = float(self.reprojection_max_px)
        return data


def reprojection_errors(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera: CameraModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> np.ndarray:
    projected, _ = cv2.projectPoints(
        object_points.astype(np.float64),
        rvec.reshape(3, 1),
        tvec.reshape(3, 1),
        camera.camera_matrix,
        camera.dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    return np.linalg.norm(projected - image_points.reshape(-1, 2), axis=1)


def _select_ippe_solution(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera: CameraModel,
) -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    try:
        out = cv2.solvePnPGeneric(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            camera.camera_matrix,
            camera.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        return []
    ok, rvecs, tvecs = out[0], out[1], out[2]
    if not ok or len(rvecs) == 0:
        return []

    candidates: list[tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    all_inliers = np.arange(len(object_points), dtype=np.int32)
    for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        points_c = transform_points(rvec_tvec_to_matrix(rvec, tvec), object_points)
        positive_fraction = float(np.mean(points_c[:, 2] > 0))
        if positive_fraction < 0.75:
            continue
        candidates.append(
            (
                f"ippe_{index}",
                np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                np.asarray(tvec, dtype=np.float64).reshape(3, 1),
                all_inliers,
            )
        )
    return candidates


def _candidate_score(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera: CameraModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
    inlier_threshold_px: float,
) -> tuple[float, np.ndarray, float]:
    errors = reprojection_errors(object_points, image_points, camera, rvec, tvec)
    inliers = np.flatnonzero(errors <= inlier_threshold_px).astype(np.int32)
    if len(inliers) < 4:
        inliers = np.arange(len(object_points), dtype=np.int32)
    inlier_errors = errors[inliers]
    inlier_rms = float(np.sqrt(np.mean(inlier_errors**2)))
    outlier_count = len(object_points) - len(inliers)
    score = inlier_rms + outlier_count * float(inlier_threshold_px)
    return score, inliers, inlier_rms


def estimate_camera_pose(
    layout: BoardLayout,
    detections: list[TagDetection],
    camera: CameraModel,
    ransac_reprojection_error_px: float = 4.0,
    max_reprojection_error_px: float = 8.0,
) -> PoseResult:
    object_points, image_points, corner_tag_ids = collect_correspondences(layout, detections)
    point_count = int(len(object_points))
    if point_count < 4:
        return PoseResult(False, "need at least four detected layout corners", point_count=point_count)

    rvec = None
    tvec = None
    inliers = np.arange(point_count, dtype=np.int32)
    reason = "candidate_refined"
    candidates: list[tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []

    if point_count >= 8:
        try:
            ok, rv, tv, inlier_idx = cv2.solvePnPRansac(
                object_points,
                image_points,
                camera.camera_matrix,
                camera.dist_coeffs,
                iterationsCount=200,
                reprojectionError=float(ransac_reprojection_error_px),
                confidence=0.999,
                flags=cv2.SOLVEPNP_EPNP,
            )
        except cv2.error:
            ok, rv, tv, inlier_idx = False, None, None, None
        if ok and inlier_idx is not None and len(inlier_idx) >= 4:
            candidates.append(
                (
                    "ransac_epnp",
                    np.asarray(rv, dtype=np.float64).reshape(3, 1),
                    np.asarray(tv, dtype=np.float64).reshape(3, 1),
                    inlier_idx.reshape(-1).astype(np.int32),
                )
            )

    candidates.extend(_select_ippe_solution(object_points, image_points, camera))

    if point_count >= 4:
        for name, flag in (("sqpnp", cv2.SOLVEPNP_SQPNP), ("iterative", cv2.SOLVEPNP_ITERATIVE)):
            try:
                ok, rv, tv = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera.camera_matrix,
                    camera.dist_coeffs,
                    flags=flag,
                )
            except cv2.error:
                ok, rv, tv = False, None, None
            if ok:
                candidates.append(
                    (
                        name,
                        np.asarray(rv, dtype=np.float64).reshape(3, 1),
                        np.asarray(tv, dtype=np.float64).reshape(3, 1),
                        np.arange(point_count, dtype=np.int32),
                    )
                )

    if not candidates:
        return PoseResult(False, "PnP failed", point_count=point_count)

    best: tuple[float, str, np.ndarray, np.ndarray, np.ndarray] | None = None
    inlier_threshold_px = max(float(ransac_reprojection_error_px), float(max_reprojection_error_px))
    for candidate_reason, candidate_rvec, candidate_tvec, candidate_inliers in candidates:
        candidate_inliers = candidate_inliers.astype(np.int32)
        if len(candidate_inliers) < 4:
            continue
        refined_rvec = candidate_rvec.copy()
        refined_tvec = candidate_tvec.copy()
        try:
            refined_rvec, refined_tvec = cv2.solvePnPRefineLM(
                object_points[candidate_inliers],
                image_points[candidate_inliers],
                camera.camera_matrix,
                camera.dist_coeffs,
                refined_rvec,
                refined_tvec,
            )
        except cv2.error:
            pass
        score, scored_inliers, _ = _candidate_score(
            object_points,
            image_points,
            camera,
            refined_rvec,
            refined_tvec,
            inlier_threshold_px,
        )
        if len(scored_inliers) >= 4:
            try:
                refined_rvec, refined_tvec = cv2.solvePnPRefineLM(
                    object_points[scored_inliers],
                    image_points[scored_inliers],
                    camera.camera_matrix,
                    camera.dist_coeffs,
                    refined_rvec,
                    refined_tvec,
                )
            except cv2.error:
                pass
            score, scored_inliers, _ = _candidate_score(
                object_points,
                image_points,
                camera,
                refined_rvec,
                refined_tvec,
                inlier_threshold_px,
            )
        if best is None or score < best[0]:
            best = (score, f"{candidate_reason}_refined", refined_rvec, refined_tvec, scored_inliers)

    if best is None:
        return PoseResult(False, "PnP failed", point_count=point_count)

    _, reason, rvec, tvec, inliers = best
    refined_obj = object_points[inliers]
    refined_img = image_points[inliers]

    errors = reprojection_errors(refined_obj, refined_img, camera, rvec, tvec)
    mean = float(np.mean(errors))
    rms = float(np.sqrt(np.mean(errors**2)))
    max_error = float(np.max(errors))
    if rms > float(max_reprojection_error_px):
        return PoseResult(
            False,
            f"reprojection RMS {rms:.3f}px exceeds limit {max_reprojection_error_px:.3f}px",
            rvec=rvec,
            tvec=tvec,
            reprojection_mean_px=mean,
            reprojection_rms_px=rms,
            reprojection_max_px=max_error,
            point_count=point_count,
            inlier_count=int(len(inliers)),
        )

    t_cw = rvec_tvec_to_matrix(rvec, tvec)
    t_wc = invert_transform(t_cw)
    used_ids = tuple(sorted({corner_tag_ids[int(i)] for i in inliers}))
    return PoseResult(
        True,
        reason,
        rvec=rvec,
        tvec=tvec,
        t_cw=t_cw,
        t_wc=t_wc,
        reprojection_mean_px=mean,
        reprojection_rms_px=rms,
        reprojection_max_px=max_error,
        used_tag_ids=used_ids,
        inlier_count=int(len(inliers)),
        point_count=point_count,
    )


def draw_pose_axes(
    image: np.ndarray,
    camera: CameraModel,
    pose: PoseResult,
    axis_length_m: float,
) -> np.ndarray:
    out = image.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if not pose.success or pose.rvec is None or pose.tvec is None:
        return out
    cv2.drawFrameAxes(
        out,
        camera.camera_matrix,
        camera.dist_coeffs,
        pose.rvec.reshape(3, 1),
        pose.tvec.reshape(3, 1),
        float(axis_length_m),
        2,
    )
    return out
