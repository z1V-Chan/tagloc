from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src.board import BoardLayout, marker_image
from utils.camera import CameraModel
from utils.geometry import rvec_tvec_to_matrix


def render_board_texture(layout: BoardLayout, width_px: int = 2480) -> np.ndarray:
    height_px = int(round(width_px * layout.paper_height_m / layout.paper_width_m))
    texture = np.full((height_px, width_px, 3), 255, dtype=np.uint8)
    marker_px = max(40, int(round(width_px * layout.nominal_tag_size_m / layout.paper_width_m)))

    for tag in layout.tags:
        marker = marker_image(layout.family, tag.tag_id, marker_px)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        center_x = (float(tag.center[0]) + layout.paper_width_m / 2.0) / layout.paper_width_m
        center_y = (layout.paper_height_m / 2.0 - float(tag.center[1])) / layout.paper_height_m
        cx = int(round(center_x * width_px))
        cy = int(round(center_y * height_px))
        half = marker_px // 2
        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(width_px, x0 + marker_px)
        y1 = min(height_px, y0 + marker_px)
        sx0 = 0
        sy0 = 0
        sx1 = x1 - x0
        sy1 = y1 - y0
        texture[y0:y1, x0:x1] = marker_bgr[sy0:sy1, sx0:sx1]
    return texture


def look_at_tcw(
    eye_w: np.ndarray,
    target_w: np.ndarray | None = None,
    up_w: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    eye = np.asarray(eye_w, dtype=np.float64).reshape(3)
    target = np.zeros(3, dtype=np.float64) if target_w is None else np.asarray(target_w, dtype=np.float64).reshape(3)
    up = np.array([0.0, 1.0, 0.0], dtype=np.float64) if up_w is None else np.asarray(up_w, dtype=np.float64).reshape(3)

    z_cam_w = target - eye
    z_cam_w /= np.linalg.norm(z_cam_w)
    x_cam_w = np.cross(z_cam_w, up)
    x_cam_w /= np.linalg.norm(x_cam_w)
    y_cam_w = np.cross(z_cam_w, x_cam_w)
    y_cam_w /= np.linalg.norm(y_cam_w)
    r_wc = np.column_stack([x_cam_w, y_cam_w, z_cam_w])
    r_cw = r_wc.T
    t_cw = -r_cw @ eye
    rvec, _ = cv2.Rodrigues(r_cw)
    return rvec.reshape(3, 1), t_cw.reshape(3, 1)


def project_points(points_w: np.ndarray, camera: CameraModel, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    points, _ = cv2.projectPoints(
        np.asarray(points_w, dtype=np.float64),
        rvec.reshape(3, 1),
        tvec.reshape(3, 1),
        camera.camera_matrix,
        camera.dist_coeffs,
    )
    return points.reshape(-1, 2)


def paper_corners_world(layout: BoardLayout) -> np.ndarray:
    w = layout.paper_width_m
    h = layout.paper_height_m
    return np.array(
        [
            [-w / 2.0, h / 2.0, 0.0],
            [w / 2.0, h / 2.0, 0.0],
            [w / 2.0, -h / 2.0, 0.0],
            [-w / 2.0, -h / 2.0, 0.0],
        ],
        dtype=np.float64,
    )


def render_view(
    layout: BoardLayout,
    camera: CameraModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
    texture: np.ndarray | None = None,
    image_size: tuple[int, int] = (1280, 900),
    background: int = 230,
    occlusions: list[tuple[float, float, float, float]] | None = None,
    occlusion_polygons: list[list[tuple[float, float]]] | None = None,
    blur: float = 0.0,
    noise_std: float = 0.0,
) -> np.ndarray:
    width, height = image_size
    board_texture = texture if texture is not None else render_board_texture(layout)
    src_h, src_w = board_texture.shape[:2]
    src = np.array(
        [[0, 0], [src_w - 1, 0], [src_w - 1, src_h - 1], [0, src_h - 1]],
        dtype=np.float32,
    )
    dst = project_points(paper_corners_world(layout), camera, rvec, tvec).astype(np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    image = cv2.warpPerspective(
        board_texture,
        homography,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(background, background, background),
    )

    if occlusions:
        for rect in occlusions:
            x0, y0, x1, y1 = rect
            if max(abs(v) for v in rect) <= 1.0:
                x0, x1 = x0 * width, x1 * width
                y0, y1 = y0 * height, y1 * height
            cv2.rectangle(
                image,
                (int(round(x0)), int(round(y0))),
                (int(round(x1)), int(round(y1))),
                (210, 210, 210),
                thickness=-1,
            )

    if occlusion_polygons:
        for poly in occlusion_polygons:
            pts = np.asarray(poly, dtype=np.float32)
            if pts.ndim != 2 or pts.shape[1] != 2:
                raise ValueError("occlusion polygons must be lists of (x, y) points")
            if np.max(np.abs(pts)) <= 1.0:
                pts[:, 0] *= width
                pts[:, 1] *= height
            cv2.fillPoly(image, [np.round(pts).astype(np.int32)], (205, 205, 205))

    if blur > 0:
        k = max(3, int(round(blur)) | 1)
        image = cv2.GaussianBlur(image, (k, k), 0)
    if noise_std > 0:
        noise = np.random.default_rng(0).normal(0.0, noise_std, image.shape)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return image


def default_camera(image_size: tuple[int, int] = (1280, 900), focal_px: float = 950.0) -> CameraModel:
    width, height = image_size
    k = np.array(
        [
            [focal_px, 0.0, width / 2.0],
            [0.0, focal_px, height / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return CameraModel(k, np.zeros(5, dtype=np.float64), image_size)


def save_rendered_view(
    path: str | Path,
    layout: BoardLayout,
    camera: CameraModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
    **kwargs,
) -> Path:
    image = render_view(layout, camera, rvec, tvec, **kwargs)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(p), image)
    return p


def pose_matrix_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    return rvec_tvec_to_matrix(rvec, tvec)
