# A4 AprilTag Camera Localization

This project builds an A4 AprilTag calibration/localization sheet, estimates
camera pose from detected tags, and renders synthetic test images to validate
the pipeline.

The default implementation uses OpenCV's `cv2.aruco` AprilTag support in the
`rec` conda environment. No AprilRobotics submodule is required for the current
version.

## Coordinate System

- Unit: meters.
- World origin: center of the A4 paper.
- `+X`: right on the paper.
- `+Y`: up on the paper.
- `Z = 0`: paper plane.
- `+Z`: outward from the printed side.

The generated layout is a 3 x 4 grid of 12 `DICT_APRILTAG_36h11` tags. The
default nominal tag edge size is `0.045m`.

## Files

- `generate_board.py`: generate the A4 board PDF, PNG preview, and layout JSON.
- `localize_camera.py`: estimate camera pose from real or synthetic images.
- `export_scene.py`: export board markers and estimated camera poses to PLY.
- `render_test.py`: render synthetic board images and validate localization.
- `test.py`: end-to-end smoke test.
- `src/`: board, detection, pose, calibration, and rendering logic.
- `utils/`: camera, geometry, and I/O helpers.

Generated artifacts are written under `outputs/` and are ignored by git.

## Environment

Use the existing conda environment:

```bash
conda run -n rec python -V
```

The code expects:

- Python 3.11
- OpenCV with `cv2.aruco`
- NumPy
- Matplotlib
- PyYAML

These were verified in the `rec` environment.

## Generate The A4 Board

```bash
conda run -n rec python generate_board.py --output-dir outputs/board
```

This creates:

- `outputs/board/a4_apriltag_board.pdf`
- `outputs/board/a4_apriltag_board.png`
- `outputs/board/board_layout.json`

Print the PDF at actual size when possible. If the printer scales the page,
measure the edge size of one printed tag and pass that measured value to
localization with `--tag-size-m`.

The generated PDF is clean by default: no center crosshair or reference axes are
drawn, to avoid interfering with tag detection. For debugging only, pass
`--draw-axes` to `generate_board.py`.

Example: if the printed tag edge is `46.5mm`, use:

```bash
--tag-size-m 0.0465
```

The code then applies a uniform scale factor to the whole board:

```text
scale = measured_tag_size / nominal_tag_size
```

This handles uniform print scaling. It does not model different X/Y stretch.

## Camera Config Format

Known-intrinsics mode accepts JSON or YAML:

```yaml
camera_matrix:
  - [950.0, 0.0, 640.0]
  - [0.0, 950.0, 450.0]
  - [0.0, 0.0, 1.0]
dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
image_size: [1280, 900]
```

`image_size` is `[width, height]`.

## Localize With Known Intrinsics

```bash
conda run -n rec python localize_camera.py \
  --board outputs/board/board_layout.json \
  --tag-size-m 0.0465 \
  --camera camera.yaml \
  --output poses.json \
  --annotate-dir outputs/real_annotated \
  images/
```

The output JSON contains per-image detections, `rvec/tvec`, `T_cw`, `T_wc`,
reprojection error, inlier count, and the tag IDs used by PnP.

## Localize Without Intrinsics

When `--camera` is omitted, the first `N` images are used to estimate intrinsics
with `cv2.calibrateCamera`, then all provided images are localized.

```bash
conda run -n rec python localize_camera.py \
  --board outputs/board/board_layout.json \
  --tag-size-m 0.0465 \
  --calibration-count 10 \
  --min-calibration-views 5 \
  --save-estimated-camera outputs/estimated_camera.json \
  --output poses_selfcalib.json \
  images/
```

For self-calibration, the first images should show the board from varied
viewpoints. A sequence of nearly fronto-parallel views can produce poor
intrinsics.

## Pose Estimation Behavior

- Only detected tags whose IDs exist in `board_layout.json` are used.
- Partial board visibility is allowed. A frame can localize with a subset of
  complete detected tags.
- Multiple visible tags use all valid corners, `solvePnPRansac`, and
  `solvePnPRefineLM`.
- Sparse cases fall back to an IPPE planar pose solve.
- Frames with too few corners or high reprojection error report failure in the
  output JSON instead of silently returning an unreliable pose.

## Synthetic Render Test

Run the synthetic validation:

```bash
conda run -n rec python render_test.py --output-dir outputs/render_test
```

This renders a board with a deliberately different actual tag size, localizes
with `--tag-size-m` semantics, includes partial occlusion cases, and writes:

- synthetic images under `outputs/render_test/images/`
- overlays under `outputs/render_test/annotated/`
- metrics under `outputs/render_test/metrics.json`
- a synthetic camera file at `outputs/render_test/synthetic_camera.json`

Run the smoke test:

```bash
conda run -n rec python test.py
```

At the time of implementation, the smoke test passed with:

- max known-intrinsics camera-center error: `0.001767m`
- max known-intrinsics rotation error: `0.126815deg`
- self-calibration RMS: `0.2215px`

## Notes For Real Capture

- Keep the board flat.
- Measure the printed tag edge length at the tag border used by detection.
- Use good lighting and avoid motion blur.
- In no-intrinsics mode, capture at least 5 useful views and prefer 10 or more.
- If localization fails, inspect the annotated output first: missing detections
  usually indicate blur, small tags, glare, or low print contrast.
