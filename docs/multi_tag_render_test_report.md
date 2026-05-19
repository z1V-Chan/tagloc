# Multi-Tag Render Test Report

This report explains the synthetic test result in
`outputs/multi_tag_render_test` and records the exact commands needed to
reproduce it.

## Test Goal

The multi-tag render test validates the AprilTag localization pipeline for a
different physical setup:

- Multiple independent printed pages, with one AprilTag per page.
- Same physical tag size on every page.
- Undistorted input images.
- Irregular tag positions and yaw angles on one shared table plane.
- A table/world coordinate frame whose physical table boundary does not need to
  be visible.
- Known-intrinsics layout estimation and localization.
- No-intrinsics joint estimation of camera intrinsics, calibration-view camera
  poses, and tag layout from the first `n` images.
- Later localization frames where some tags are fully hidden.

The key feasibility condition is that every tag's planar pose is either known
from measurement or estimated from calibration images in one shared world
frame. If tag positions are arbitrary and neither measured nor estimated, the
system can detect tag IDs but cannot recover one absolute table/world pose from
those unrelated tags.

In this report, tilt angle means the angle between the table normal and the
camera-to-table viewing direction. `0deg` is fronto-parallel; larger values are
more grazing.

## Reproduction Commands

Run all commands from the repository root:

```bash
cd /ssd1/czw_new/tagloc
```

Optional cleanup for a fresh run:

```bash
rm -rf outputs/multi_tags_16 outputs/multi_tag_render_test
```

Generate one sample printable page per tag and a deterministic irregular table
layout:

```bash
conda run -n rec python generate_multi_tags.py \
  --output-dir outputs/multi_tags_16 \
  --count 16 \
  --tag-size-m 0.16 \
  --sample-layout \
  --seed 7
```

Generate the synthetic irregular multi-tag render test:

```bash
conda run -n rec python render_multi_tag_test.py \
  --output-dir outputs/multi_tag_render_test \
  --tag-count 16 \
  --tag-size-m 0.16 \
  --table-width-m 2.8 \
  --table-height-m 2.8 \
  --calibration-views 10 \
  --seed 7
```

Estimate the irregular tag layout from the first 10 images with known
intrinsics. `layout/table_layout.json` is the renderer's ground truth and is
not passed to localization in this workflow:

```bash
conda run -n rec python estimate_table_layout.py \
  --tag-size-m 0.16 \
  --camera outputs/multi_tag_render_test/synthetic_camera.json \
  --calibration-count 10 \
  --output outputs/multi_tag_render_test/estimated_layout_known.json \
  --summary outputs/multi_tag_render_test/estimated_layout_known_summary.json \
  outputs/multi_tag_render_test/images
```

Localize the later evaluation images with known intrinsics and the estimated
layout:

```bash
conda run -n rec python localize_camera.py \
  --board outputs/multi_tag_render_test/estimated_layout_known.json \
  --camera outputs/multi_tag_render_test/synthetic_camera.json \
  --output outputs/multi_tag_render_test/localize_estimated_layout_known.json \
  --annotate-dir outputs/multi_tag_render_test/localize_estimated_layout_known_annotated \
  'outputs/multi_tag_render_test/images/view_1*_evaluation.png'
```

Estimate intrinsics and layout jointly from the first 10 images:

```bash
conda run -n rec python estimate_table_layout.py \
  --tag-size-m 0.16 \
  --calibration-count 10 \
  --min-camera-observations 20 \
  --output outputs/multi_tag_render_test/estimated_layout_selfcalib.json \
  --save-estimated-camera outputs/multi_tag_render_test/estimated_layout_camera.json \
  --summary outputs/multi_tag_render_test/estimated_layout_selfcalib_summary.json \
  outputs/multi_tag_render_test/images
```

Localize the later evaluation images with the jointly estimated intrinsics and
layout:

```bash
conda run -n rec python localize_camera.py \
  --board outputs/multi_tag_render_test/estimated_layout_selfcalib.json \
  --camera outputs/multi_tag_render_test/estimated_layout_camera.json \
  --output outputs/multi_tag_render_test/localize_estimated_layout_selfcalib.json \
  --annotate-dir outputs/multi_tag_render_test/localize_estimated_layout_selfcalib_annotated \
  'outputs/multi_tag_render_test/images/view_1*_evaluation.png'
```

Run the original smoke test to confirm the default A4-board path still passes:

```bash
conda run -n rec python test.py
```

The smoke-test summary observed after this change was:

```text
known-intrinsics frames: 8/8
self-calibration: calibrated
self-calibration RMS: 0.2160px
smoke test passed: max center error 0.010445m, max rotation error 0.249998deg
```

## Generated Files

Sample page generation writes:

- `outputs/multi_tags_16/pages/tag_000.pdf` and matching PNG previews.
- `outputs/multi_tags_16/pages/tag_001.pdf` through `tag_015.pdf`.
- `outputs/multi_tags_16/tag_pages_manifest.json`.
- `outputs/multi_tags_16/table_layout.json`.

The synthetic render test writes:

- `layout/table_layout.json`: renderer-only ground-truth table layout.
- `layout/table_layout_preview.png`: top-down preview of the simulated table.
- `synthetic_camera.json`: ground-truth camera intrinsics used for rendering.
- `images/view_00_calibration.png` to `images/view_09_calibration.png`:
  calibration views.
- `images/view_10_evaluation.png` to `images/view_15_evaluation.png`:
  evaluation views with hidden tags.
- `annotated/`: overlays generated by `render_multi_tag_test.py`.
- `estimated_layout_known.json`: layout estimated from the first 10 images with known intrinsics.
- `estimated_layout_known_summary.json`: known-intrinsics layout-estimation summary.
- `localize_estimated_layout_known.json`: known-intrinsics localization with the estimated layout.
- `estimated_layout_selfcalib.json`: layout jointly estimated with camera intrinsics.
- `estimated_layout_camera.json`: camera estimated jointly with the layout.
- `estimated_layout_selfcalib_summary.json`: joint-estimation summary.
- `localize_estimated_layout_selfcalib.json`: localization with jointly estimated intrinsics and layout.
- `metrics.json`: primary test metrics.

## Layout Setup

The rendered table layout used:

```text
tag_count = 16
tag_size_m = 0.16
table_size_m = [2.8, 2.8]
paper_size_m = [0.21, 0.297]
world_frame = table_center_x_right_y_up_z_out
```

Each tag has its own center and yaw angle in the table coordinate frame. The
simulated table boundary is only a rendering surface; pose estimation uses the
tag corner coordinates from `table_layout.json`.

The table is deliberately larger than the minimum tag footprint. With the
reported seed, the closest tag centers are `0.4254m` apart, which leaves about
`0.0616m` beyond an A4 page diagonal. This avoids the paper-on-paper overlap
that occurred with the earlier `1.7m x 1.3m` sample table.

For `table_*` layouts, passing `--tag-size-m` to `localize_camera.py` preserves
the measured tag centers and only resizes each tag's corners. This differs from
the default A4 board, where `--tag-size-m` uniformly scales the whole board.

## Summary Result

`metrics.json` reports:

```text
success = true
```

This means the renderer-side validation succeeded. The reproduction workflow
below is stricter: it first estimates the unknown tag layout from the first 10
images, then localizes only the later evaluation images.

The CLI reproduction commands also passed:

```text
outputs/multi_tag_render_test/localize_estimated_layout_known.json: success true, frames 6/6
outputs/multi_tag_render_test/localize_estimated_layout_selfcalib.json: success true, frames 6/6
```

## Detection Quality

Detection quality is reported before pose solving. For calibration frames, the
expected tag set is all 16 tags. For evaluation frames, the expected tag set is
all 16 tags minus the deliberately hidden IDs. A missed tag is an expected tag
that was not detected. A false detection is a detected tag outside the expected
set.

`metrics.json` includes these values under `detection_summary` and
`detection_quality`.

Summary:

| Set | Frames | Expected tags | Detected expected tags | Missed tags | Miss rate | False detections | False detection rate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Calibration | 10 | 160 | 138 | 22 | `13.75%` | 0 | `0.00%` |
| Evaluation | 6 | 76 | 56 | 20 | `26.32%` | 0 | `0.00%` |
| Overall | 16 | 236 | 194 | 42 | `17.80%` | 0 | `0.00%` |

Per-frame detection results:

| Image | Expected | Detected | Missed | Miss rate | Missed IDs |
| --- | ---: | ---: | ---: | ---: | --- |
| `view_00_calibration.png` | 16 | 16 | 0 | `0.00%` | `-` |
| `view_01_calibration.png` | 16 | 16 | 0 | `0.00%` | `-` |
| `view_02_calibration.png` | 16 | 16 | 0 | `0.00%` | `-` |
| `view_03_calibration.png` | 16 | 14 | 2 | `12.50%` | `9, 15` |
| `view_04_calibration.png` | 16 | 12 | 4 | `25.00%` | `11, 13, 14, 15` |
| `view_05_calibration.png` | 16 | 12 | 4 | `25.00%` | `3, 7, 10, 15` |
| `view_06_calibration.png` | 16 | 14 | 2 | `12.50%` | `2, 3` |
| `view_07_calibration.png` | 16 | 16 | 0 | `0.00%` | `-` |
| `view_08_calibration.png` | 16 | 12 | 4 | `25.00%` | `1, 4, 8, 9` |
| `view_09_calibration.png` | 16 | 10 | 6 | `37.50%` | `0, 4, 8, 9, 12, 15` |
| `view_10_evaluation.png` | 13 | 12 | 1 | `7.69%` | `8` |
| `view_11_evaluation.png` | 12 | 12 | 0 | `0.00%` | `-` |
| `view_12_evaluation.png` | 13 | 9 | 4 | `30.77%` | `10, 13, 14, 15` |
| `view_13_evaluation.png` | 13 | 9 | 4 | `30.77%` | `0, 7, 11, 15` |
| `view_14_evaluation.png` | 13 | 9 | 4 | `30.77%` | `0, 1, 2, 3` |
| `view_15_evaluation.png` | 12 | 5 | 7 | `58.33%` | `0, 3, 4, 5, 6, 8, 9` |

Interpretation:

- There were no false tag IDs in this run.
- Misses increase in grazing views where some tags become small or strongly
  foreshortened.
- Pose estimation still succeeds because it only needs enough complete detected
  tag corners, not all tags in the layout.

## Evaluation View Conditions

The evaluation views use oblique angles and explicitly hide selected complete
tags after rendering the table layout:

| Image | Tilt | Table area ratio | Hidden tag IDs |
| --- | ---: | ---: | --- |
| `view_10_evaluation.png` | `46deg` | `0.40` | `1, 6, 11` |
| `view_11_evaluation.png` | `55deg` | `0.42` | `0, 5, 10, 15` |
| `view_12_evaluation.png` | `63deg` | `0.38` | `2, 7, 12` |
| `view_13_evaluation.png` | `70deg` | `0.40` | `3, 8, 13` |
| `view_14_evaluation.png` | `74deg` | `0.36` | `4, 9, 14` |
| `view_15_evaluation.png` | `78deg` | `0.34` | `1, 2, 12, 13` |

## Known-Intrinsics Layout Estimation

These results use the ground-truth synthetic camera from
`synthetic_camera.json`, but not the ground-truth tag layout. The first 10
images initialize the layout from single-tag poses, then jointly optimize all
calibration-image camera poses plus every non-anchor tag's planar `x/y/yaw`.
The anchor tag is fixed at the estimated world origin to remove gauge freedom.

Layout-estimation summary:

```text
estimated_tag_count = 16
single_tag_pose_count = 138
mean_single_tag_pose_rms_px = 0.0859
joint_initial_reprojection_rms_px = 0.3748
joint_final_reprojection_rms_px = 0.1304
optimized_intrinsics = false
```

Evaluation with `estimated_layout_known.json`:

| Image | Detected tags | Used tags | Solver | Reprojection RMS |
| --- | ---: | ---: | --- | ---: |
| `view_10_evaluation.png` | 12 | 12 | `ippe_0_refined` | `0.151px` |
| `view_11_evaluation.png` | 12 | 12 | `ippe_0_refined` | `0.124px` |
| `view_12_evaluation.png` | 9 | 9 | `ippe_0_refined` | `0.141px` |
| `view_13_evaluation.png` | 9 | 9 | `iterative_refined` | `0.196px` |
| `view_14_evaluation.png` | 9 | 9 | `ippe_0_refined` | `0.297px` |
| `view_15_evaluation.png` | 5 | 5 | `sqpnp_refined` | `0.143px` |

Interpretation:

- All irregular-layout evaluation frames localized.
- Full-tag occlusion is tolerated when enough other complete tags remain.
- `view_15_evaluation.png` is the sparsest evaluation case with 5 detected
  tags after hiding 4 tags.
- Reprojection RMS remains below `0.30px` in all known-intrinsics evaluation
  frames.

## Joint Intrinsics And Layout Estimation

The no-intrinsics workflow assumes undistorted input images. It first
bootstraps intrinsics from individual tag observations, then jointly optimizes
`fx/fy/cx/cy`, every calibration image's camera pose, and every non-anchor
tag's planar `x/y/yaw`. Distortion coefficients are fixed to zero.

Bootstrap and joint result:

```text
success = true
single_tag_observation_count = 138
bootstrap_camera_rms_px = 0.1207918795
joint_initial_reprojection_rms_px = 0.8168
joint_final_reprojection_rms_px = 0.1294
optimized_intrinsics = true
```

Jointly estimated camera matrix:

```text
[[949.9098992006914, 0.0, 640.6537093332335],
 [0.0, 950.1600445349425, 451.03508858387147],
 [0.0, 0.0, 1.0]]
```

Distortion coefficients:

```text
[0.0, 0.0, 0.0, 0.0, 0.0]
```

Evaluation with `estimated_layout_selfcalib.json` and
`estimated_layout_camera.json`:

| Image | Detected tags | Used tags | Solver | Reprojection RMS |
| --- | ---: | ---: | --- | ---: |
| `view_10_evaluation.png` | 12 | 12 | `iterative_refined` | `0.151px` |
| `view_11_evaluation.png` | 12 | 12 | `iterative_refined` | `0.125px` |
| `view_12_evaluation.png` | 9 | 9 | `ippe_0_refined` | `0.133px` |
| `view_13_evaluation.png` | 9 | 9 | `iterative_refined` | `0.196px` |
| `view_14_evaluation.png` | 9 | 9 | `ransac_epnp_refined` | `0.295px` |
| `view_15_evaluation.png` | 5 | 5 | `sqpnp_refined` | `0.139px` |

Interpretation:

- The first 10 varied table views are sufficient to recover both usable
  intrinsics and a usable tag layout.
- Every hidden-tag evaluation frame still localizes after both intrinsics and
  layout are estimated from images alone.
- The estimated focal lengths are close to the synthetic ground-truth value
  `950px`.

## Estimation Error Against Render Truth

These absolute errors can only be computed in the synthetic test because the
renderer has a hidden ground-truth layout. The comparison first transforms the
ground-truth layout into the same anchor-tag coordinate frame used by
`estimate_table_layout.py`; otherwise the table-frame origin and yaw gauge would
make a direct coordinate comparison meaningless.

Layout error after anchor alignment:

| Estimation mode | Center mean | Center RMS | Center max | Yaw mean | Yaw RMS | Yaw max | Corner RMS | Corner max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Known intrinsics | `6.64mm` | `7.40mm` | `12.62mm` | `0.054deg` | `0.067deg` | `0.127deg` | `7.40mm` | `12.64mm` |
| Joint intrinsics/layout | `6.94mm` | `7.66mm` | `12.46mm` | `0.053deg` | `0.066deg` | `0.128deg` | `7.66mm` | `12.49mm` |

Worst cases:

- Known-intrinsics center/corner max: tag `15`, about `12.6mm`.
- Known-intrinsics yaw max: tag `14`, about `0.127deg`.
- Joint-estimation center/corner max: tag `15`, about `12.6mm`.
- Joint-estimation yaw max: tag `14`, about `0.128deg`.

Jointly estimated camera error against the synthetic camera:

| Parameter | Estimate | Ground truth | Error |
| --- | ---: | ---: | ---: |
| `fx` | `949.9099` | `950.0000` | `-0.0901px` |
| `fy` | `950.1600` | `950.0000` | `0.1600px` |
| `cx` | `640.6537` | `640.0000` | `0.6537px` |
| `cy` | `451.0351` | `450.0000` | `1.0351px` |

Interpretation:

- The estimated relative tag layout is within roughly `1.3cm` worst-case
  translation error in this synthetic setup.
- The focal length error is below `0.02%`.
- Distortion is fixed to zero by assumption.

## What To Inspect Visually

Open these overlays to inspect detection and pose axes:

- `outputs/multi_tag_render_test/annotated/view_10_evaluation_known_intrinsics.png`
- `outputs/multi_tag_render_test/annotated/view_11_evaluation_known_intrinsics.png`
- `outputs/multi_tag_render_test/annotated/view_12_evaluation_known_intrinsics.png`
- `outputs/multi_tag_render_test/annotated/view_13_evaluation_known_intrinsics.png`
- `outputs/multi_tag_render_test/annotated/view_14_evaluation_known_intrinsics.png`
- `outputs/multi_tag_render_test/annotated/view_15_evaluation_known_intrinsics.png`

For CLI-generated overlays:

- `outputs/multi_tag_render_test/localize_estimated_layout_known_annotated/view_10_evaluation_pose.png`
- `outputs/multi_tag_render_test/localize_estimated_layout_known_annotated/view_11_evaluation_pose.png`
- `outputs/multi_tag_render_test/localize_estimated_layout_selfcalib_annotated/view_10_evaluation_pose.png`
- `outputs/multi_tag_render_test/localize_estimated_layout_selfcalib_annotated/view_11_evaluation_pose.png`

Expected visual behavior:

- Detected tags should have marker outlines and IDs.
- The coordinate axes should be drawn on the table plane.
- Evaluation frames should show gray occluders covering complete tags.
- The top-down preview should show single-tag pages in irregular positions and
  rotations.

## Conclusion

The irregular multi-tag synthetic test passed. It confirms that:

- Multiple one-tag pages can be generated.
- An unknown irregular table layout can be initialized from individual tag
  observations and refined as one `BoardLayout`.
- The existing detector, calibration, layout-estimation, and pose-estimation
  pipeline works with non-grid tag placement.
- Known-intrinsics localization remains accurate after estimating the layout
  from the first 10 views.
- No-intrinsics mode can jointly optimize camera intrinsics, calibration-view
  camera poses, and tag layout from the first 10 varied views, then recover
  valid poses for subsequent frames.
- Later frames can tolerate complete tag hiding when enough other tags remain
  visible.
