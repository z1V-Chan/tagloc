# iPhone 15 Multi-Tag Real Image Test Report

本报告记录 `data/iphone_15_mt` 真实图片上的不规则多 AprilTag layout 估计、无内参自标定、全量相机外参定位，以及 3D 可视化导出结果。

## 输入数据

- 图片目录：`data/iphone_15_mt`
- 图片数量：`22`
- 图片分辨率：`1707 x 1280`
- AprilTag family：`DICT_APRILTAG_36h11`
- 检测到的 tag id：`6, 7, 8, 9, 10, 11, 12, 14, 16, 66, 67, 68`

## 尺寸与模式

本次没有提供相机内参，且全部 `22` 张图片都用于内参和 layout 的联合估计。

`estimate_table_layout.py` 需要 tag 物理边长来确定米制尺度。本次使用当前测量结果：

```text
72.0mm = 0.072m
```

如果后续 tag 尺寸测量更新，估计出的米制 layout 和相机平移会按尺寸线性缩放；重投影误差和像素内参仍可作为几何一致性参考。

当前真实图片流程假设输入图片已经无畸变，因此畸变参数固定为零。

## 复现命令

从仓库根目录运行：

```bash
cd /ssd1/czw_new/tagloc
mkdir -p outputs/iphone_15_mt
```

使用全部 22 张图片估计 layout 和相机内参：

```bash
conda run -n rec python estimate_table_layout.py \
  --tag-size-m 0.072 \
  --calibration-count 22 \
  --min-camera-observations 20 \
  --max-optimizer-iterations 120 \
  --output outputs/iphone_15_mt/estimated_layout_selfcalib.json \
  --save-estimated-camera outputs/iphone_15_mt/estimated_camera.json \
  --summary outputs/iphone_15_mt/estimated_layout_summary.json \
  data/iphone_15_mt/{1..22}.jpg
```

使用估计出的 layout 和 camera 对全部 22 张图片定位：

```bash
conda run -n rec python localize_camera.py \
  --board outputs/iphone_15_mt/estimated_layout_selfcalib.json \
  --camera outputs/iphone_15_mt/estimated_camera.json \
  --output outputs/iphone_15_mt/localization_selfcalib.json \
  --annotate-dir outputs/iphone_15_mt/annotated \
  --axis-length-m 0.06 \
  --ransac-error-px 6.0 \
  --max-reprojection-error-px 12.0 \
  data/iphone_15_mt/{1..22}.jpg
```

导出 3D PLY：

```bash
conda run -n rec python export_scene.py \
  --localization outputs/iphone_15_mt/localization_selfcalib.json \
  --board outputs/iphone_15_mt/estimated_layout_selfcalib.json \
  --output outputs/iphone_15_mt/scene_tags_cameras.ply \
  --camera-depth-m 0.06 \
  --axis-length-m 0.06 \
  --tag-grid 8
```

导出静态 3D PNG 预览：

```bash
conda run -n rec python render_3d_preview.py \
  --localization outputs/iphone_15_mt/localization_selfcalib.json \
  --board outputs/iphone_15_mt/estimated_layout_selfcalib.json \
  --output outputs/iphone_15_mt/scene_overview.png \
  --camera-depth-m 0.06 \
  --max-cameras 80 \
  --dpi 180
```

## 输出文件

- `outputs/iphone_15_mt/estimated_layout_selfcalib.json`：估计出的 12-tag 平面 layout。
- `outputs/iphone_15_mt/estimated_camera.json`：估计出的相机内参。
- `outputs/iphone_15_mt/estimated_layout_summary.json`：layout 和内参估计摘要。
- `outputs/iphone_15_mt/localization_selfcalib.json`：22 张图片的检测与外参结果。
- `outputs/iphone_15_mt/annotated/*_pose.png`：逐帧检测框与坐标轴叠加图，共 `22` 张。
- `outputs/iphone_15_mt/scene_tags_cameras.ply`：3D 可视化场景。
- `outputs/iphone_15_mt/scene_overview.png`：静态 3D 总览图。

PLY 内容统计：

```text
vertices = 3372
faces    = 879
edges    = 297
```

## Layout 与内参估计结果

全部 22 张图片参与 layout 和内参估计，共 `264` 个 tag 观测。

内参初始化：

```text
calibrateCamera RMS = 0.326850 px
observation_count   = 264
dist_coeffs         = [0, 0, 0, 0, 0]
```

联合优化结果：

```text
optimized_intrinsics        = true
view_count                  = 22
observation_count           = 264
initial_reprojection_rms_px = 4.387252
final_reprojection_rms_px   = 0.665092
optimizer_nfev              = 52
termination                 = ftol
```

最终相机矩阵：

```text
[[1222.1711482104624, 0.0, 852.1148673527604],
 [0.0, 1222.805335725888, 651.6403801414675],
 [0.0, 0.0, 1.0]]
```

最终畸变参数：

```text
[0, 0, 0, 0, 0]
```

估计出的 tag layout 坐标以 tag `6` 为 anchor，单位为米：

| Tag | x | y | yaw deg |
| ---: | ---: | ---: | ---: |
| 6 | `0.000000` | `0.000000` | `0.000` |
| 7 | `0.382585` | `0.392587` | `-179.755` |
| 8 | `0.142095` | `0.002132` | `90.822` |
| 9 | `0.265443` | `0.000288` | `-179.784` |
| 10 | `0.261539` | `0.389580` | `-90.221` |
| 11 | `0.382272` | `0.002021` | `91.603` |
| 12 | `0.380509` | `0.129040` | `-89.549` |
| 14 | `0.000668` | `0.393543` | `179.304` |
| 16 | `0.384030` | `0.265397` | `0.576` |
| 66 | `0.141236` | `0.390066` | `-90.380` |
| 67 | `-0.000350` | `0.141712` | `88.321` |
| 68 | `0.002336` | `0.269754` | `-1.543` |

## 检测统计

全量 22 张图的检测结果：

```text
detected_tag_total = 264
detected_per_image_min = 12
detected_per_image_max = 12
detected_per_image_mean = 12.0000
```

每个 tag 都在 22 张图中被检测到：

| Tag | Detected frames |
| ---: | ---: |
| 6 | 22 |
| 7 | 22 |
| 8 | 22 |
| 9 | 22 |
| 10 | 22 |
| 11 | 22 |
| 12 | 22 |
| 14 | 22 |
| 16 | 22 |
| 66 | 22 |
| 67 | 22 |
| 68 | 22 |

## 外参定位结果

全部 22 张图片均成功估计相机外参：

```text
success_frames = 22 / 22
failed_frames  = 0
```

重投影误差统计：

```text
rms_px_min    = 0.775369
rms_px_max    = 1.165137
rms_px_mean   = 0.932618
rms_px_median = 0.953084

mean_px_min   = 0.686852
mean_px_max   = 1.076018
mean_px_mean  = 0.826880

max_px_min    = 1.564212
max_px_max    = 3.020084
max_px_mean   = 2.092129
```

RMS 较大的帧：

| Image | Detected tags | RMS px | Mean px | Max px |
| --- | ---: | ---: | ---: | ---: |
| `7.jpg` | 12 | `1.1651` | `1.0760` | `2.0567` |
| `1.jpg` | 12 | `1.1503` | `1.0146` | `2.5793` |
| `18.jpg` | 12 | `1.0975` | `0.9618` | `2.2238` |
| `14.jpg` | 12 | `1.0476` | `0.9369` | `2.3840` |
| `19.jpg` | 12 | `1.0086` | `0.9000` | `2.8897` |

相机中心在估计 layout 坐标系下的范围，单位为米：

```text
min_xyz  = [-0.165921, -0.645622, 0.568543]
max_xyz  = [ 0.989066,  0.931554, 0.662474]
mean_xyz = [ 0.468133,  0.209645, 0.603034]
```

## 结论

- 已在无内参模式下使用全部 `22` 张图片完成内参和 12-tag layout 联合估计。
- 联合优化后 calibration/layout 重投影 RMS 为 `0.665px`。
- 使用估计出的内参和 layout 回投定位，全部 `22/22` 张图片成功，逐帧定位 RMS 均值为 `0.933px`。
- 所有帧都检测到完整 12 个 tag，数据几何约束充足。
- 已导出 3D 结果：
  - `outputs/iphone_15_mt/scene_tags_cameras.ply`
  - `outputs/iphone_15_mt/scene_overview.png`

限制：

- tag 边长使用当前测量值 `0.072m`；若后续测量更新，米制结果需要按实际尺寸重新估计或缩放。
- 当前流程固定畸变为零，若图片没有提前去畸变，真实镜头畸变会进入 layout 和外参误差。
- 本报告没有外部真实 layout 或相机轨迹真值，因此误差评估主要依据重投影误差和多帧几何一致性。
