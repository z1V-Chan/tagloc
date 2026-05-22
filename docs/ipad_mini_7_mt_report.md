# iPad Mini 7 Multi-Tag Real Image Test Report

本报告记录 `data/ipad_mini_7_mt` 真实图片上的不规则多 AprilTag layout 估计、相机内参估计、全量相机外参定位，以及 3D 可视化导出结果。

## 输入数据

- 图片目录：`data/ipad_mini_7_mt`
- 图片数量：`163`
- 图片分辨率：`1920 x 1080`
- AprilTag family：`DICT_APRILTAG_36h11`
- 检测到的 tag id：`8, 9, 10, 11, 12, 13, 14, 15`

## 实测尺寸

使用最小单位 `0.02mm` 的游标卡尺测得 tag 宽度：

```text
153.34mm = 0.15334m
```

本次所有 layout 估计、相机内参估计和外参定位均使用：

```text
--tag-size-m 0.15334
```

当前 `estimate_table_layout.py` 的真实图片流程假设输入图片已经无畸变，因此畸变参数固定为零。

## 复现命令

从仓库根目录运行：

```bash
cd /ssd1/czw_new/tagloc
rm -rf outputs/ipad_mini_7_mt
```

选择 21 张代表帧估计 layout 和相机内参。这些帧覆盖前段、中段、末段视角，并且覆盖全部 8 个 tag：

```bash
conda run -n rec python estimate_table_layout.py \
  --tag-size-m 0.15334 \
  --calibration-count 21 \
  --min-camera-observations 20 \
  --max-optimizer-iterations 60 \
  --output outputs/ipad_mini_7_mt/estimated_layout_selfcalib.json \
  --save-estimated-camera outputs/ipad_mini_7_mt/estimated_camera.json \
  --summary outputs/ipad_mini_7_mt/estimated_layout_summary.json \
  data/ipad_mini_7_mt/000001.jpg \
  data/ipad_mini_7_mt/000005.jpg \
  data/ipad_mini_7_mt/000010.jpg \
  data/ipad_mini_7_mt/000015.jpg \
  data/ipad_mini_7_mt/000020.jpg \
  data/ipad_mini_7_mt/000025.jpg \
  data/ipad_mini_7_mt/000030.jpg \
  data/ipad_mini_7_mt/000035.jpg \
  data/ipad_mini_7_mt/000040.jpg \
  data/ipad_mini_7_mt/000059.jpg \
  data/ipad_mini_7_mt/000060.jpg \
  data/ipad_mini_7_mt/000063.jpg \
  data/ipad_mini_7_mt/000064.jpg \
  data/ipad_mini_7_mt/000156.jpg \
  data/ipad_mini_7_mt/000157.jpg \
  data/ipad_mini_7_mt/000158.jpg \
  data/ipad_mini_7_mt/000159.jpg \
  data/ipad_mini_7_mt/000160.jpg \
  data/ipad_mini_7_mt/000161.jpg \
  data/ipad_mini_7_mt/000162.jpg \
  data/ipad_mini_7_mt/000163.jpg
```

使用估计出的 layout 和 camera 对全部 163 张图片定位：

```bash
conda run -n rec python localize_camera.py \
  --board outputs/ipad_mini_7_mt/estimated_layout_selfcalib.json \
  --camera outputs/ipad_mini_7_mt/estimated_camera.json \
  --output outputs/ipad_mini_7_mt/localization_selfcalib.json \
  --annotate-dir outputs/ipad_mini_7_mt/annotated \
  --axis-length-m 0.12 \
  --ransac-error-px 6.0 \
  --max-reprojection-error-px 12.0 \
  data/ipad_mini_7_mt
```

导出 3D PLY。这个文件用于 MeshLab、CloudCompare、Open3D 等 3D 工具中交互查看：

```bash
conda run -n rec python export_scene.py \
  --localization outputs/ipad_mini_7_mt/localization_selfcalib.json \
  --board outputs/ipad_mini_7_mt/estimated_layout_selfcalib.json \
  --output outputs/ipad_mini_7_mt/scene_tags_cameras.ply \
  --camera-depth-m 0.12 \
  --axis-length-m 0.12 \
  --tag-grid 8
```

导出静态 3D PNG 预览。这个文件是报告用快照，不包含 PLY 的交互能力：

```bash
conda run -n rec python render_3d_preview.py \
  --localization outputs/ipad_mini_7_mt/localization_selfcalib.json \
  --board outputs/ipad_mini_7_mt/estimated_layout_selfcalib.json \
  --output outputs/ipad_mini_7_mt/scene_overview.png \
  --camera-depth-m 0.12 \
  --max-cameras 80 \
  --dpi 180
```

## 输出文件

- `outputs/ipad_mini_7_mt/estimated_layout_selfcalib.json`：估计出的 8-tag 平面 layout。
- `outputs/ipad_mini_7_mt/estimated_camera.json`：估计出的相机内参。
- `outputs/ipad_mini_7_mt/estimated_layout_summary.json`：layout 和内参估计摘要。
- `outputs/ipad_mini_7_mt/localization_selfcalib.json`：163 张图片的检测与外参结果。
- `outputs/ipad_mini_7_mt/annotated/*_pose.png`：逐帧检测框与坐标轴叠加图，共 `163` 张。
- `outputs/ipad_mini_7_mt/scene_tags_cameras.ply`：3D 可视化场景，可用 MeshLab、CloudCompare、Open3D 等打开。
- `outputs/ipad_mini_7_mt/scene_overview.png`：静态 3D 总览图。

PLY 内容：

- tag 平面：估计出的 8 个 tag 位置和黑白 tag 图案。
- 纸面/世界坐标轴：红色 X、绿色 Y、蓝色 Z。
- 相机轨迹：全部成功定位帧的相机视锥。

PLY 文件统计：

```text
vertices = 6443
faces    = 1968
edges    = 1832
```

## Layout 与内参估计结果

layout 估计使用 21 张代表帧，共 `158` 个单 tag 观测。

内参初始化：

```text
calibrateCamera RMS = 0.398986 px
observation_count   = 158
dist_coeffs         = [0, 0, 0, 0, 0]
```

联合优化结果：

```text
optimized_intrinsics        = true
view_count                  = 21
observation_count           = 158
initial_reprojection_rms_px = 1.964311
final_reprojection_rms_px   = 0.964220
optimizer_nfev              = 50
termination                 = ftol
```

最终相机矩阵：

```text
[[1789.8003039049552, 0.0, 952.1757649727407],
 [0.0, 1842.9122128443946, 625.2445648977709],
 [0.0, 0.0, 1.0]]
```

最终畸变参数：

```text
[0, 0, 0, 0, 0]
```

估计出的 tag layout 坐标以 tag `8` 为 anchor，单位为米：

| Tag | x | y | yaw deg |
| ---: | ---: | ---: | ---: |
| 8 | `0.000000` | `0.000000` | `0.000` |
| 9 | `-0.004240` | `0.570035` | `-89.979` |
| 10 | `0.584350` | `0.255666` | `-89.666` |
| 11 | `0.274735` | `-0.013176` | `-179.951` |
| 12 | `0.558675` | `-0.011876` | `1.290` |
| 13 | `0.240053` | `0.591263` | `178.518` |
| 14 | `-0.000850` | `0.275967` | `-179.423` |
| 15 | `0.576225` | `0.565487` | `-90.657` |

## 检测统计

全量 163 张图的检测结果：

```text
detected_tag_total = 720
detected_per_image_min = 1
detected_per_image_max = 8
detected_per_image_mean = 4.4172
```

每张图检测到的 tag 数量分布：

| Detected tags per image | Frame count |
| ---: | ---: |
| 1 | 14 |
| 2 | 45 |
| 3 | 19 |
| 4 | 15 |
| 5 | 16 |
| 6 | 4 |
| 7 | 5 |
| 8 | 45 |

各 tag 被检测到的帧数：

| Tag | Detected frames |
| ---: | ---: |
| 8 | 84 |
| 9 | 85 |
| 10 | 92 |
| 11 | 94 |
| 12 | 77 |
| 13 | 104 |
| 14 | 100 |
| 15 | 84 |

如果把场景中 8 个 tag 都视为每帧理论存在，则总理论次数为：

```text
163 * 8 = 1304
```

实际检测到 `720` 次，对应未检出占比为：

```text
(1304 - 720) / 1304 = 44.79%
```

注意：这个数包含 tag 不在画面内、被遮挡、边缘裁切、运动模糊等情况，因此不能直接等价为 AprilTag 检测器的 false negative rate。当前数据没有逐帧可见 tag 真值标注，无法区分“可见但漏检”和“实际不可见”。

## 外参定位结果

全部 163 张图片均成功估计相机外参：

```text
success_frames = 163 / 163
failed_frames  = 0
```

用于外参估计的 tag 数量和检测数量一致：

```text
used_tag_count_min  = 1
used_tag_count_max  = 8
used_tag_count_mean = 4.4172
```

重投影误差统计：

```text
rms_px_min    = 0.173591
rms_px_max    = 3.997775
rms_px_mean   = 1.507167
rms_px_median = 1.396184

mean_px_min   = 0.172547
mean_px_max   = 3.466445
mean_px_mean  = 1.337532

max_px_min    = 0.197201
max_px_max    = 9.251187
max_px_mean   = 2.953238
```

RMS 较大的帧：

| Image | Detected tags | RMS px | Mean px | Max px | Used tags |
| --- | ---: | ---: | ---: | ---: | --- |
| `000125.jpg` | 3 | `3.9978` | `3.4664` | `9.2512` | `10,13,15` |
| `000150.jpg` | 3 | `3.3642` | `3.0412` | `5.6710` | `11,13,15` |
| `000137.jpg` | 4 | `3.1323` | `2.8439` | `6.1026` | `8,11,12,14` |
| `000114.jpg` | 2 | `2.8609` | `2.7145` | `3.7864` | `9,14` |
| `000151.jpg` | 4 | `2.8504` | `2.3804` | `5.7586` | `10,11,13,15` |

相机中心在估计 layout 坐标系下的范围，单位为米：

```text
min_xyz  = [-0.698700, -0.784705, 0.419170]
max_xyz  = [ 1.432877,  1.464525, 1.012072]
mean_xyz = [ 0.325554,  0.097436, 0.699436]
```

## 结论

- 使用实测 tag 宽度 `153.34mm` 后，成功从真实多 tag 图像估计出了 8-tag 平面 layout。
- 无内参模式下完成了相机内参与 layout 联合优化，最终 calibration/layout 重投影 RMS 为 `0.964px`。
- 全量 `163/163` 张图片均成功定位，逐帧重投影 RMS 均值为 `1.507px`。
- 已导出 3D 结果：
  - `outputs/ipad_mini_7_mt/scene_tags_cameras.ply`
  - `outputs/ipad_mini_7_mt/scene_overview.png`

限制：

- 当前流程固定畸变为零，若这些图片没有提前去畸变，真实镜头畸变会进入 layout 和外参误差。
- 本报告没有外部真实 layout 或相机轨迹真值，因此误差评估主要依据重投影误差和多帧几何一致性。
- 部分帧只有 1-2 个 tag，可完成 PnP，但姿态约束比多 tag 帧弱，建议应用侧对低 tag 数帧设置置信度门限。
