# iPad Mini 7 Real Image Test Report

本报告记录 `data/ipad_mini_7` 真实拍摄图片的 AprilTag 检测、相机自标定、相机外参估计，以及 3D 可视化 PLY 导出结果。

## 输入数据

- 图片目录：`data/ipad_mini_7`
- 图片数量：8
- 图片分辨率：`4032 x 3024`
- 图片文件：
  - `IMG_0488.jpeg`
  - `IMG_0489.jpeg`
  - `IMG_0490.jpeg`
  - `IMG_0491.jpeg`
  - `IMG_0492.jpeg`
  - `IMG_0493.jpeg`
  - `IMG_0494.jpeg`
  - `IMG_0495.jpeg`

## 实测尺寸

使用最小单位 `0.02mm` 的游标卡尺测得 Tag 宽度：

```text
60.94mm = 0.06094m
```

当前 `outputs/board/board_layout.json` 中的 nominal Tag 宽度为：

```text
45.00mm = 0.045m
```

因此本次定位使用统一缩放：

```text
scale = 0.06094 / 0.045 = 1.3542222222
```

注意：这等价于假设整张模板按同一比例缩放。若真实打印存在 X/Y 非等比拉伸，只测一个 Tag 宽度无法修正该误差。

## 复现命令

从仓库根目录运行：

```bash
cd /ssd1/czw_new/tagloc
```

运行自标定和外参估计：

```bash
rm -rf outputs/ipad_mini_7

conda run -n rec python localize_camera.py \
  --board outputs/board/board_layout.json \
  --tag-size-m 0.06094 \
  --calibration-count 8 \
  --min-calibration-views 5 \
  --output outputs/ipad_mini_7/localization_selfcalib.json \
  --save-estimated-camera outputs/ipad_mini_7/estimated_camera.json \
  --annotate-dir outputs/ipad_mini_7/annotated \
  data/ipad_mini_7
```

导出 3D 可视化 PLY：

```bash
conda run -n rec python export_scene.py \
  --localization outputs/ipad_mini_7/localization_selfcalib.json \
  --output outputs/ipad_mini_7/scene_markers_cameras.ply \
  --camera-depth-m 0.12 \
  --axis-length-m 0.08 \
  --tag-grid 8
```

PLY 可用 MeshLab、CloudCompare、Open3D 等工具打开。

## 输出文件

- `outputs/ipad_mini_7/localization_selfcalib.json`：检测、自标定、逐帧外参结果。
- `outputs/ipad_mini_7/estimated_camera.json`：从 8 张图估计出的相机内参。
- `outputs/ipad_mini_7/annotated/*_pose.png`：检测框和坐标轴叠加图。
- `outputs/ipad_mini_7/scene_markers_cameras.ply`：3D 可视化场景。

PLY 内容：

- 白色矩形：缩放后的 A4 纸面。
- 黑白网格：12 个 AprilTag 的实际图案样式，默认每个 Tag 用 `16 x 16` 个小面片近似。若要精细化，可以修改参数为 `--tag-grid 16` 或更高。
- 纸面原点坐标轴：红色 X、绿色 Y、蓝色 Z。
- 彩色视锥：每张图片对应的相机外参。
- 每个相机位置也带局部坐标轴：红色 X、绿色 Y、蓝色 Z。

## 检测结果

8 张图片全部检测到 12 个 Tag，且所有 Tag 都参与了外参估计。

| Image | Detected tags | Used tags |
| --- | ---: | --- |
| `IMG_0488.jpeg` | 12 | `0-11` |
| `IMG_0489.jpeg` | 12 | `0-11` |
| `IMG_0490.jpeg` | 12 | `0-11` |
| `IMG_0491.jpeg` | 12 | `0-11` |
| `IMG_0492.jpeg` | 12 | `0-11` |
| `IMG_0493.jpeg` | 12 | `0-11` |
| `IMG_0494.jpeg` | 12 | `0-11` |
| `IMG_0495.jpeg` | 12 | `0-11` |

## 自标定结果

目录中没有提供独立相机内参文件，因此本次使用 8 张图进行自标定。

```text
success = true
valid_view_count = 8
calibration_rms_px = 0.7829769002
```

估计相机矩阵：

```text
[[3324.3052118148444, 0.0, 1993.0142352186233],
 [0.0, 3319.8852471974324, 1487.2077361807742],
 [0.0, 0.0, 1.0]]
```

估计畸变参数：

```text
[0.1286481076, -0.2861833713, 0.0, 0.0, 0.0]
```

## 外参估计结果

表中的 `camera_center_xyz_m` 是相机中心在纸面世界坐标系下的位置，单位为米。世界坐标系定义为：纸心为原点，`+X` 向右，`+Y` 向上，`+Z` 朝纸面外。

| Image | Solver | RMS px | Mean px | Max px | camera_center_xyz_m |
| --- | --- | ---: | ---: | ---: | --- |
| `IMG_0488.jpeg` | `sqpnp_refined` | `0.8723` | `0.7549` | `2.8497` | `[-0.2816, -0.0451, 0.6937]` |
| `IMG_0489.jpeg` | `ippe_1_refined` | `0.6186` | `0.5593` | `1.2036` | `[-0.1984, -0.0128, 1.0228]` |
| `IMG_0490.jpeg` | `ippe_1_refined` | `0.7194` | `0.6718` | `1.2102` | `[-0.1944, 0.3297, 1.0694]` |
| `IMG_0491.jpeg` | `sqpnp_refined` | `0.8017` | `0.7467` | `1.4715` | `[-0.0471, 0.2173, 0.9884]` |
| `IMG_0492.jpeg` | `iterative_refined` | `0.6841` | `0.6018` | `1.5883` | `[-0.2768, 0.3799, 0.4646]` |
| `IMG_0493.jpeg` | `ippe_0_refined` | `0.7975` | `0.7072` | `1.7219` | `[-0.1375, 0.3956, 0.5544]` |
| `IMG_0494.jpeg` | `ippe_0_refined` | `0.8756` | `0.7797` | `2.5306` | `[-0.1584, 0.1593, 0.5552]` |
| `IMG_0495.jpeg` | `iterative_refined` | `0.8545` | `0.7902` | `1.4813` | `[-0.2318, 0.0574, 0.5544]` |

重投影 RMS 统计：

```text
min = 0.6186 px
max = 0.8756 px
mean = 0.7779 px
```

相机中心范围：

```text
min_xyz = [-0.2816, -0.0451, 0.4646]
max_xyz = [-0.0471,  0.3956, 1.0694]
mean_xyz = [-0.1908,  0.1852, 0.7379]
```

## 结论

- 8 张真实图片全部成功检测到完整 12 个 Tag。
- 自标定成功，标定 RMS 为 `0.783px`。
- 所有图片外参估计成功，逐帧重投影 RMS 在 `0.619px` 到 `0.876px` 之间。
- 已导出 `outputs/ipad_mini_7/scene_markers_cameras.ply`，可用于检查 Tag 平面和相机轨迹的 3D 空间关系。

本次没有外部真值轨迹或独立相机标定结果，因此报告中的精度指标主要是重投影误差和多帧位姿几何一致性。若后续需要绝对精度评估，建议使用独立标定内参或高精度位姿真值进行对比。
