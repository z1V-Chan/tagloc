# test1 Multi-Camera AprilTag Test Report

本报告记录 `data/test1` 三个 episode、三路相机视频的 AprilTag 检测与基于已知 layout 的相机定位结果。

## 输入数据

- 数据目录：`data/test1`
- Episode：
  - `20260528_164701_946`
  - `20260528_165240_355`
  - `20260528_165439_173`
- 相机角色：`head`, `left_wrist`, `right_wrist`
- 输入视频：各 episode 下的 `{role}/color_h264.mp4`
- 视频分辨率：`1280 x 720`
- 视频帧率：`30 FPS`
- AprilTag family：`DICT_APRILTAG_36h11`

## Layout 与相机内参

按要求，本次测试使用已有 iPhone 多 tag layout：

```text
outputs/iphone_15_mt/estimated_layout_selfcalib.json
```

该 layout 使用 tag size `0.072m`，包含 12 个 tag：

```text
6, 7, 8, 9, 10, 11, 12, 14, 16, 66, 67, 68
```

每个 episode 都使用自身目录下的 `camera_intrinsics.json`。本次只处理 RGB 视频，因此使用各相机角色的 `color` 内参，并转换为 OpenCV camera JSON：

```text
outputs/test1/<episode>/cameras/<role>_color_camera.json
```

## 输出文件

- 总摘要：`outputs/test1/test1_summary.json`
- 每个 episode 摘要：`outputs/test1/<episode>/summary.json`
- 每路视频逐帧结果：`outputs/test1/<episode>/<role>_frame_results.json`
- 每路相机内参转换结果：`outputs/test1/<episode>/cameras/<role>_color_camera.json`
- 每路可视化视频：`outputs/test1/<episode>/annotated/<role>_annotated.mp4`
- 每路可视化摘要：`outputs/test1/<episode>/annotated/<role>_annotated_summary.json`

逐帧结果中包含：

- `detected_tags`
- `detected_ids`
- `pose_success`
- `pose_reason`
- `used_tag_ids`
- `reprojection_*_px`

可视化视频由 `annotate_table_video.py` 生成。视频中包含 AprilTag 检测框、估计 layout 的投影边框、table frame 坐标轴，以及每帧状态文本。失败帧会显示红色 `pose failed` 状态；对应 summary 完整记录 `zero_detection_frames`、`zero_detection_frame_ranges`、`failed_frames`、`failed_frame_ranges` 和 `nonzero_detection_failed_frames`。

生成命令模板：

```bash
conda run -n rec python annotate_table_video.py \
  data/test1/<episode>/<role>/color_h264.mp4 \
  --board outputs/iphone_15_mt/estimated_layout_selfcalib.json \
  --camera outputs/test1/<episode>/cameras/<role>_color_camera.json \
  --output outputs/test1/<episode>/annotated/<role>_annotated.mp4 \
  --summary outputs/test1/<episode>/annotated/<role>_annotated_summary.json \
  --axis-length-m 0.06 \
  --ransac-error-px 6.0 \
  --max-reprojection-error-px 12.0
```

本次生成的 9 个可视化视频：

```text
outputs/test1/20260528_164701_946/annotated/head_annotated.mp4
outputs/test1/20260528_164701_946/annotated/left_wrist_annotated.mp4
outputs/test1/20260528_164701_946/annotated/right_wrist_annotated.mp4
outputs/test1/20260528_165240_355/annotated/head_annotated.mp4
outputs/test1/20260528_165240_355/annotated/left_wrist_annotated.mp4
outputs/test1/20260528_165240_355/annotated/right_wrist_annotated.mp4
outputs/test1/20260528_165439_173/annotated/head_annotated.mp4
outputs/test1/20260528_165439_173/annotated/left_wrist_annotated.mp4
outputs/test1/20260528_165439_173/annotated/right_wrist_annotated.mp4
```

## 总体结果

共处理 `9` 路视频、`1788` 帧：

```text
total_frames               = 1788
pose_success_frames        = 1182
pose_failed_frames         = 606
zero_detection_frames      = 606
```

本次所有 pose failed 帧都等价于 0-tag 检测失败帧：

```text
detected_tags == 0 的失败帧 = 606
检测到 tag 但 PnP 失败的帧 = 0
```

成功定位帧的整体重投影 RMS：

```text
rms_px_min    = 0.075507
rms_px_max    = 2.886772
rms_px_mean   = 0.872601
rms_px_median = 0.808112
```

## Episode 结果

| Episode | Role | Frames | Success | Detection failed | Detected tags mean | RMS mean px | RMS max px |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `20260528_164701_946` | `head` | 150 | 150 | 0 | 7.653 | 1.079 | 2.399 |
| `20260528_164701_946` | `left_wrist` | 150 | 99 | 51 | 0.920 | 0.829 | 2.513 |
| `20260528_164701_946` | `right_wrist` | 150 | 77 | 73 | 0.720 | 0.693 | 2.478 |
| `20260528_165240_355` | `head` | 295 | 295 | 0 | 7.864 | 0.924 | 2.270 |
| `20260528_165240_355` | `left_wrist` | 295 | 107 | 188 | 0.847 | 1.185 | 2.887 |
| `20260528_165240_355` | `right_wrist` | 295 | 108 | 187 | 0.508 | 0.623 | 2.363 |
| `20260528_165439_173` | `head` | 151 | 151 | 0 | 5.623 | 0.685 | 2.083 |
| `20260528_165439_173` | `left_wrist` | 151 | 119 | 32 | 1.219 | 0.944 | 2.298 |
| `20260528_165439_173` | `right_wrist` | 151 | 76 | 75 | 0.781 | 0.683 | 0.994 |

## 检测失败帧

这里的“检测失败帧”定义为 AprilTag 检测结果为 0 个 tag 的帧。区间为闭区间 `[start, end]`，使用视频帧号。

### `20260528_164701_946`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 0 | none |
| `left_wrist` | 51 | `[72,74]`, `[100,100]`, `[102,134]`, `[136,149]` |
| `right_wrist` | 73 | `[4,40]`, `[44,79]` |

### `20260528_165240_355`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 0 | none |
| `left_wrist` | 188 | `[103,104]`, `[106,106]`, `[108,141]`, `[144,294]` |
| `right_wrist` | 187 | `[12,54]`, `[56,61]`, `[71,92]`, `[149,149]`, `[157,166]`, `[172,226]`, `[228,251]`, `[260,285]` |

### `20260528_165439_173`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 0 | none |
| `left_wrist` | 32 | `[56,56]`, `[119,146]`, `[148,150]` |
| `right_wrist` | 75 | `[45,62]`, `[65,77]`, `[79,86]`, `[88,91]`, `[93,119]`, `[146,150]` |

完整失败帧列表在各自的逐帧 JSON 里：

```text
outputs/test1/<episode>/<role>_frame_results.json
```

## 检测分布

### `20260528_164701_946`

| Role | Detection count histogram |
| --- | --- |
| `head` | `3:2, 4:2, 5:1, 6:7, 7:38, 8:79, 9:21` |
| `left_wrist` | `0:51, 1:60, 2:39` |
| `right_wrist` | `0:73, 1:49, 2:26, 3:1, 4:1` |

### `20260528_165240_355`

| Role | Detection count histogram |
| --- | --- |
| `head` | `6:6, 7:108, 8:101, 9:80` |
| `left_wrist` | `0:188, 1:20, 2:48, 3:22, 4:17` |
| `right_wrist` | `0:187, 1:92, 2:6, 3:1, 4:2, 5:7` |

### `20260528_165439_173`

| Role | Detection count histogram |
| --- | --- |
| `head` | `4:2, 5:62, 6:78, 7:9` |
| `left_wrist` | `0:32, 1:57, 2:59, 3:3` |
| `right_wrist` | `0:75, 1:34, 2:42` |

## 结论

- 已按要求使用 `outputs/iphone_15_mt/estimated_layout_selfcalib.json` 作为 layout。
- 已使用每个 episode 的 `camera_intrinsics.json` 中对应 `color` 相机内参。
- 已用 `annotate_table_video.py` 生成全部 9 路视频的可视化输出，并完整保留失败帧列表与失败区间。
- `head` 相机三段视频全部成功定位，没有检测失败帧。
- 腕部相机存在较多 0-tag 帧，尤其在视频后段，导致对应帧无法定位。
- 所有定位失败都来自检测失败；没有检测到 tag 后仍然 PnP 失败的情况。
- 成功定位帧整体 RMS 均值为 `0.873px`，说明使用该 layout 和内参时，能检测到 tag 的帧几何一致性较好。
