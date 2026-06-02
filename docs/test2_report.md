# test2 Multi-Camera AprilTag Test Report

本报告记录 `data/test2` 三个 episode、三路相机视频的 AprilTag 检测、基于已知 layout 的相机定位，以及可视化视频输出结果。

## 输入数据

- 数据目录：`data/test2`
- Episode：
  - `20260528_165516_564`
  - `20260528_165623_696`
  - `20260528_165715_753`
- 相机角色：`head`, `left_wrist`, `right_wrist`
- 输入视频：各 episode 下的 `{role}/color_h264.mp4`
- 视频分辨率：`1280 x 720`
- 视频帧率：`30 FPS`
- AprilTag family：`DICT_APRILTAG_36h11`

## Layout 与相机内参

按要求，本次测试继续使用已有 iPhone 多 tag layout：

```text
outputs/iphone_15_mt/estimated_layout_selfcalib.json
```

该 layout 使用 tag size `0.072m`，包含 12 个 tag：

```text
6, 7, 8, 9, 10, 11, 12, 14, 16, 66, 67, 68
```

每个 episode 使用自身目录下的 `camera_intrinsics.json`。本次只处理 RGB 视频，因此使用各相机角色的 `color` 内参，并转换为 OpenCV camera JSON：

```text
outputs/test2/<episode>/cameras/<role>_color_camera.json
```

## 输出文件

- 总摘要：`outputs/test2/test2_summary.json`
- 每个 episode 摘要：`outputs/test2/<episode>/summary.json`
- 每路视频逐帧结果：`outputs/test2/<episode>/<role>_frame_results.json`
- 每路相机内参转换结果：`outputs/test2/<episode>/cameras/<role>_color_camera.json`
- 每路可视化视频：`outputs/test2/<episode>/annotated/<role>_annotated.mp4`
- 每路可视化摘要：`outputs/test2/<episode>/annotated/<role>_annotated_summary.json`

可视化视频由 `annotate_table_video.py` 生成，包含 AprilTag 检测框、估计 layout 的投影边框、table frame 坐标轴和每帧状态文本。失败帧会显示红色 `pose failed` 状态；summary 完整记录 `zero_detection_frames`、`zero_detection_frame_ranges`、`failed_frames`、`failed_frame_ranges` 和 `nonzero_detection_failed_frames`。

## 总体结果

共处理 `9` 路视频、`3606` 帧：

```text
total_frames               = 3606
pose_success_frames        = 1960
pose_failed_frames         = 1646
zero_detection_frames      = 1646
```

本次所有 pose failed 帧都等价于 0-tag 检测失败帧：

```text
detected_tags == 0 的失败帧 = 1646
检测到 tag 但 PnP 失败的帧 = 0
```

成功定位帧的整体重投影 RMS：

```text
rms_px_min    = 0.014624
rms_px_max    = 3.308554
rms_px_mean   = 0.937119
rms_px_median = 0.831845
```

## Episode 结果

| Episode | Role | Frames | Success | Detection failed | Detected tags mean | RMS mean px | RMS max px |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `20260528_165516_564` | `head` | 301 | 299 | 2 | 6.369 | 0.867 | 2.217 |
| `20260528_165516_564` | `left_wrist` | 301 | 62 | 239 | 0.229 | 0.788 | 2.145 |
| `20260528_165516_564` | `right_wrist` | 301 | 78 | 223 | 0.409 | 0.695 | 2.048 |
| `20260528_165623_696` | `head` | 301 | 298 | 3 | 8.558 | 1.241 | 3.309 |
| `20260528_165623_696` | `left_wrist` | 301 | 112 | 189 | 0.754 | 0.847 | 2.670 |
| `20260528_165623_696` | `right_wrist` | 301 | 61 | 240 | 0.359 | 0.731 | 1.411 |
| `20260528_165715_753` | `head` | 600 | 600 | 0 | 9.552 | 1.134 | 2.703 |
| `20260528_165715_753` | `left_wrist` | 600 | 20 | 580 | 0.037 | 0.959 | 1.687 |
| `20260528_165715_753` | `right_wrist` | 600 | 430 | 170 | 1.265 | 0.618 | 2.823 |

## 检测失败帧

这里的“检测失败帧”定义为 AprilTag 检测结果为 0 个 tag 的帧。区间为闭区间 `[start, end]`，使用视频帧号。

### `20260528_165516_564`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 2 | `[261,262]` |
| `left_wrist` | 239 | `[0,21]`, `[23,55]`, `[58,68]`, `[70,74]`, `[108,108]`, `[122,125]`, `[135,157]`, `[161,300]` |
| `right_wrist` | 223 | `[59,59]`, `[62,92]`, `[96,103]`, `[109,220]`, `[224,294]` |

### `20260528_165623_696`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 3 | `[118,118]`, `[272,273]` |
| `left_wrist` | 189 | `[79,115]`, `[128,136]`, `[138,158]`, `[160,228]`, `[243,244]`, `[249,249]`, `[251,300]` |
| `right_wrist` | 240 | `[10,10]`, `[14,26]`, `[28,52]`, `[55,80]`, `[84,95]`, `[98,122]`, `[137,146]`, `[148,170]`, `[176,227]`, `[231,231]`, `[240,275]`, `[281,292]`, `[297,300]` |

### `20260528_165715_753`

| Role | Detection failed count | Failed frame ranges |
| --- | ---: | --- |
| `head` | 0 | none |
| `left_wrist` | 580 | `[0,6]`, `[8,27]`, `[29,31]`, `[33,38]`, `[40,41]`, `[45,45]`, `[54,54]`, `[57,57]`, `[61,599]` |
| `right_wrist` | 170 | `[3,5]`, `[128,144]`, `[228,229]`, `[232,232]`, `[236,241]`, `[248,300]`, `[302,302]`, `[319,324]`, `[338,339]`, `[358,409]`, `[552,555]`, `[568,575]`, `[577,582]`, `[584,584]`, `[587,592]`, `[595,596]` |

完整失败帧列表在各自的逐帧 JSON 和 annotated summary 里：

```text
outputs/test2/<episode>/<role>_frame_results.json
outputs/test2/<episode>/annotated/<role>_annotated_summary.json
```

## 检测分布

### `20260528_165516_564`

| Role | Detection count histogram |
| --- | --- |
| `head` | `0:2, 1:1, 2:1, 4:1, 5:29, 6:142, 7:87, 8:38` |
| `left_wrist` | `0:239, 1:55, 2:7` |
| `right_wrist` | `0:223, 1:38, 2:35, 3:5` |

### `20260528_165623_696`

| Role | Detection count histogram |
| --- | --- |
| `head` | `0:3, 1:1, 2:2, 3:2, 4:5, 5:2, 6:15, 7:45, 8:23, 9:95, 10:97, 11:11` |
| `left_wrist` | `0:189, 1:50, 2:29, 3:13, 4:20` |
| `right_wrist` | `0:240, 1:34, 2:8, 3:18, 4:1` |

### `20260528_165715_753`

| Role | Detection count histogram |
| --- | --- |
| `head` | `2:2, 3:3, 4:6, 5:5, 6:4, 7:4, 8:13, 9:172, 10:336, 11:55` |
| `left_wrist` | `0:580, 1:19, 3:1` |
| `right_wrist` | `0:170, 1:192, 2:147, 3:91` |

## 可视化视频

本次生成的 9 个可视化视频：

```text
outputs/test2/20260528_165516_564/annotated/head_annotated.mp4
outputs/test2/20260528_165516_564/annotated/left_wrist_annotated.mp4
outputs/test2/20260528_165516_564/annotated/right_wrist_annotated.mp4
outputs/test2/20260528_165623_696/annotated/head_annotated.mp4
outputs/test2/20260528_165623_696/annotated/left_wrist_annotated.mp4
outputs/test2/20260528_165623_696/annotated/right_wrist_annotated.mp4
outputs/test2/20260528_165715_753/annotated/head_annotated.mp4
outputs/test2/20260528_165715_753/annotated/left_wrist_annotated.mp4
outputs/test2/20260528_165715_753/annotated/right_wrist_annotated.mp4
```

## 结论

- 已按要求使用 `outputs/iphone_15_mt/estimated_layout_selfcalib.json` 作为 layout。
- 已使用每个 episode 的 `camera_intrinsics.json` 中对应 `color` 相机内参。
- 已生成全部 9 路视频的可视化输出，并完整保留失败帧列表与失败区间。
- `head` 相机整体表现较好，但前两个 episode 各有少量 0-tag 帧；第三个 episode 的 `head` 全部成功定位。
- 腕部相机存在大量 0-tag 帧，尤其 `20260528_165715_753/left_wrist`，只有 `20/600` 帧成功定位。
- 所有定位失败都来自检测失败；没有检测到 tag 后仍然 PnP 失败的情况。
- 成功定位帧整体 RMS 均值为 `0.937px`，能检测到 tag 的帧几何一致性较好。
