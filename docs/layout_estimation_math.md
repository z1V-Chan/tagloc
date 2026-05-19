# Layout Estimation Math / Layout 估计数学说明

本文说明 `estimate_table_layout.py` 中不规则多 AprilTag 平面 layout 的估计流程。

## 中文说明

### 问题定义

桌面上有 $M$ 个 AprilTag。所有 tag 位于同一平面，实际边长相同且已知，记为

$$
s \in \mathbb{R}_{>0}.
$$

前 $N$ 张图像用于估计 tag 之间的相对 layout。AprilTag 检测器给出每个被观测 tag 的四个角点像素坐标：

$$
\mathbf{u}_{i,j,k} \in \mathbb{R}^2,
$$

其中：

- $i \in \{1,\dots,N\}$ 是图像编号；
- $j \in \{1,\dots,M\}$ 是 tag id；
- $k \in \{1,2,3,4\}$ 是 tag 角点编号。

当前实现假设输入图像已经无畸变，因此畸变系数固定为零：

$$
\mathbf{d}=\mathbf{0}.
$$

### 未知量

由于所有 tag 共面，每个 tag 的 layout 只包含平面内平移和绕平面法向的旋转：

$$
\boldsymbol{\ell}_j =
\begin{bmatrix}
x_j \\
y_j \\
\theta_j
\end{bmatrix}.
$$

第 $i$ 张图像对应一个相机外参：

$$
T_i = (R_i, \mathbf{t}_i),
\qquad
R_i \in SO(3),\quad \mathbf{t}_i \in \mathbb{R}^3.
$$

代码中 $R_i$ 使用 OpenCV Rodrigues 向量 $\mathbf{r}_i$ 参数化。

如果相机内参未知，则同时估计：

$$
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}.
$$

如果相机内参已知，则 $K$ 固定不参与优化。

### Gauge 固定

仅由图像观测无法确定 layout 在真实桌面坐标系中的绝对平移和绝对 yaw。为消除这个 gauge 自由度，选择一个 anchor tag $a$，并固定：

$$
x_a = 0,\qquad y_a = 0,\qquad \theta_a = 0.
$$

默认 anchor 是所有被观测 tag id 中最小的一个。最终输出的 layout 坐标是在 anchor tag 坐标系下的相对坐标，而不是外部测量得到的桌面绝对坐标。

### Tag 角点模型

在 tag 自身坐标系中，四个角点定义为：

$$
\mathbf{q}_1 =
\begin{bmatrix}
-s/2 \\
s/2 \\
0
\end{bmatrix},
\quad
\mathbf{q}_2 =
\begin{bmatrix}
s/2 \\
s/2 \\
0
\end{bmatrix},
\quad
\mathbf{q}_3 =
\begin{bmatrix}
s/2 \\
-s/2 \\
0
\end{bmatrix},
\quad
\mathbf{q}_4 =
\begin{bmatrix}
-s/2 \\
-s/2 \\
0
\end{bmatrix}.
$$

给定 tag $j$ 的 layout 参数 $\boldsymbol{\ell}_j=(x_j,y_j,\theta_j)$，其第 $k$ 个角点在 layout 平面坐标系中的三维坐标为：

$$
\mathbf{P}_{j,k}(\boldsymbol{\ell}_j)
=
\begin{bmatrix}
x_j \\
y_j \\
0
\end{bmatrix}
+
R_z(\theta_j)\mathbf{q}_k,
$$

其中绕 $z$ 轴旋转矩阵为：

$$
R_z(\theta_j)=
\begin{bmatrix}
\cos\theta_j & -\sin\theta_j & 0 \\
\sin\theta_j & \cos\theta_j & 0 \\
0 & 0 & 1
\end{bmatrix}.
$$

### 投影模型

第 $i$ 张图像中，layout 平面点 $\mathbf{P}_{j,k}$ 先通过相机外参变换到相机坐标系：

$$
\mathbf{X}_{i,j,k}^{c}
=
R_i \mathbf{P}_{j,k} + \mathbf{t}_i
=
\begin{bmatrix}
X \\
Y \\
Z
\end{bmatrix}.
$$

在无畸变假设下，其像素投影为：

$$
\pi(K,T_i,\mathbf{P}_{j,k})
=
\begin{bmatrix}
f_x X/Z + c_x \\
f_y Y/Z + c_y
\end{bmatrix}.
$$

实现中使用 `cv2.projectPoints()`，并传入零畸变系数。

### 优化目标

记所有有效角点观测集合为 $\mathcal{O}$。每个角点观测的二维重投影残差为：

$$
\mathbf{e}_{i,j,k}
=
\pi(K,T_i,\mathbf{P}_{j,k}(\boldsymbol{\ell}_j))
-
\mathbf{u}_{i,j,k}.
$$

也就是：

$$
\mathbf{e}_{i,j,k}
=
\begin{bmatrix}
e^x_{i,j,k} \\
e^y_{i,j,k}
\end{bmatrix}.
$$

代码中传给 `least_squares()` 的残差不是每个角点的范数，而是把所有角点的 $x/y$ 像素误差展平成一个长向量：

$$
\mathbf{f}(\boldsymbol{\eta})
=
\operatorname{vec}
\left(
\left\{
e^x_{i,j,k},
e^y_{i,j,k}
\right\}_{(i,j,k)\in\mathcal{O}}
\right)
\in
\mathbb{R}^{2|\mathcal{O}|}.
$$

其中 $\boldsymbol{\eta}$ 表示当前模式下参与优化的全部参数。

已知内参模式的参数为：

$$
\boldsymbol{\eta}_{\text{known}}
=
\left[
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right],
$$

并求解：

$$
\min_{\boldsymbol{\eta}_{\text{known}}}
F(\boldsymbol{\eta}_{\text{known}})
\quad
\text{s.t.}\quad
\boldsymbol{\ell}_a=(0,0,0),\quad K=\text{known},\quad \mathbf{d}=\mathbf{0}.
$$

无内参模式的参数为：

$$
\boldsymbol{\eta}_{\text{self}}
=
\left[
f_x,f_y,c_x,c_y,
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right],
$$

并求解：

$$
\min_{\boldsymbol{\eta}_{\text{self}}}
F(\boldsymbol{\eta}_{\text{self}})
\quad
\text{s.t.}\quad
\boldsymbol{\ell}_a=(0,0,0),\quad \mathbf{d}=\mathbf{0}.
$$

这里的目标函数 $F$ 和 SciPy `least_squares(loss="soft_l1", f_scale=2.0)` 一致。令 $C=2.0$，对任意参数 $\boldsymbol{\eta}$：

$$
F(\boldsymbol{\eta})
=
\frac{1}{2}C^2
\sum_{m=1}^{2|\mathcal{O}|}
\rho\!\left(
\left(
\frac{f_m(\boldsymbol{\eta})}{C}
\right)^2
\right),
$$

其中 `soft_l1` 的鲁棒损失为：

$$
\rho(z)
=
2\left(\sqrt{1+z}-1\right).
$$

若使用普通最小二乘，则相当于最小化 $\frac{1}{2}\|\mathbf{f}(\boldsymbol{\eta})\|_2^2$。当前实现使用 `soft_l1`，所以较大的检测误差会被降权。

无内参模式下还对内参施加了代码中的边界：

$$
100 \le f_x,f_y \le 10000,\qquad
0 \le c_x \le W,\qquad
0 \le c_y \le H.
$$

其他相机外参和非 anchor tag layout 参数没有显式边界。

因此这不是解析闭式求解，而是数值非线性优化。Jacobian 由 SciPy 默认数值差分计算。

### 初始化流程

非线性优化需要较好的初值，当前流程分四步。

#### 1. 内参初始化

若提供相机文件，则直接使用其中的 $K$。

若未提供相机文件，则先把每次检测到的 tag 当作一个独立的已知尺寸正方形，调用 `cv2.calibrateCamera()` 初始化：

$$
f_x,\ f_y,\ c_x,\ c_y.
$$

畸变仍固定为：

$$
\mathbf{d}=\mathbf{0}.
$$

#### 2. 单 tag 位姿初始化

对每张图中每个检测到的 tag，使用 `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` 估计单 tag 位姿：

$$
T_{c \leftarrow j}^{(i)}.
$$

它表示在第 $i$ 张图像中，tag $j$ 坐标系到相机坐标系的变换。

#### 3. 相对 layout 初始化

这里采用的记号是：$T_{p \leftarrow q}$ 表示把 $q$ 坐标系中的点变换到 $p$ 坐标系。

若某张图同时观测到 anchor tag $a$ 和 tag $j$，则可计算 tag $j$ 相对于 anchor 的初始变换：

$$
T_{a \leftarrow j}^{(i)}
=
\left(T_{c \leftarrow a}^{(i)}\right)^{-1}
T_{c \leftarrow j}^{(i)}.
$$

把这个齐次变换写成：

$$
T_{a \leftarrow j}^{(i)}
=
\begin{bmatrix}
R_{a \leftarrow j}^{(i)} & \mathbf{t}_{a \leftarrow j}^{(i)} \\
\mathbf{0}^T & 1
\end{bmatrix}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & t_x \\
r_{21} & r_{22} & r_{23} & t_y \\
r_{31} & r_{32} & r_{33} & t_z \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

layout 只保留平面内的 $x/y$ 平移和 yaw，因此直接取平移列的前两个分量：

$$
x_j^{(i)} = t_x = T_{a \leftarrow j}^{(i)}[0,3],
\qquad
y_j^{(i)} = t_y = T_{a \leftarrow j}^{(i)}[1,3].
$$

理论上所有 tag 共面时 $t_z=0$。实际单 tag PnP 会有数值噪声，所以初始化时丢弃 $t_z$，后续联合优化仍强制所有 tag 位于同一平面。

yaw 的提取来自旋转矩阵的第一列。第一列表示 tag $j$ 的局部 $x$ 轴在 anchor 坐标系中的方向：

$$
\mathbf{v}_x
=
R_{a \leftarrow j}^{(i)}
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
=
\begin{bmatrix}
r_{11} \\
r_{21} \\
r_{31}
\end{bmatrix}.
$$

因为 layout 模型只允许绕平面法向旋转，代码把这个方向投影到 anchor 的 $xy$ 平面，并计算：

$$
\theta_j^{(i)}
=
\operatorname{atan2}(r_{21}, r_{11})
=
\operatorname{atan2}
\left(
T_{a \leftarrow j}^{(i)}[1,0],
T_{a \leftarrow j}^{(i)}[0,0]
\right).
$$

这和代码中的 `_yaw_from_transform()` 一致：

```python
atan2(transform[1, 0], transform[0, 0])
```

如果某个 tag 没有直接和 anchor 同时出现，代码会通过已经初始化的 source tag $s$ 继续传播初始化。若某张图同时观测到已知 layout 的 tag $s$ 和未知 tag $j$，则先计算：

$$
T_{s \leftarrow j}^{(i)}
=
\left(T_{c \leftarrow s}^{(i)}\right)^{-1}
T_{c \leftarrow j}^{(i)}.
$$

由于 $T_{a \leftarrow s}$ 已知，因此：

$$
T_{a \leftarrow j}^{(i)}
=
T_{a \leftarrow s}
T_{s \leftarrow j}^{(i)}.
$$

之后仍使用同样的规则从 $T_{a \leftarrow j}^{(i)}$ 中提取 $x_j^{(i)},y_j^{(i)},\theta_j^{(i)}$。

同一个 tag 可能有多个初始化样本。代码对平移使用中位数，对角度使用圆均值：

$$
x_j = \operatorname{median}(\{x_j^{(m)}\}),
\qquad
y_j = \operatorname{median}(\{y_j^{(m)}\}),
$$

$$
\theta_j =
\operatorname{atan2}
\left(
\frac{1}{L}\sum_{m=1}^{L}\sin\theta_j^{(m)},
\frac{1}{L}\sum_{m=1}^{L}\cos\theta_j^{(m)}
\right).
$$

#### 4. 每张图相机位姿初始化

有了初始 layout 和当前内参后，对每张 calibration 图像，把该图内所有检测到的 tag 角点合并起来，再通过 PnP 求该图的初始相机外参：

$$
T_i^{(0)}=(R_i^{(0)},\mathbf{t}_i^{(0)}).
$$

这些初值随后进入联合重投影优化。

### 优化变量

已知内参模式下，优化变量为：

$$
\mathbf{x}
=
\left[
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right].
$$

无内参模式下，优化变量为：

$$
\mathbf{x}
=
\left[
f_x,f_y,c_x,c_y,
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right].
$$

anchor tag 的 $\boldsymbol{\ell}_a$ 不在优化变量中，因为它已经固定为原点和零 yaw。

### 输出与后续定位

优化完成后，估计出的 tag layout 会写成普通 `BoardLayout` JSON：

```text
estimated_layout_known.json
estimated_layout_selfcalib.json
```

后续图像定位时，`localize_camera.py` 使用这些估计得到的 layout，而不是使用渲染器的真实 layout 文件。

渲染测试中的真值文件：

```text
outputs/multi_tag_render_test/layout/table_layout.json
```

只用于生成图像和计算误差，不应作为真实工作流中的定位输入。

## English Reference

### Problem

There are $M$ AprilTags on one plane. All tags have the same measured side length:

$$
s \in \mathbb{R}_{>0}.
$$

For the first $N$ images, the detector provides corner observations:

$$
\mathbf{u}_{i,j,k} \in \mathbb{R}^2,
$$

where $i$ is the image index, $j$ is the tag id, and $k\in\{1,2,3,4\}$ is the corner index.

The estimator assumes undistorted images:

$$
\mathbf{d}=\mathbf{0}.
$$

### Unknowns

Each tag is constrained to the table plane:

$$
\boldsymbol{\ell}_j =
\begin{bmatrix}
x_j \\
y_j \\
\theta_j
\end{bmatrix}.
$$

Each calibration image has a camera pose:

$$
T_i=(R_i,\mathbf{t}_i),
\qquad
R_i\in SO(3),\quad \mathbf{t}_i\in\mathbb{R}^3.
$$

If intrinsics are unknown, the optimized camera matrix is:

$$
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}.
$$

Otherwise $K$ is fixed.

### Gauge Fix

The planar layout has arbitrary translation and yaw. One anchor tag $a$ is fixed:

$$
x_a=0,\qquad y_a=0,\qquad \theta_a=0.
$$

The estimated layout is therefore expressed in the anchor tag frame.

### Corner and Projection Models

The four tag-local corners are:

$$
\mathbf{q}_1=(-s/2,s/2,0)^T,\quad
\mathbf{q}_2=(s/2,s/2,0)^T,\quad
\mathbf{q}_3=(s/2,-s/2,0)^T,\quad
\mathbf{q}_4=(-s/2,-s/2,0)^T.
$$

The table-frame corner location is:

$$
\mathbf{P}_{j,k}(\boldsymbol{\ell}_j)
=
\begin{bmatrix}
x_j \\
y_j \\
0
\end{bmatrix}
+
R_z(\theta_j)\mathbf{q}_k.
$$

The camera-frame point is:

$$
\mathbf{X}_{i,j,k}^{c}
=
R_i\mathbf{P}_{j,k}+\mathbf{t}_i
=
(X,Y,Z)^T.
$$

With zero distortion, the projected pixel is:

$$
\pi(K,T_i,\mathbf{P}_{j,k})
=
\begin{bmatrix}
f_xX/Z+c_x \\
f_yY/Z+c_y
\end{bmatrix}.
$$

### Objective

For observation set $\mathcal{O}$, the 2D reprojection residual of one observed corner is:

$$
\mathbf{e}_{i,j,k}
=
\pi(K,T_i,\mathbf{P}_{j,k}(\boldsymbol{\ell}_j))
-
\mathbf{u}_{i,j,k}.
$$

Equivalently:

$$
\mathbf{e}_{i,j,k}
=
\begin{bmatrix}
e^x_{i,j,k} \\
e^y_{i,j,k}
\end{bmatrix}.
$$

The implementation passes a flattened scalar residual vector to `least_squares()`:

$$
\mathbf{f}(\boldsymbol{\eta})
=
\operatorname{vec}
\left(
\left\{
e^x_{i,j,k},
e^y_{i,j,k}
\right\}_{(i,j,k)\in\mathcal{O}}
\right)
\in
\mathbb{R}^{2|\mathcal{O}|}.
$$

Known-intrinsics mode optimizes:

$$
\boldsymbol{\eta}_{\text{known}}
=
\left[
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right],
$$

with:

$$
\min_{\boldsymbol{\eta}_{\text{known}}}
F(\boldsymbol{\eta}_{\text{known}})
\quad
\text{s.t.}\quad
\boldsymbol{\ell}_a=(0,0,0),\quad K=\text{known},\quad \mathbf{d}=\mathbf{0}.
$$

No-intrinsics mode optimizes:

$$
\boldsymbol{\eta}_{\text{self}}
=
\left[
f_x,f_y,c_x,c_y,
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right],
$$

with:

$$
\min_{\boldsymbol{\eta}_{\text{self}}}
F(\boldsymbol{\eta}_{\text{self}})
\quad
\text{s.t.}\quad
\boldsymbol{\ell}_a=(0,0,0),\quad \mathbf{d}=\mathbf{0}.
$$

The objective $F$ matches SciPy `least_squares(loss="soft_l1", f_scale=2.0)`. Let $C=2.0$. For any parameter vector $\boldsymbol{\eta}$:

$$
F(\boldsymbol{\eta})
=
\frac{1}{2}C^2
\sum_{m=1}^{2|\mathcal{O}|}
\rho\!\left(
\left(
\frac{f_m(\boldsymbol{\eta})}{C}
\right)^2
\right),
$$

where the `soft_l1` loss is:

$$
\rho(z)
=
2\left(\sqrt{1+z}-1\right).
$$

With a linear loss this would reduce to $\frac{1}{2}\|\mathbf{f}(\boldsymbol{\eta})\|_2^2$. With `soft_l1`, large scalar pixel residuals are down-weighted.

In no-intrinsics mode, the camera intrinsics are bounded as:

$$
100 \le f_x,f_y \le 10000,\qquad
0 \le c_x \le W,\qquad
0 \le c_y \le H.
$$

This is a robust nonlinear least-squares problem solved by SciPy `least_squares()` with numerical Jacobians.

### Initialization

If a camera file is provided, its $K$ is fixed. Otherwise, `cv2.calibrateCamera()` initializes:

$$
f_x,\ f_y,\ c_x,\ c_y
$$

from independent single-tag observations while keeping distortion at zero.

For every detected tag, `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)` estimates:

$$
T_{c \leftarrow j}^{(i)}.
$$

The notation $T_{p \leftarrow q}$ means a transform from frame $q$ into frame $p$.

When image $i$ observes both anchor tag $a$ and tag $j$, the initial relative layout sample is:

$$
T_{a \leftarrow j}^{(i)}
=
\left(T_{c \leftarrow a}^{(i)}\right)^{-1}
T_{c \leftarrow j}^{(i)}.
$$

Write this homogeneous transform as:

$$
T_{a \leftarrow j}^{(i)}
=
\begin{bmatrix}
R_{a \leftarrow j}^{(i)} & \mathbf{t}_{a \leftarrow j}^{(i)} \\
\mathbf{0}^T & 1
\end{bmatrix}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & t_x \\
r_{21} & r_{22} & r_{23} & t_y \\
r_{31} & r_{32} & r_{33} & t_z \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

The planar layout keeps only $x/y$ translation and yaw:

$$
x_j^{(i)} = t_x = T_{a \leftarrow j}^{(i)}[0,3],
\qquad
y_j^{(i)} = t_y = T_{a \leftarrow j}^{(i)}[1,3].
$$

For perfectly coplanar tags $t_z=0$. Single-tag PnP can introduce small numerical out-of-plane noise, so $t_z$ is discarded and the later joint optimization keeps all tags on the shared plane.

Yaw is read from the first column of the rotation matrix, which is tag $j$'s local $x$ axis expressed in the anchor frame:

$$
\mathbf{v}_x
=
R_{a \leftarrow j}^{(i)}
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
=
\begin{bmatrix}
r_{11} \\
r_{21} \\
r_{31}
\end{bmatrix}.
$$

The implementation projects this direction into the anchor $xy$ plane and computes:

$$
\theta_j^{(i)}
=
\operatorname{atan2}(r_{21}, r_{11})
=
\operatorname{atan2}
\left(
T_{a \leftarrow j}^{(i)}[1,0],
T_{a \leftarrow j}^{(i)}[0,0]
\right).
$$

This is the same formula used by `_yaw_from_transform()`:

```python
atan2(transform[1, 0], transform[0, 0])
```

If tag $j$ is not directly co-observed with the anchor, the script propagates through an already initialized source tag $s$. If image $i$ observes known tag $s$ and unknown tag $j$, it first computes:

$$
T_{s \leftarrow j}^{(i)}
=
\left(T_{c \leftarrow s}^{(i)}\right)^{-1}
T_{c \leftarrow j}^{(i)}.
$$

Since $T_{a \leftarrow s}$ is already known:

$$
T_{a \leftarrow j}^{(i)}
=
T_{a \leftarrow s}
T_{s \leftarrow j}^{(i)}.
$$

The same extraction rule then gives $x_j^{(i)},y_j^{(i)},\theta_j^{(i)}$.

Finally, the script combines multiple samples using median $x/y$ and circular mean yaw, then initializes each image camera pose with PnP over all currently estimated tag corners.

### Optimizer Variables

Known-intrinsics mode optimizes:

$$
\mathbf{x}
=
\left[
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right].
$$

No-intrinsics mode optimizes:

$$
\mathbf{x}
=
\left[
f_x,f_y,c_x,c_y,
\mathbf{r}_1,\mathbf{t}_1,
\dots,
\mathbf{r}_N,\mathbf{t}_N,
\boldsymbol{\ell}_j\ \text{for}\ j\neq a
\right].
$$

The optimized output is written as `BoardLayout` JSON and used by `localize_camera.py` for later pose estimation.
