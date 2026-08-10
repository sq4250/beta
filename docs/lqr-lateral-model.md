# LQR 横向控制器 — 系统建模

> 3-state + $w_o$ 一阶惯性模型，控制量 $\Delta\delta$，r−y 误差约定

---

## 1. 坐标系与误差约定

**FLU 坐标系**：X+ 前，Y+ 左，Z+ 上。

**状态定义**（全部 r−y = VST − Car，投影到 VST 体轴系）：

$$
\begin{aligned}
e_y &= \text{RotZ}(-\theta_{vst}) \cdot (\mathbf{p}_{vst} - \mathbf{p}_{car}) \quad\text{的 Y 分量} \\[4pt]
e_\theta &= \theta_{vst} - \theta_{car} \\[4pt]
\dot{e}_\theta &= \dot{\theta}_{vst} - \dot{\theta}_{car} = \omega_{vst} - \omega_{car}
\end{aligned}
$$

**误差物理含义**：

| 量 | > 0 的含义 | 正确响应 |
|---|---|---|
| $e_y$ | VST 在车左侧（车偏右） | 左转，$\Delta\delta > 0$ |
| $e_\theta$ | VST 航向 > 车航向（车头偏右） | 左转，$\Delta\delta > 0$ |
| $\dot{e}_\theta$ | VST 横摆 > 车横摆 | 增大转角，$\Delta\delta > 0$ |

---

## 2. 控制量

$$
u = \Delta\delta = \delta_{car} - \delta_{vst}
$$

前轮转角相对 VST 前馈转角的增量。实际指令：

$$
\delta_{cmd} = \delta_{vst} + \Delta\delta_{fb}
$$

**相比旧模型**：旧模型控制量是 $\omega$（转向速率），需要在模型外加 `delta += omega * dt` 隐式积分。新模型直接控制 $\Delta\delta$（位置），消除隐式积分器。

---

## 3. 被控对象动态

### 3.1 运动学（小角度近似）

$$
\begin{aligned}
\dot{e}_y &= v \cdot \sin(e_\theta) \approx v \cdot e_\theta \\[4pt]
\dot{e}_\theta &= \dot{e}_\theta \quad\text{(定义)}
\end{aligned}
$$

### 3.2 横摆动态 — 一阶惯性假设

假设车体横摆 + 舵机执行器的联合动态为一阶滞后，带宽 $w_o$：

$$
\dot{\omega}_{car} = -w_o \cdot \omega_{car} + w_o \cdot \frac{v}{L} \cdot \delta_{car}
$$

稳态验证：$\dot{\omega}_{car}=0 \;\Rightarrow\; \omega_{car} = \frac{v}{L}\delta_{car}$（单车模型） ✓

### 3.3 误差动态推导

设 $\delta_{car} = \delta_{vst} + \Delta\delta$，$\dot{\omega}_{vst} \approx 0$（前馈缓变），单车模型 $\omega_{vst} = \frac{v}{L}\delta_{vst}$：

$$
\begin{aligned}
\ddot{e}_\theta &= \dot{\omega}_{vst} - \dot{\omega}_{car} \\
&= -\dot{\omega}_{car} \\
&= w_o\omega_{car} - w_o\frac{v}{L}(\delta_{vst} + \Delta\delta) \\
&= w_o(\omega_{vst} - \dot{e}_\theta) - w_o\frac{v}{L}\delta_{vst} - w_o\frac{v}{L}\Delta\delta \\
&= w_o\cancel{(\omega_{vst} - \tfrac{v}{L}\delta_{vst})} - w_o\dot{e}_\theta - w_o\frac{v}{L}\Delta\delta \\[4pt]
&= -w_o \cdot \dot{e}_\theta - w_o \cdot \frac{v}{L} \cdot \Delta\delta
\end{aligned}
$$

---

## 4. 状态空间模型

**状态向量**：$\mathbf{x} = [e_y,\; e_\theta,\; \dot{e}_\theta]^T$

**控制输入**：$u = \Delta\delta$

$$
\underbrace{\frac{d}{dt}\begin{bmatrix} e_y \\ e_\theta \\ \dot{e}_\theta \end{bmatrix}}_{\dot{\mathbf{x}}} =
\underbrace{\begin{bmatrix} 0 & v & 0 \\ 0 & 0 & 1 \\ 0 & 0 & -w_o \end{bmatrix}}_{A}
\underbrace{\begin{bmatrix} e_y \\ e_\theta \\ \dot{e}_\theta \end{bmatrix}}_{\mathbf{x}} +
\underbrace{\begin{bmatrix} 0 \\ 0 \\ -w_o v/L \end{bmatrix}}_{B}
\Delta\delta
$$

**模型特点**：

- $B_3 = -w_o v/L < 0$（负号来自"车横摆加速需要时间"的物理）
- $A_{3,3} = -w_o$ 提供自然阻尼，替代旧模型的三重积分链
- $w_o$ 越高 → 舵机响应越快 → 越接近旧模型（$w_o \to \infty$ 时 $e^{-w_o t} \to$ 阶跃）

---

## 5. 与经典 Frenet 误差动力学的对比

### 5.1 经典形式

Frenet 标架沿参考路径移动，误差动力学显式包含路径曲率 $\kappa_{ref}$ 及其变化率：

$$
\begin{aligned}
\dot{e}_y &= v \cdot \sin(e_\theta) \approx v \cdot e_\theta \\[4pt]
\dot{e}_\theta &= \omega - v \cdot \kappa_{ref} \\[4pt]
\ddot{e}_\theta &= \dot{\omega} - \dot{\omega}_{ref}
= -w_o\omega + w_o\frac{v}{L}\delta - \underbrace{(v\dot{\kappa}_{ref} + \dot{v}\kappa_{ref})}_{\dot{\omega}_{ref}}
\end{aligned}
$$

代入 $\omega = \dot{e}_\theta + v\kappa_{ref}$，$\delta = \delta_{vst} + \Delta\delta$：

$$
\ddot{e}_\theta = -w_o\dot{e}_\theta + w_o\frac{v}{L}\Delta\delta
- \dot{\omega}_{ref}
- w_o v\Bigl(\kappa_{ref} - \frac{\delta_{vst}}{L}\Bigr)
$$

矩阵形式：

$$
\boxed{
\dot{\mathbf{x}} =
\underbrace{\begin{bmatrix} 0 & v & 0 \\ 0 & 0 & 1 \\ 0 & 0 & -w_o \end{bmatrix}}_{A}
\mathbf{x} +
\underbrace{\begin{bmatrix} 0 \\ 0 \\ -w_o v/L \end{bmatrix}}_{B}
u \;+\;
\underbrace{\begin{bmatrix} 0 \\ 0 \\ -\dot{v} \end{bmatrix}}_{B_\kappa}
\kappa_{ref} \;+\;
\underbrace{\begin{bmatrix} 0 \\ 0 \\ -v \end{bmatrix}}_{B_{\dot{\kappa}}}
\dot{\kappa}_{ref}
}
$$

### 5.2 我们的简化

| 项 | 物理含义 | 我们的处理 |
|---|---|---|
| $\kappa_{ref} - \delta_{vst}/L$ | VST 转角是否满足单车模型 | ≡ 0（VST 是单车模型推进） |
| $\dot{v}\kappa_{ref}$ | 加减速时路径曲率的影响 | ≈ 0（匀速段主导） |
| $v\dot{\kappa}_{ref}$ | 入弯/出弯时的曲率变化率 | ≈ 0（NN 转角平滑连续） |

三项归零后，模型退化为：

$$
\dot{\mathbf{x}} = A\mathbf{x} + B u
$$

没有显式扰动通道——这不是因为扰动不存在，而是因为：

1. **$B_\kappa, B_{\dot{\kappa}}$ 项通过前馈覆盖**：$\delta_{cmd} = \delta_{vst} + \Delta\delta_{fb}$，路径曲率信息通过 $\delta_{vst}$ 前馈注入，不走反馈通道
2. **残差由 LQR 增益裕度吸收**：若入弯瞬时 $\dot{\kappa}_{ref}$ 不可忽略，表现为 $\dot{e}_\theta$ 的短暂扰动，$K_{\dot{e}\theta}$ 阻尼掉

### 5.3 VST Frenet vs 经典 Frenet

| | 经典 Frenet | VST Frenet（我们） |
|---|---|---|
| 参考标架原点 | 路径上几何最近点 | 同步运动的虚拟车 |
| 投影方式 | $\arg\min_s \|\mathbf{p}_{car} - \mathbf{p}_{path}(s)\|$ | RotZ($-\theta_{vst}$) · ($\mathbf{p}_{vst} - \mathbf{p}_{car}$) |
| $\kappa_{ref}$ 来源 | 路径几何二阶导数 | VST 单车模型：$\approx \delta_{vst}/L$ |
| 优点 | 适用于任意静止路径 | 无需最近点搜索，无奇点 |
| 局限 | $e_y$ 过大时多解/无解 | 要求 VST 不远离真车（>25cm 时重同步） |

本质：经典 Frenet 跟踪**路径**，VST Frenet 跟踪一个**沿路径运动的理想车**。两者等价当 $e_y$ 小且 VST 沿路径切向移动时。

---

## 6. LQR 设计

### 6.1 代价函数

$$
J = \int_0^\infty \left( q_{ey} e_y^2 + q_{e\theta} e_\theta^2 + q_{\dot{e}\theta} \dot{e}_\theta^2 + r \cdot \Delta\delta^2 \right) dt
$$

### 6.2 Riccati 求解

$$
\begin{aligned}
A^T P + P A - P B R^{-1} B^T P + Q &= 0 \\[4pt]
K_{riccati} &= R^{-1} B^T P \quad\in \mathbb{R}^{1 \times 3}
\end{aligned}
$$

标准 LQR 最优控制：$u^* = -K_{riccati} \cdot \mathbf{x}$

### 6.3 符号约定（代码层）

由于 $B_3 < 0$，$K_{riccati}$ 各分量有正有负。为延续 r−y 风格的 `u = +K·x` 写法：

$$
\boxed{K_{stored} = -K_{riccati}}
$$

代码直接使用：

$$
\Delta\delta_{fb} = K_{stored}[0] \cdot e_y + K_{stored}[1] \cdot e_\theta + K_{stored}[2] \cdot \dot{e}_\theta
$$

两层负号抵消，$K_{stored}$ 全正。

---

## 7. 实现

### 7.1 误差计算（`tasks.c:ref_frame_error`）

```c
dx = vst->x - car->x;        // e_world = r - y
dy = vst->y - car->y;
ex =  dx * cos(θ_vst) + dy * sin(θ_vst);   // RotZ(-θ) row 0
ey = -dx * sin(θ_vst) + dy * cos(θ_vst);   // RotZ(-θ) row 1
```

### 7.2 状态计算（`lateral.c:lateral_step`）

```c
eth   = wrap_pi(vst->theta - car->theta);               // 航向误差
eth_d = bicycle_curvature(vst->v, vst->delta) - gyro_z;  // 模型yaw rate - 陀螺实测
```

### 7.3 控制律（`lateral.c:lateral_step`）

```c
delta_fb = K_stored[0]*ey + K_stored[1]*eth + K_stored[2]*eth_d;
return delta_fb;  // Δδ_fb，调用方叠加 vst->delta
```

### 7.4 指令输出（`tasks.c:tracking_layer_step`）

```c
delta_fb = lateral_step(car, vst, gyro_z, ey);
cmd->servo_delta = clamp(vst->delta + delta_fb, -SERVO_DELTA_MAX, SERVO_DELTA_MAX);
```

---

## 8. 参数说明

| 参数 | 当前值 | 含义 |
|---|---|---|
| $q_{ey}$ | 0 | 横向误差权重 |
| $q_{e\theta}$ | 5 | 航向误差权重 |
| $q_{\dot{e}\theta}$ | 0.5 | 横摆角速度误差权重 |
| $r$ | 1 | 控制增量代价 |
| $w_o$ | 15 rad/s | 横摆一阶惯性带宽 (~2.4 Hz) |
| $L$ | 0.15 m | 轴距（物理值） |

### 8.1 增益表（$v = 0.1 \sim 5.0$ m/s）

| $v$ [m/s] | $K_{ey}$ | $K_{e\theta}$ | $K_{\dot{e}\theta}$ |
|---|---|---|---|
| 0.1 | ~0 | 2.236 | 0.288 |
| 1.1 | ~0 | 2.236 | 0.610 |
| 2.1 | ~0 | 2.236 | 0.653 |
| 3.0 | ~0 | 2.236 | 0.670 |
| 5.0 | ~0 | 2.236 | 0.684 |

注意 $K_{ey} \approx 0$ 是因为 $q_{ey} = 0$——当前为纯航向控制，$e_y$ 的镇定通过"航向对齐 → 轨迹收敛"的级联方式间接完成。

---

## 9. 调参命令

```bash
# 查看增益和极点
python tools/tune_lqr.py [Q_ey] [Q_eth] [Q_ethd] [R] [wo] [L]

# 生成增益表
python tools/gen_lqr.py --q-ey 0 --q-eth 5 --q-ethd 0.5 -r 1 --wo 15 \
    -o project/code/core/lqr_gains.h

# CSV 可视化
python tools/plot_run.py [csv_file]
```
