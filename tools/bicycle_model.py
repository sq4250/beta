"""
自行车模型（Kinematic Bicycle Model）仿真
===========================================
控制层：加速度层（纵向加速度 + 前轮转角角速度）
状态: [x, y, θ, v, δ]ᵀ
控制: [a_long, ω_δ]ᵀ

约束:
  - 速度:           0 ≤ v ≤ v_max
  - 前轮转角:       |δ| ≤ δ_max
  - 纵向加速度:     |a_long| ≤ a_long,max
  - 角速度:         |ω_δ| ≤ ω_δ,max
  - 加速度椭圆:     (a_long/a_long,max)² + (a_lat/a_lat,max)² ≤ 1
                    其中 a_lat = v²·tan(δ) / L
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Callable
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


# ============================================================
#  参数配置
# ============================================================
@dataclass
class BicycleParams:
    """自行车模型物理参数与约束"""
    # 几何
    wheelbase: float = 0.15          # L: 轴距 [m]

    # 转向
    min_turn_radius: float = 0.30    # R_min: 最小转弯半径 [m]

    # 速度约束
    v_max: float = 3.5               # 最大速度 [m/s]

    # 加速度边界 (半轴)
    a_long_max: float = 4.0          # 最大纵向加速度 [m/s²]
    a_lat_max: float = 6.0           # 最大横向加速度 [m/s²]

    # 转向角速度约束
    omega_delta_max: float = 14.0    # 最大前轮转角角速度 [rad/s]

    # 仿真
    dt: float = 0.001                # 仿真步长 [s]

    @property
    def delta_max(self) -> float:
        """最大前轮转角 [rad]"""
        return np.arctan(self.wheelbase / self.min_turn_radius)


# ============================================================
#  状态与控制
# ============================================================
@dataclass
class BicycleState:
    """自行车模型状态向量"""
    x: float = 0.0       # x 坐标 [m]
    y: float = 0.0       # y 坐标 [m]
    theta: float = 0.0   # 航向角 [rad]
    v: float = 0.0       # 纵向速度 [m/s]
    delta: float = 0.0   # 前轮转角 [rad]

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta, self.v, self.delta])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "BicycleState":
        return cls(arr[0], arr[1], arr[2], arr[3], arr[4])


# ============================================================
#  控制约束裁剪
# ============================================================
class ControlClipper:
    """将原始控制量裁剪到可行域内"""

    def __init__(self, params: BicycleParams):
        self.p = params

    def clip(
        self,
        u: np.ndarray,       # [a_long_raw, omega_delta_raw]
        v: float,             # 当前速度
        delta: float,         # 当前前轮转角
    ) -> np.ndarray:
        """
        裁剪控制量 u = [a_long, ω_δ]，使下一步状态满足所有约束。

        裁剪顺序:
          1. ω_δ → 裁剪到 [-ω_δ_max, +ω_δ_max]
          2. δ_new = δ + ω_δ * dt → 裁剪到 [-δ_max, +δ_max]，反算 ω_δ
          3. 预判 v_new (名义值) → 计算 a_lat → 椭圆约束 → 裁剪 a_long
          4. a_long → 裁剪到 [-a_long_max, +a_long_max]
          5. 确保 v_new ≥ 0
        """
        a_long_raw, omega_raw = u[0], u[1]
        dt = self.p.dt

        # ---- Step 1: 裁剪 ω_δ ----
        omega = np.clip(omega_raw, -self.p.omega_delta_max, self.p.omega_delta_max)

        # ---- Step 2: 裁剪 δ (含速度相关限制, 保证 a_lat ≤ a_lat_max) ----
        delta_new_nominal = delta + omega * dt
        # 固定上限
        delta_new = np.clip(delta_new_nominal, -self.p.delta_max, self.p.delta_max)
        # 速度相关上限: |δ| ≤ arctan(a_lat_max * L / v²)
        v_for_delta = max(v, 0.01)  # 避免除零
        delta_speed_limit = np.arctan(self.p.a_lat_max * self.p.wheelbase / v_for_delta**2)
        delta_new = np.clip(delta_new, -delta_speed_limit, delta_speed_limit)
        # 反算实际角速度
        omega = (delta_new - delta) / dt

        # ---- Step 3: 椭圆约束 ----
        # 先取名义纵向加速度，计算名义新速度
        a_long = np.clip(a_long_raw, -self.p.a_long_max, self.p.a_long_max)
        v_new_nominal = v + a_long * dt
        v_new_nominal = max(v_new_nominal, 0.0)

        # 计算名义横向加速度
        a_lat = v_new_nominal**2 * np.tan(delta_new) / self.p.wheelbase
        a_lat_abs = abs(a_lat)

        # 若横向加速度本身超出上限 → 需要减速
        if a_lat_abs > self.p.a_lat_max:
            # 反算使 a_lat = a_lat_max 的最大允许速度
            v_max_allowed = np.sqrt(
                self.p.a_lat_max * self.p.wheelbase / abs(np.tan(delta_new))
            )
            v_max_allowed = min(v_max_allowed, self.p.v_max)
            # 需要减速：取制动加速度
            a_long_brake = (v_max_allowed - v) / dt
            a_long_brake = np.clip(a_long_brake, -self.p.a_long_max, 0.0)
            # 此时 a_lat = a_lat_max, 椭圆约束迫使 a_long = 0
            # 但制动是瞬态的，本步允许 a_long 为负以满足减速需求
            return np.array([a_long_brake, omega])

        # 椭圆约束: (a_long / a_long_max)² + (a_lat / a_lat_max)² ≤ 1
        # → |a_long| ≤ a_long_max * sqrt(1 - (a_lat/a_lat_max)²)
        lat_ratio = a_lat_abs / self.p.a_lat_max
        max_a_long_from_ellipse = self.p.a_long_max * np.sqrt(max(0.0, 1.0 - lat_ratio**2))

        a_long = np.clip(a_long, -max_a_long_from_ellipse, max_a_long_from_ellipse)

        # ---- Step 4: 速度上下界 ----
        v_new = v + a_long * dt
        if v_new > self.p.v_max:
            a_long = (self.p.v_max - v) / dt
        elif v_new < 0.0:
            a_long = (0.0 - v) / dt

        # 最终 a_long 也必须在边界内
        a_long = np.clip(a_long, -self.p.a_long_max, self.p.a_long_max)

        return np.array([a_long, omega])


# ============================================================
#  动力学
# ============================================================
class BicycleModel:
    """运动学自行车模型仿真器"""

    def __init__(
        self,
        params: Optional[BicycleParams] = None,
        initial_state: Optional[BicycleState] = None,
    ):
        self.p = params or BicycleParams()
        self.state = initial_state or BicycleState()
        self.clipper = ControlClipper(self.p)

        # 仿真历史记录
        self.history: list[BicycleState] = [BicycleState(
            x=self.state.x, y=self.state.y, theta=self.state.theta,
            v=self.state.v, delta=self.state.delta,
        )]

    def reset(self, state: Optional[BicycleState] = None):
        """重置仿真状态"""
        self.state = state or BicycleState()
        self.history = [BicycleState(
            x=self.state.x, y=self.state.y, theta=self.state.theta,
            v=self.state.v, delta=self.state.delta,
        )]

    def step(self, u: np.ndarray) -> BicycleState:
        """
        执行一步仿真 (Euler 前向积分)

        参数:
            u: 控制向量 [a_long, ω_δ]
        返回:
            更新后的状态
        """
        dt = self.p.dt
        L = self.p.wheelbase

        # 裁剪控制量
        u_clipped = self.clipper.clip(u, self.state.v, self.state.delta)
        a_long, omega = u_clipped[0], u_clipped[1]

        # ---- Euler 积分 ----
        x_new     = self.state.x     + self.state.v * np.cos(self.state.theta) * dt
        y_new     = self.state.y     + self.state.v * np.sin(self.state.theta) * dt
        theta_new = self.state.theta + self.state.v * np.tan(self.state.delta) / L * dt
        v_new     = self.state.v     + a_long * dt
        delta_new = self.state.delta + omega * dt

        # 数值安全裁剪
        v_new = max(0.0, min(v_new, self.p.v_max))
        delta_new = np.clip(delta_new, -self.p.delta_max, self.p.delta_max)
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))  # 标准化到 [-π, π]

        self.state = BicycleState(x_new, y_new, theta_new, v_new, delta_new)
        self.history.append(self.state)
        return self.state

    def simulate(
        self,
        controller: Callable[[BicycleState, float], np.ndarray],
        duration: float,
    ) -> list[BicycleState]:
        """
        运行仿真

        参数:
            controller: 控制律函数 (state, t) -> u = [a_long, ω_δ]
            duration:   仿真时长 [s]
        返回:
            状态历史列表
        """
        n_steps = int(duration / self.p.dt)
        for k in range(n_steps):
            t = k * self.p.dt
            u = controller(self.state, t)
            self.step(u)
        return self.history

    @property
    def history_array(self) -> np.ndarray:
        """状态历史 -> (N, 5) numpy 数组"""
        return np.array([s.to_array() for s in self.history])

    @property
    def time(self) -> np.ndarray:
        return np.arange(len(self.history)) * self.p.dt

    # ---- 便捷属性: 派生量 ----
    def a_lat_history(self) -> np.ndarray:
        """横向加速度时间序列"""
        L = self.p.wheelbase
        return np.array([
            s.v**2 * np.tan(s.delta) / L for s in self.history
        ])


# ============================================================
#  可视化
# ============================================================
def plot_simulation(model: BicycleModel, title: str = "Bicycle Model Simulation"):
    """绘制仿真结果: 轨迹 + 状态时间序列 + 加速度椭圆"""
    arr = model.history_array
    t = model.time
    a_lat = model.a_lat_history()
    p = model.p

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle(title, fontsize=13, fontweight="bold")

    # ----- (0,0) 轨迹 XY -----
    ax = axes[0, 0]
    ax.plot(arr[:, 0], arr[:, 1], linewidth=0.8)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # ----- (0,1) 速度 -----
    ax = axes[0, 1]
    ax.plot(t, arr[:, 3], linewidth=0.8)
    ax.axhline(p.v_max, color="r", linestyle="--", alpha=0.5, label=f"v_max={p.v_max}")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("v [m/s]")
    ax.set_title("Longitudinal Speed")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ----- (0,2) 航向角 -----
    ax = axes[0, 2]
    ax.plot(t, np.rad2deg(arr[:, 2]), linewidth=0.8)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("θ [deg]")
    ax.set_title("Heading Angle")
    ax.grid(True, alpha=0.3)

    # ----- (1,0) 前轮转角 -----
    ax = axes[1, 0]
    ax.plot(t, np.rad2deg(arr[:, 4]), linewidth=0.8)
    ax.axhline(np.rad2deg(p.delta_max), color="r", linestyle="--", alpha=0.5,
               label=f"δ_max={np.rad2deg(p.delta_max):.1f}°")
    ax.axhline(-np.rad2deg(p.delta_max), color="r", linestyle="--", alpha=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("δ [deg]")
    ax.set_title("Steering Angle")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ----- (1,1) 加速度椭圆 -----
    ax = axes[1, 1]
    # 绘制椭圆边界
    ellipse = Ellipse((0, 0), 2 * p.a_long_max, 2 * p.a_lat_max,
                      edgecolor="gray", facecolor="lightblue", alpha=0.3,
                      linewidth=1.5, linestyle="--")
    ax.add_patch(ellipse)
    # 绘制历史加速度点
    # 从差分估算实际 a_long
    a_long_est = np.gradient(arr[:, 3], p.dt)
    ax.scatter(a_long_est[::50], a_lat[::50], s=1, alpha=0.5, color="blue")
    ax.set_xlabel("a_long [m/s²]")
    ax.set_ylabel("a_lat [m/s²]")
    ax.set_title("Acceleration Ellipse")
    ax.set_xlim(-p.a_long_max * 1.3, p.a_long_max * 1.3)
    ax.set_ylim(-p.a_lat_max * 1.3, p.a_lat_max * 1.3)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # ----- (1,2) 横向加速度 -----
    ax = axes[1, 2]
    ax.plot(t, a_lat, linewidth=0.8)
    ax.axhline(p.a_lat_max, color="r", linestyle="--", alpha=0.5, label=f"a_lat,max={p.a_lat_max}")
    ax.axhline(-p.a_lat_max, color="r", linestyle="--", alpha=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("a_lat [m/s²]")
    ax.set_title("Lateral Acceleration")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


# ============================================================
#  示例控制器
# ============================================================
def example_controller_double_lane_change(state: BicycleState, t: float) -> np.ndarray:
    """
    示例：模拟类双移线操作
    加速到巡航速度 → 左转 → 直行 → 右转 → 直行
    """
    # ---- 加速阶段 (0-0.5s) ----
    if t < 0.5:
        a_long = 3.0       # 加速
        omega = 0.0

    # ---- 左转 (0.5-1.0s) ----
    elif t < 1.0:
        a_long = 0.0       # 匀速
        omega = 10.0        # 快速左转

    # ---- 回正并右转 (1.0-1.8s) ----
    elif t < 1.4:
        a_long = 0.0
        omega = -12.0       # 快速右转
    elif t < 1.8:
        a_long = 0.0
        omega = 0.0        # 保持

    # ---- 回正 (1.8-2.2s) ----
    elif t < 2.2:
        a_long = -2.0       # 轻微制动
        omega = 12.0        # 回正

    # ---- 巡航 ----
    else:
        a_long = 0.0
        omega = 0.0

    return np.array([a_long, omega])


def example_controller_circle(state: BicycleState, t: float) -> np.ndarray:
    """示例：加速 + 定圆行驶"""
    if state.v < 1.5:
        a_long = 3.0
        omega = 0.0
    else:
        # 匀速 + 恒定方向盘转角（大致对应定圆）
        a_long = 0.0
        # 目标: 稳态转角 ≈ 0.2 rad
        target_delta = 0.25
        omega = (target_delta - state.delta) / 0.05  # 简单 P 控制器
        omega = np.clip(omega, -14.0, 14.0)
    return np.array([a_long, omega])


# ============================================================
#  主函数
# ============================================================
# ============================================================
#  子步到达检测: 检查线段到目标的最小距离
# ============================================================
def check_hit_substep(s_prev_xy, s_next_xy, target_xy, tol):
    """检查从 s_prev 到 s_next 的路径是否穿过目标的 TOL 圆.

    当前 20Hz 离散切换只在步端点检查 — 可能跳过 TOL 圆.
    本函数额外检查线段到目标的最小距离, 模拟子步触发.

    Args:
        s_prev_xy: (x, y) 步起点
        s_next_xy: (x, y) 步终点
        target_xy: (x, y) 目标坐标
        tol: 到达阈值
    Returns:
        bool: 是否到达
    """
    # 端点优先
    ax, ay = s_prev_xy[0], s_prev_xy[1]
    bx, by = s_next_xy[0], s_next_xy[1]
    px, py = target_xy[0], target_xy[1]

    if (bx - px)*(bx - px) + (by - py)*(by - py) < tol*tol:
        return True

    # 线段到点的最小距离
    abx, aby = bx - ax, by - ay
    ab2 = abx*abx + aby*aby
    if ab2 < 1e-12:
        return False

    t = max(0.0, min(1.0, ((px - ax)*abx + (py - ay)*aby) / ab2))
    cx, cy = ax + t*abx, ay + t*aby
    return (px - cx)*(px - cx) + (py - cy)*(py - cy) < tol*tol


if __name__ == "__main__":
    params = BicycleParams(
        wheelbase=0.15,
        min_turn_radius=0.30,
        v_max=3.5,
        a_long_max=4.0,
        a_lat_max=6.0,
        omega_delta_max=14.0,
        dt=0.001,
    )

    print("=" * 60)
    print("自行车模型参数")
    print("=" * 60)
    print(f"  轴距 L:           {params.wheelbase:.3f} m")
    print(f"  最小转弯半径:     {params.min_turn_radius:.3f} m")
    print(f"  最大前轮转角 δ_max: {params.delta_max:.3f} rad = {np.rad2deg(params.delta_max):.1f}°")
    print(f"  最大速度 v_max:   {params.v_max:.1f} m/s")
    print(f"  加速度椭圆:       a_long ≤ {params.a_long_max}, a_lat ≤ {params.a_lat_max}")
    print(f"  最大转角角速度:   {params.omega_delta_max:.0f} rad/s")
    print(f"  仿真步长 dt:      {params.dt:.3f} s")
    print()

    # ---- 仿真 1: 双移线 ----
    print("运行仿真: 双移线...")
    model1 = BicycleModel(params)
    model1.simulate(example_controller_double_lane_change, duration=3.0)
    fig1 = plot_simulation(model1, title="Double Lane Change Maneuver")
    print(f"  完成: {len(model1.history)} 步, 终止状态 v={model1.state.v:.2f}, "
          f"δ={np.rad2deg(model1.state.delta):.1f}°")

    # ---- 仿真 2: 定圆 ----
    print("运行仿真: 定圆行驶...")
    model2 = BicycleModel(params)
    model2.simulate(example_controller_circle, duration=4.0)
    fig2 = plot_simulation(model2, title="Circular Motion")
    print(f"  完成: {len(model2.history)} 步, 终止状态 v={model2.state.v:.2f}, "
          f"δ={np.rad2deg(model2.state.delta):.1f}°")

    # 保存图像
    fig1.savefig("bicycle_model_double_lane_change.png", dpi=150)
    fig2.savefig("bicycle_model_circle.png", dpi=150)
    print("\n图像已保存: bicycle_model_double_lane_change.png, bicycle_model_circle.png")
    print("仿真完成。")
