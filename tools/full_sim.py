"""
full_sim.py — β_ackerman 全闭环仿真: NN 规划器 + VST + LQR + LADRC + 小车动力学

与固件逐层对应 (test/re-record 分支, config.h 参数):
    1kHz  ISR : 状态估计 (四元数航向视为理想) + 被控对象积分
    200Hz 跟踪: VST 摩擦圆运动学 + 5cm 重同步 + LQR 横向 + LADRC 纵向
    20Hz  规划: 航点队列 + TSP + 线段命中检测 + NN 前向 (ZOH)

被控对象 = 我们建模的假设 (见 tools/longitudinal_ladrc.tex / lateral_3state.tex):
    纵向:  v̇ = −α·v + b0·u + f,   u = (thr_L+thr_R)/2,  α=0.6 b0=15 (名义)
           f 为残差扰动 (默认 0, 可用 --f0/--slope 注入坡度/摩擦)
    横向:  ω̇ = wo·((v/L)·tanδ − ω),  wo=15 (LQR 设计模型, 舵机视为理想)
           θ̇ = ω,  ẋ = v·cosθ,  ẏ = v·sinθ

NN 权重: 默认直接解析部署的 core/nn_model_a5b3l4.c (scale 已折叠进第一层,
         输入为原始物理量); 也可 --ckpt runs/gp_small_kamm533.pt 加载训练
         检查点 (未折叠, 输入需归一化 v/5, δ/DELTA_MAX, d/5, ang/π).

用法:
  python full_sim.py --mode 2                        # 6 航点自跑 (默认)
  python full_sim.py --mode 1                        # 位置阶跃 (rx 0→1.2m)
  python full_sim.py --mode 2 --f0 -0.3              # 注入恒定扰动 (坡度)
  python full_sim.py --mode 2 --ckpt C:/Devel/test/pipeline_kamm533/runs/gp_small_kamm533.pt
  python full_sim.py --mode 2 --plot                 # 出 CSV 后直接画概览图

输出 CSV 与车端格式完全一致 (MODE2 12 列 / MODE1 5 列),
可直接喂 tools/plot_run.py 和 tools/plot_traj_stats.py。
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np

PROJ = Path(__file__).resolve().parents[1]
CODE = PROJ / 'project' / 'code'

# ═══════════════════ 配置 (镜像 config.h) ═══════════════════
WHEELBASE      = 0.15
INV_WHEELBASE  = 1.0 / WHEELBASE
TRACK_WIDTH    = 0.15
STARTUP_DELAY_MS = 2000
SERVO_DELTA_MAX  = 0.576
DELTA_MAX        = 0.46364761
OMEGA_DELTA_MAX  = 14.0
TOL_XY           = 0.10

ISR_FREQ, ISR_DT     = 1000, 0.001
TRACKER_FREQ, CTRL_DT = 200, 0.005
TRACKER_DIV = ISR_FREQ // TRACKER_FREQ
PLANNER_MS  = 50

LQR_V_MIN = 0.1
LQR_SPEED_BP = np.array([0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0])
LQR_GAINS = np.array([
    [4.472136, 1.187723, 0.077195], [4.472136, 1.297979, 0.076722],
    [4.472136, 1.420622, 0.075638], [4.472136, 1.529344, 0.074331],
    [4.472136, 1.626936, 0.072967], [4.472136, 1.715568, 0.071623],
    [4.472136, 1.796881, 0.070330], [4.472136, 1.872121, 0.069101],
    [4.472136, 1.942245, 0.067939], [4.472136, 2.008002, 0.066844],
    [4.472136, 2.069987, 0.065812]])

LONG_B0, LONG_ALPHA, LONG_WO = 15.0, 0.6, 8.0
LONG_KP, LONG_KD, LONG_FHAT_MAX = 4.0, 4.0, 1.5
SPEED_FLT_ALPHA = 0.03

STEP_TEST_DX = 1.2
# 车端 CSV 周期参考: MODE1=20ms(50Hz), MODE2=100ms(10Hz, UART 115200 限制)

# 模型约束 (g_model_kamm, 补录分支)
MODEL = dict(a_long=5.0, a_brake=3.0, a_lat=4.0, v_max=5.0,
             nn_a_max=5.0, nn_a_brake=3.0, nn_o_max=14.0)
A_BRAKE_MAX = 3.0

# 当年 6 点坐标 (cm → m)
BEACON_WORLD_CM = np.array([
    [171.5, 81.5], [344.5, 143.0], [456.5, 9.5],
    [370.0, -148.0], [191.0, -109.0], [296.5, -7.5]])
BEACONS = BEACON_WORLD_CM * 0.01
CAR_START = np.array([0.0, 0.0])


# ═══════════════════ NN (解析部署 C 文件) ═══════════════════
_C_W_RE = re.compile(r'static const f32 (\w+)\[\d+\] = \{\s*([^}]*)\};')

def load_nn_from_c(c_path=None):
    """解析 nn_model_a5b3l4.c → 权重 dict (scale 已折叠, 输入用原始物理量)."""
    c_path = c_path or CODE / 'core' / 'nn_model_a5b3l4.c'
    src = Path(c_path).read_text(encoding='utf-8')
    W = {}
    for name, body in _C_W_RE.findall(src):
        vals = [float(x.rstrip('f')) for x in body.split(',') if x.strip()]
        W[name] = np.array(vals, dtype=np.float64)
    need = ['fc_state_w', 'fc_state_b', 'fc_tgt_w', 'fc_tgt_b',
            'fc1_w', 'fc1_b', 'fc2_w', 'fc2_b', 'fc3_w', 'fc3_b', 'fc4_w', 'fc4_b']
    missing = [k for k in need if k not in W]
    if missing:
        raise SystemExit(f'C 权重解析失败, 缺 {missing}: {c_path}')
    # C 平铺数组按 w[i*in_dim+j] 行主序存放 → reshape 回矩阵
    W['fc_state_w'] = W['fc_state_w'].reshape(8, 2)
    W['fc_tgt_w']   = W['fc_tgt_w'].reshape(8, 2)
    W['fc1_w'] = W['fc1_w'].reshape(48, 32)
    W['fc2_w'] = W['fc2_w'].reshape(32, 48)
    W['fc3_w'] = W['fc3_w'].reshape(16, 32)
    W['fc4_w'] = W['fc4_w'].reshape(2, 16)
    return W


def load_nn_from_pt(pt_path):
    """加载训练检查点 (未折叠权重, 输入需归一化)."""
    try:
        import torch
    except ImportError:
        raise SystemExit('--ckpt 需要 torch; 默认走 C 文件解析则不需要')
    sd = torch.load(pt_path, map_location='cpu', weights_only=False)['model_state_dict']
    W = {k: v.detach().cpu().numpy().astype(np.float64) for k, v in sd.items()}
    return {'state_enc': (W['state_enc.weight'], W['state_enc.bias']),
            'target_enc': (W['target_enc.weight'], W['target_enc.bias']),
            'fc1': (W['fc1.weight'], W['fc1.bias']),
            'fc2': (W['fc2.weight'], W['fc2.bias']),
            'fc3': (W['fc3.weight'], W['fc3.bias']),
            'fc4': (W['fc4.weight'], W['fc4.bias'])}


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def polar_encode_c(vst, g1, g2, g3, v2, v3):
    """与 nn_model_a5b3l4.c polar_encode 一致: 原始物理量 (scale 已折叠进权重)."""
    out = np.zeros(10)
    dx1, dy1 = g1[0] - vst[0], g1[1] - vst[1]
    out[0] = vst[3]                 # v
    out[1] = vst[4]                 # delta
    out[2] = np.hypot(dx1, dy1)     # d1
    out[3] = wrap_pi(np.arctan2(dy1, dx1) - vst[2])
    if v2 > 0:
        dx12, dy12 = g2[0] - g1[0], g2[1] - g1[1]
        out[4] = np.hypot(dx12, dy12)
        out[5] = wrap_pi(np.arctan2(dy12, dx12) - np.arctan2(dy1, dx1))
    if v3 > 0:
        dx23, dy23 = g3[0] - g2[0], g3[1] - g2[1]
        out[6] = np.hypot(dx23, dy23)
        out[7] = wrap_pi(np.arctan2(dy23, dx23) - np.arctan2(g2[1] - g1[1], g2[0] - g1[0]))
    out[8], out[9] = v2, v3
    return out


def polar_encode_pt(vst, g1, g2, g3, v2, v3):
    """与训练 pipeline polar_8d 一致: 归一化输入 (配合未折叠检查点)."""
    out = polar_encode_c(vst, g1, g2, g3, v2, v3)
    out[0] /= 5.0
    out[1] /= DELTA_MAX
    out[2] /= 5.0
    out[3] /= np.pi
    out[4] /= 5.0
    out[5] /= np.pi
    out[6] /= 5.0
    out[7] /= np.pi
    return out


class NNPlanner:
    """GP-Small 3.7K: polar_8d → 32 → 48 → 32 → 16 → 2 (ReLU)."""

    def __init__(self, weights, normalize=False):
        self.normalize = normalize
        if normalize:  # torch 检查点键名
            (w, b), W = weights['state_enc'], {}
            W['fc_state_w'], W['fc_state_b'] = w, b
            (w, b) = weights['target_enc']
            W['fc_tgt_w'], W['fc_tgt_b'] = w, b
            for i in range(1, 5):
                (w, b) = weights[f'fc{i}']
                W[f'fc{i}_w'], W[f'fc{i}_b'] = w, b
        else:
            W = weights
        self.W = W

    @staticmethod
    def _dense(x, w, b, relu=False):
        y = b + w @ x
        return np.maximum(y, 0.0) if relu else y

    def __call__(self, vst, g1, g2, g3, v2, v3):
        inp = polar_encode_pt(vst, g1, g2, g3, v2, v3) if self.normalize \
              else polar_encode_c(vst, g1, g2, g3, v2, v3)
        W = self.W
        h_state = self._dense(inp[:2], W['fc_state_w'], W['fc_state_b'])
        h_t1 = self._dense(inp[2:4], W['fc_tgt_w'], W['fc_tgt_b'])
        h_t2 = self._dense(inp[4:6], W['fc_tgt_w'], W['fc_tgt_b']) * v2
        h_t3 = self._dense(inp[6:8], W['fc_tgt_w'], W['fc_tgt_b']) * v3
        cat = np.concatenate([h_state, h_t1, h_t2, h_t3])
        h = self._dense(cat, W['fc1_w'], W['fc1_b'], relu=True)
        h = self._dense(h, W['fc2_w'], W['fc2_b'], relu=True)
        h = self._dense(h, W['fc3_w'], W['fc3_b'], relu=True)
        out = self._dense(h, W['fc4_w'], W['fc4_b'])
        a = np.clip(out[0], -MODEL['nn_a_brake'], MODEL['nn_a_max'])
        om = np.clip(out[1], -MODEL['nn_o_max'], MODEL['nn_o_max'])
        return a, om


# ═══════════════════ 小车被控对象 (我们的建模假设) ═══════════════════
class CarPlant:
    """纵向一阶阻力模型 + 横向 wo 一阶惯性 + 自行车运动学.

    state: [x, y, theta, v, omega]
    u: 左右油门平均; delta: 实际前轮角 (可选一阶舵机滞后, tau_s=0 即理想).
    """

    def __init__(self, alpha=LONG_ALPHA, b0=LONG_B0, wo=15.0,
                 f0=0.0, c2=0.0, tau_s=0.0):
        self.alpha, self.b0, self.wo = alpha, b0, wo
        self.f0, self.c2 = f0, c2
        self.tau_s = tau_s
        self.delta_act = 0.0

    def step(self, s, thr_avg, delta_cmd, dt=ISR_DT):
        v = s[3]
        f = self.f0 - self.c2 * v * abs(v)          # 残差扰动 (坡度/二次阻力)
        dv = (-self.alpha * v + self.b0 * thr_avg + f) * dt
        vn = max(0.0, v + dv)
        s[3] = vn
        # 舵机一阶滞后: 真值 δ_act 追赶指令, 估计器仍用 cmd (与车端一致)
        if self.tau_s > 0.0:
            self.delta_act += (delta_cmd - self.delta_act) * dt / self.tau_s
        else:
            self.delta_act = delta_cmd
        s[4] += self.wo * ((vn / WHEELBASE) * np.tan(self.delta_act) - s[4]) * dt
        s[2] = wrap_pi(s[2] + s[4] * dt)
        s[0] += vn * np.cos(s[2]) * dt
        s[1] += vn * np.sin(s[2]) * dt


class TireBicyclePlant:
    """动力学自行车模型 + 线性轮胎 (侧偏角 → 侧偏力, 带 μFz 饱和).

    纵向仍为一阶阻力模型 (与 CarPlant 一致); 横向:
      前轮侧偏角  αf = δ − atan((vy + lf·ω)/vx)
      后轮侧偏角  αr =     − atan((vy − lr·ω)/vx)
      侧偏力      Fy = clip(−C·α, ±μFz)
      m·v̇y = Fyf·cosδ + Fyr − m·vx·ω
      Iz·ω̇ = lf·Fyf·cosδ − lr·Fyr
      θ̇ = ω,  ẋ = vx·cosθ − vy·sinθ,  ẏ = vx·sinθ + vy·cosθ

    稳态转向曲率 ω/vx = δ/(L + Kus·vx²), Kus = m/L·(lr/Cf − lf/Cr)
    (Cf=Cr 中性转向时 Kus=0 → 稳态与 kinematic 模型一致, 只多瞬态滞后+侧滑)

    state 接口与 CarPlant 相同: s = [x, y, theta, vx, omega], vy 内部保存.
    编码器量的是轮速 → vx (车身系), 侧滑导致估计里程计与真值分离 (与实车一致).
    """

    G = 9.81

    def __init__(self, alpha=LONG_ALPHA, b0=LONG_B0, f0=0.0, c2=0.0,
                 m=2.0, iz=0.01, cf=60.0, cr=60.0, mu=0.4,
                 lf=WHEELBASE / 2, lr=WHEELBASE / 2, tau_s=0.0):
        self.alpha, self.b0 = alpha, b0
        self.f0, self.c2 = f0, c2
        self.m, self.iz, self.cf, self.cr, self.mu = m, iz, cf, cr, mu
        self.lf, self.lr = lf, lr
        self.fzf = m * self.G * lr / (lf + lr)   # 静态轴荷 (忽略载荷转移)
        self.fzr = m * self.G * lf / (lf + lr)
        self.vy = 0.0
        self.tau_s = tau_s
        self.delta_act = 0.0

    def step(self, s, thr_avg, delta_cmd, dt=ISR_DT):
        vx = s[3]
        f = self.f0 - self.c2 * vx * abs(vx)
        vx = max(0.0, vx + (-self.alpha * vx + self.b0 * thr_avg + f) * dt)
        s[3] = vx
        om = s[4]
        if self.tau_s > 0.0:
            self.delta_act += (delta_cmd - self.delta_act) * dt / self.tau_s
        else:
            self.delta_act = delta_cmd
        delta = self.delta_act
        vs = max(vx, 0.2)                         # 低速守卫 (α 分母)
        af = delta - np.arctan2(self.vy + self.lf * om, vs)
        ar = -np.arctan2(self.vy - self.lr * om, vs)
        fyf = float(np.clip(self.cf * af, -self.mu * self.fzf, self.mu * self.fzf))
        fyr = float(np.clip(self.cr * ar, -self.mu * self.fzr, self.mu * self.fzr))
        cd = np.cos(delta)
        self.vy += ((fyf * cd + fyr) / self.m - vx * om) * dt
        s[4] += (self.lf * fyf * cd - self.lr * fyr) / self.iz * dt
        s[2] = wrap_pi(s[2] + s[4] * dt)
        s[0] += (vx * np.cos(s[2]) - self.vy * np.sin(s[2])) * dt
        s[1] += (vx * np.sin(s[2]) + self.vy * np.cos(s[2])) * dt


# ═══════════════════ 状态估计 (镜像 estimator.c) ═══════════════════
class Estimator:
    def __init__(self, yaw_noise_deg=0.0, tau_n=0.05):
        self.v_filt = 0.0
        self.yaw_offset = 0.0
        self.s = np.zeros(5)  # x, y, theta, v, delta (估计值 g_car)
        # 航向测量噪声: OU 过程 (τ_n 相关时间), 稳态 std = yaw_noise_deg
        self.n_theta = 0.0
        self.theta_n_std = np.deg2rad(yaw_noise_deg) if yaw_noise_deg > 0 else 0.0
        self.tau_n = tau_n

    def seed(self):
        self.yaw_offset = wrap_pi(self.s[2])
        self.s[2] = 0.0

    def update(self, plant_s, cmd_delta, dt=ISR_DT):
        if self.theta_n_std > 0.0:
            self.n_theta += (-self.n_theta / self.tau_n) * dt \
                            + self.theta_n_std * np.sqrt(2.0 * dt / self.tau_n) \
                              * np.random.randn()
        self.s[2] = wrap_pi(plant_s[2] - self.yaw_offset + self.n_theta)  # 四元数+噪声
        self.v_filt += SPEED_FLT_ALPHA * (plant_s[3] - self.v_filt)  # 编码器 EMA
        self.s[3] = self.v_filt
        self.s[0] += self.v_filt * np.cos(self.s[2]) * dt
        self.s[1] += self.v_filt * np.sin(self.s[2]) * dt
        self.s[4] = cmd_delta                                 # 舵机跟踪良好假设


# ═══════════════════ 纵向 LADRC (镜像 longitudinal.c) ═══════════════════
class LADRC:
    def __init__(self):
        self.z = 0.0
        self.u_prev = 0.0
        self.f_hat = 0.0

    def step(self, v_meas, v_ref, a_ref, e_x, delta):
        z_dot = -LONG_WO * (self.z + (LONG_WO - LONG_ALPHA) * v_meas
                            + LONG_B0 * self.u_prev)
        self.z += z_dot * CTRL_DT
        self.f_hat = self.z + LONG_WO * v_meas
        if self.f_hat > 0.0:
            self.f_hat = 0.0
            self.z = -LONG_WO * v_meas
        elif self.f_hat < -LONG_FHAT_MAX:
            self.f_hat = -LONG_FHAT_MAX
            self.z = -LONG_FHAT_MAX - LONG_WO * v_meas

        u0 = LONG_KP * e_x + LONG_KD * (v_ref - v_meas) + a_ref \
             + LONG_ALPHA * v_meas - self.f_hat
        thr = float(np.clip(u0 / LONG_B0, -1.0, 1.0))
        self.u_prev = thr

        # 阿克曼差速
        if abs(delta) < 1e-4:
            return thr, thr
        hw = TRACK_WIDTH * 0.5
        inv_r = np.tan(delta) * INV_WHEELBASE
        return thr * (1.0 - hw * inv_r), thr * (1.0 + hw * inv_r)


# ═══════════════════ 横向 LQR (镜像 lateral.c) ═══════════════════
def lqr_lookup(v):
    vc = min(max(v, LQR_V_MIN), 5.0)
    i = int(np.searchsorted(LQR_SPEED_BP, vc))
    if i == 0:
        return LQR_GAINS[0]
    if i >= len(LQR_SPEED_BP):
        return LQR_GAINS[-1]
    lo, hi = LQR_SPEED_BP[i - 1], LQR_SPEED_BP[i]
    frac = (vc - lo) / (hi - lo)
    return LQR_GAINS[i - 1] + frac * (LQR_GAINS[i] - LQR_GAINS[i - 1])


def lateral_step(car, vst, gyro_z, ey):
    eth = wrap_pi(vst[2] - car[2])
    eth_d = vst[3] * np.tan(vst[4]) * INV_WHEELBASE - gyro_z
    g = lqr_lookup(vst[3])
    return g[0] * ey + g[1] * eth + g[2] * eth_d


# ═══════════════════ VST 摩擦圆运动学 (镜像 kinematics.c) ═══════════════════
def mcu_kinematics_step(s, a_raw, w_raw, dt=CTRL_DT):
    an = float(np.clip(a_raw, -MODEL['a_brake'], MODEL['a_long']))
    om = float(np.clip(w_raw, -OMEGA_DELTA_MAX, OMEGA_DELTA_MAX))

    vn = s[3] + an * dt
    if vn > MODEL['v_max']:
        vn = MODEL['v_max']
        al = (s[3] > MODEL['v_max']) and -MODEL['a_brake'] or ((MODEL['v_max'] - s[3]) / dt)
    elif vn < 0.0:
        al = (0.0 - s[3]) / dt
        vn = 0.0
    else:
        al = an

    semi = MODEL['a_long'] if al >= 0.0 else MODEL['a_brake']
    r = float(np.clip(al / (semi + 1e-8), -1.0, 1.0))
    alm = MODEL['a_lat'] * np.sqrt(max(1.0 - r * r, 0.0) + 1e-12)

    vs = max(vn, 0.01)
    dl = np.arctan(alm * WHEELBASE / (vs * vs))
    dmax = min(DELTA_MAX, dl)
    dn = float(np.clip(s[4] + om * dt, -dmax, dmax))
    om = (dn - s[4]) / dt

    s[0] += s[3] * np.cos(s[2]) * dt
    s[1] += s[3] * np.sin(s[2]) * dt
    s[2] = wrap_pi(s[2] + s[3] * np.tan(dn) / WHEELBASE * dt)
    s[3] = vn
    s[4] = dn
    return al, om


# ═══════════════════ 航点规划 (镜像 planner_task.c + tsp.c) ═══════════════════
def check_hit_substep(px, py, nx, ny, tx, ty, tol):
    if (nx - tx) ** 2 + (ny - ty) ** 2 < tol * tol:
        return True
    abx, aby = nx - px, ny - py
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return False
    t = min(max(((tx - px) * abx + (ty - py) * aby) / ab2, 0.0), 1.0)
    return (tx - (px + t * abx)) ** 2 + (ty - (py + t * aby)) ** 2 < tol * tol


def tsp_solve(points, start):
    pts = [tuple(p) for p in points]
    ordered, cur = [], np.array(start, dtype=float)
    while pts:
        j = min(range(len(pts)), key=lambda i: np.hypot(pts[i][0] - cur[0], pts[i][1] - cur[1]))
        ordered.append(pts.pop(j))
        cur = np.array(ordered[-1])
    return ordered


class Planner:
    SLOT_HOME = 0xFF

    def __init__(self, nn):
        self.nn = nn
        self.queue = []
        self.wp_active = False
        self.plan = (-A_BRAKE_MAX, 0.0)
        self.car_prev = CAR_START.copy()

    def init(self):
        route = tsp_solve(BEACONS, CAR_START)
        self.queue = [(i, p[0], p[1]) for i, p in enumerate(route)]
        self.queue.append((self.SLOT_HOME, CAR_START[0], CAR_START[1]))
        self.wp_active = True

    def step(self, car_xy, vst, g_ms):
        if not self.wp_active:
            self.plan = (-A_BRAKE_MAX, 0.0)
            return
        g = self.queue[:3]
        if not g:
            self.plan = (-A_BRAKE_MAX, 0.0)
            return
        if check_hit_substep(*self.car_prev, *car_xy, g[0][1], g[0][2], TOL_XY):
            self.queue = [w for w in self.queue if w[0] != g[0][0]]
            g = self.queue[:3]
            if not g:
                self.plan = (-A_BRAKE_MAX, 0.0)
                return
        while len(g) < 3:
            g.append(g[-1])
        v2 = 1.0 if len(self.queue) > 1 else 0.0
        v3 = 1.0 if len(self.queue) > 2 else 0.0
        vst_s = (vst[0], vst[1], vst[2], vst[3], vst[4])
        a, om = self.nn(vst_s, g[0][1:], g[1][1:], g[2][1:], v2, v3)
        self.plan = (float(a), float(om))
        self.car_prev = np.array(car_xy)


# ═══════════════════ 主仿真 ═══════════════════
def simulate(mode=2, t_max=None, nn=None, plant=None, plot=False, out=None,
             csv_ms=None, yaw_noise=0.0, servo_tau=0.0):
    assert mode in (1, 2)
    if nn is None:
        nn = NNPlanner(load_nn_from_c())
    if plant is None:
        plant = CarPlant(tau_s=servo_tau)

    vst = np.zeros(5)          # x, y, theta, v, delta
    plant_s = np.zeros(5)      # x, y, theta, v, omega
    est = Estimator(yaw_noise_deg=yaw_noise)
    ladrc = LADRC()
    planner = Planner(nn)
    if mode == 2:
        planner.init()
    step_rx = 0.0

    cmd = dict(servo_delta=0.0, motor_l=0.0, motor_r=0.0)
    g_ms = 0
    vst_seeded = False
    # 仿真无 UART 带宽限制: 默认 20ms(50Hz), 可 --csv-ms 5 (200Hz, 全保真)
    period = csv_ms if csv_ms is not None else 20

    if t_max is None:
        t_max = 5.0 if mode == 1 else 30.0
    n_isr = int(t_max * ISR_FREQ)

    rows, div = [], 0
    t_stop = None
    for k in range(n_isr):
        g_ms = k

        # 种子在估计器之前 (与 tasks.c 一致: 用上一帧车状态)
        if div == 0 and not vst_seeded:
            est.seed()
            vst[:] = est.s
            vst_seeded = True

        est.update(plant_s, cmd['servo_delta'])

        if div == 0:
            if mode == 2:
                dx, dy = vst[0] - est.s[0], vst[1] - est.s[1]
                if dx * dx + dy * dy > 0.05 * 0.05:   # 误差>5cm 重同步
                    vst[:] = est.s
            if mode == 1 and step_rx == 0.0 and g_ms >= STARTUP_DELAY_MS:
                step_rx = STEP_TEST_DX
            # ── 跟踪层 200Hz ──
            if mode == 1:
                cmd['servo_delta'] = 0.0
                thr_l, thr_r = ladrc.step(est.s[3], 0.0, 0.0,
                                          step_rx - est.s[0], 0.0)
            else:
                a_eff, w_eff = mcu_kinematics_step(vst, planner.plan[0], planner.plan[1])
                ct, st = np.cos(vst[2]), np.sin(vst[2])
                ex = (vst[0] - est.s[0]) * ct + (vst[1] - est.s[1]) * st
                ey = (est.s[0] - vst[0]) * st - (est.s[1] - vst[1]) * ct
                delta_fb = lateral_step(est.s, vst, plant_s[4], ey)
                cmd['servo_delta'] = float(np.clip(
                    vst[4] + delta_fb, -SERVO_DELTA_MAX, SERVO_DELTA_MAX))
                thr_l, thr_r = ladrc.step(est.s[3], vst[3], a_eff, ex, est.s[4])
            cmd['motor_l'], cmd['motor_r'] = thr_l, thr_r

        # 被控对象 (用本拍实际执行量)
        plant.step(plant_s, 0.5 * (cmd['motor_l'] + cmd['motor_r']),
                   cmd['servo_delta'])

        if g_ms % PLANNER_MS == 0 and mode == 2:
            if g_ms >= STARTUP_DELAY_MS:
                planner.step(est.s[:2], vst, g_ms)

        if g_ms % period == 0:
            if mode == 1:
                rows.append((g_ms * 0.001, step_rx, est.s[0], est.s[3], ladrc.f_hat))
            else:
                eth_d = vst[3] * np.tan(vst[4]) * INV_WHEELBASE - plant_s[4]
                delta_fb = cmd['servo_delta'] - vst[4]
                rows.append((g_ms * 0.001, vst[0], vst[1], est.s[0], est.s[1],
                             np.degrees(vst[2]), np.degrees(est.s[2]),
                             eth_d, delta_fb, est.s[3], vst[3], ladrc.f_hat,
                             cmd['servo_delta']))

        if mode == 2 and not planner.queue and t_stop is None:
            t_stop = g_ms + 2000
        if t_stop is not None and g_ms >= t_stop:
            break

        div = (div + 1) % TRACKER_DIV

    data = np.array(rows)
    if out is None:
        out = PROJ / 'tools' / ('sim_step.csv' if mode == 1 else 'sim_run.csv')
    out = Path(out)
    if mode == 1:
        header = ('#pos-step kp=%.1f kd=%.1f b0=%.1f alpha=%.1f wo=%.1f dx=%.1fm\n'
                  't[s],rx[m],ins_x[m],ins_v[m/s],f_hat[m/s2]'
                  % (LONG_KP, LONG_KD, LONG_B0, LONG_ALPHA, LONG_WO, STEP_TEST_DX))
        fmt = '%.3f,%.3f,%.3f,%.3f,%.3f'
    else:
        header = ('#bias=0.0000deg/s mode=2 q=%u\n'
                  't[s],vst_x[m],vst_y[m],car_x[m],car_y[m],vst_th[deg],car_th[deg],'
                  'eth_d[rad/s],delta_fb[rad],v_car[m/s],v_vst[m/s],f_hat[m/s2],servo_delta[rad]'
                  % len(planner.queue))
        fmt = '%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(header + '\n')
        for r in data:
            f.write(fmt % tuple(r) + '\n')
    print(f'MODE {mode}: {len(data)} rows, t=[{data[0,0]:.2f},{data[-1,0]:.2f}]s → {out}')

    if plot:
        plot_overview(mode, data, out)
    return data, out


def plot_overview(mode, data, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'figure.facecolor': '#1a1a19', 'axes.facecolor': '#22221f',
                         'axes.edgecolor': '#2c2c2a', 'axes.labelcolor': '#c3c2b7',
                         'text.color': '#ffffff', 'xtick.color': '#898781',
                         'ytick.color': '#898781', 'grid.color': '#2c2c2a',
                         'grid.alpha': 0.3, 'font.size': 8})
    if mode == 1:
        fig, axes = plt.subplots(2, 2, figsize=(12, 6))
        t = data[:, 0]
        axes[0, 0].plot(t, data[:, 1], label='rx'); axes[0, 0].plot(t, data[:, 2], label='ins_x')
        axes[0, 0].set_title('position step'); axes[0, 0].legend(); axes[0, 0].grid(True)
        axes[0, 1].plot(t, data[:, 3]); axes[0, 1].set_title('v'); axes[0, 1].grid(True)
        axes[1, 0].plot(t, data[:, 4]); axes[1, 0].set_title('f_hat'); axes[1, 0].grid(True)
        axes[1, 1].axis('off')
    else:
        fig, axes = plt.subplots(2, 2, figsize=(12, 7))
        ax = axes[0, 0]
        ax.plot(data[:, 3], data[:, 4], label='car', color='#e66767')
        ax.plot(data[:, 1], data[:, 2], '--', label='vst', color='#3987e5', alpha=0.6)
        ax.scatter(BEACONS[:, 0], BEACONS[:, 1], marker='x', c='#c98500', label='wp')
        ax.set_aspect('equal'); ax.legend(); ax.grid(True); ax.set_title('trajectory')
        t = data[:, 0]
        axes[0, 1].plot(t, data[:, 9], label='v_car'); axes[0, 1].plot(t, data[:, 10], label='v_vst')
        axes[0, 1].legend(); axes[0, 1].grid(True); axes[0, 1].set_title('velocity')
        axes[1, 0].plot(t, data[:, 7], label='eth_d'); axes[1, 0].plot(t, data[:, 8], label='delta_fb')
        axes[1, 0].legend(); axes[1, 0].grid(True); axes[1, 0].set_title('lateral')
        axes[1, 1].plot(t, data[:, 11], label='f_hat'); axes[1, 1].legend()
        axes[1, 1].grid(True); axes[1, 1].set_title('f_hat')
    png = out.with_suffix('.png')
    fig.savefig(str(png), dpi=150, facecolor='#1a1a19')
    plt.close()
    print(f'Saved: {png}')


def main():
    ap = argparse.ArgumentParser(description='β_ackerman 全闭环仿真')
    ap.add_argument('--mode', type=int, default=2, choices=[1, 2])
    ap.add_argument('--ckpt', type=str, default=None,
                    help='训练检查点 .pt (未折叠权重, 输入归一化); 默认解析部署 C 文件')
    ap.add_argument('--t-max', type=float, default=None)
    ap.add_argument('--f0', type=float, default=0.0, help='恒定残差扰动 [m/s²] (负=阻力/坡度)')
    ap.add_argument('--c2', type=float, default=0.0, help='二次阻力系数 (v|v| 项)')
    ap.add_argument('--plant-b0', type=float, default=LONG_B0)
    ap.add_argument('--plant-alpha', type=float, default=LONG_ALPHA)
    ap.add_argument('--tire', action='store_true',
                    help='横向改用轮胎模型: 侧偏角+侧偏刚度+μFz 饱和 (默认 wo 一阶惯性)')
    ap.add_argument('--m', type=float, default=2.0, help='整车质量 [kg] (--tire)')
    ap.add_argument('--iz', type=float, default=0.01, help='横摆惯量 [kg·m²] (--tire)')
    ap.add_argument('--cf', type=float, default=60.0, help='前轴侧偏刚度 [N/rad] (--tire)')
    ap.add_argument('--cr', type=float, default=60.0, help='后轴侧偏刚度 [N/rad] (--tire)')
    ap.add_argument('--mu', type=float, default=0.4,
                    help='轮胎摩擦系数, μ·g≈侧向加速度上限 (--tire)')
    ap.add_argument('--yaw-noise', type=float, default=0.0,
                    help='航向测量噪声 std [deg] (IMU 四元数抖动, OU 过程 τ=50ms)')
    ap.add_argument('--servo-tau', type=float, default=0.0,
                    help='舵机一阶滞后时间常数 [ms] (0=理想舵机)')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--out', type=str, default=None)
    ap.add_argument('--csv-ms', type=int, default=20,
                    help='CSV 打印周期 [ms] (车端 MODE2=100/UART限制, 仿真默认 20, 5=200Hz 全保真)')
    args = ap.parse_args()

    if args.ckpt:
        nn = NNPlanner(load_nn_from_pt(args.ckpt), normalize=True)
    else:
        nn = NNPlanner(load_nn_from_c())
    if args.tire:
        plant = TireBicyclePlant(alpha=args.plant_alpha, b0=args.plant_b0,
                                 f0=args.f0, c2=args.c2,
                                 m=args.m, iz=args.iz, cf=args.cf, cr=args.cr,
                                 mu=args.mu, tau_s=args.servo_tau * 1e-3)
    else:
        plant = CarPlant(alpha=args.plant_alpha, b0=args.plant_b0,
                         f0=args.f0, c2=args.c2,
                         tau_s=args.servo_tau * 1e-3)
    simulate(mode=args.mode, t_max=args.t_max, nn=nn, plant=plant,
             plot=args.plot, out=args.out, csv_ms=args.csv_ms,
             yaw_noise=args.yaw_noise)


if __name__ == '__main__':
    main()
