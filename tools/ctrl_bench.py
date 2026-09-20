"""
ctrl_bench.py — 控制器性能对比: 设计模型 (wo 一阶惯性) vs 轮胎侧滑模型

LQR 设计模型 (gen_lqr.py, 部署 3 态):
    x = [ey, eth, ėθ],  u = Δδ
    ẋ = [[0,v,0],[0,0,1],[0,0,-wo]]·x + [0,0,-wo·v/L]ᵀ·u      ← yaw 极点恒为 wo
轮胎模型的实际 yaw 极点 (线性化, Cf=Cr 中性转向):
    ẋ_ω = -(2·lf²·C/(Iz·v))·ω + (lf·C/Iz)·δ                  ← 极点 ∝ 1/v
    → 两模型在设计速度点 v* = 2·lf²·C/(Iz·wo) 处带宽相等

测试:
  A. 车道保持阶跃 — 等速直线参考 + 初始横向偏差, 测闭环响应 (超调/调节时间/稳态误差)
  B. 全场跟踪    — 复用 full_sim 的 6 航点运行, 按速度分箱统计 ey/eth

用法:
  python tools/ctrl_bench.py            # 全部测试 + 图 (out/ctrl_bench.png)
  python tools/ctrl_bench.py --quick    # 只跑测试 A
"""
import argparse
import sys
from pathlib import Path

import numpy as np

PROJ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJ / 'tools'))
import full_sim as fs

OUT = PROJ / 'tools' / 'out'

# ── 被控对象集合: (标签, 工厂, 说明) ──
WO_DESIGN = 15.0
PLANTS = [
    ('wo=15 设计模型', lambda: fs.CarPlant(wo=WO_DESIGN), '与 LQR 设计完全一致 (基准)'),
    ('wo=8 参数失配', lambda: fs.CarPlant(wo=8.0), '设计值一半 (慢一倍)'),
    ('wo=25 参数失配', lambda: fs.CarPlant(wo=25.0), '设计值 1.7× (快)'),
    ('轮胎 C=60', lambda: fs.TireBicyclePlant(cf=60, cr=60, mu=0.55), '极点随速度变 (v*=4.5m/s 处匹配)'),
    ('轮胎 C=200', lambda: fs.TireBicyclePlant(cf=200, cr=200, mu=0.55), '接近运动学 (刚性轮胎)'),
    ('轮胎 C=30', lambda: fs.TireBicyclePlant(cf=30, cr=30, mu=0.55), '软轮胎 (极点低 2×)'),
]


def tire_yaw_pole(v, C=60.0, m=2.0, iz=0.01, lf=0.075):
    """轮胎模型线性化 yaw 极点 [rad/s] (Cf=Cr, lf=lr → 无 vy 耦合)."""
    return 2.0 * lf * lf * C / (iz * v)


# ═══════════════════ 测试 A: 车道保持阶跃 ═══════════════════
def lane_step(plant, v_ref=3.0, ey0=0.30, t_end=4.0):
    """等速直线参考 + 初始横向偏差 → 闭环响应.

    VST: 沿 x 轴等速前进, δ_vst=0, 故 ey = y_car, eth = -θ_car, ėθ = -ω_car.
    返回 (t, ey, eth, delta) 数组.
    """
    vst_v = v_ref
    vst_x = 0.0
    car = np.zeros(5)          # x, y, theta, v, omega
    car[1] = ey0               # 初始横向偏差
    car[3] = v_ref             # 从参考速度起步 (隔离纵向)
    if hasattr(plant, 'vy'):
        plant.vy = 0.0
    if hasattr(plant, 'delta_act'):
        plant.delta_act = 0.0
    ladrc = fs.LADRC()

    ts, eys, eths, deltas = [], [], [], []
    n = int(t_end * fs.ISR_FREQ)
    div = 0
    thr = 0.0
    for k in range(n):
        if div == 0:
            vst_x += vst_v * fs.CTRL_DT               # 参考等速前进
            # 与 tasks.c frame_error 一致: ey = vst.y − car.y (θ_vst=0)
            ey = -car[1]
            eth = fs.wrap_pi(0.0 - car[2])
            eth_d = 0.0 - car[4]
            g = fs.lqr_lookup(car[3])
            delta = float(np.clip(g[0] * ey + g[1] * eth + g[2] * eth_d,
                                  -fs.SERVO_DELTA_MAX, fs.SERVO_DELTA_MAX))
            ex = vst_x - car[0]
            thr_l, _ = ladrc.step(car[3], vst_v, 0.0, ex, delta)
            thr = thr_l
            ts.append(k * fs.ISR_DT); eys.append(-car[1]); eths.append(eth); deltas.append(delta)
        plant.step(car, thr, delta, fs.ISR_DT)
        div = (div + 1) % fs.TRACKER_DIV
    return map(np.array, (ts, eys, eths, deltas))


def step_metrics(t, ey, ey0, band):
    """超调 / 调节时间(±band) / 稳态误差."""
    overshoot = max(0.0, (np.max(np.abs(ey)) - abs(ey0)) / abs(ey0)) * 100
    ss = np.mean(np.abs(ey[t > t[-1] * 0.75]))
    out_band = np.where(np.abs(ey) > band)[0]
    settle = t[out_band[-1]] if len(out_band) else 0.0
    return dict(overshoot=overshoot, settle=settle, ss=ss)


# ═══════════════════ 测试 B: 全场跟踪 (按速度分箱) ═══════════════════
def run_track(nn, plant, out_csv):
    d, _ = fs.simulate(mode=2, nn=nn, plant=plant, out=out_csv)
    return d


def binned_errors(d):
    """按 VST 速度分箱统计 |ey| / |eth| 的 RMS."""
    t, vx, vy, cx, cy, vth, cth = d[:, 0], d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 5], d[:, 6]
    v = d[:, 10]
    thr = np.deg2rad(vth)
    dx, dy = vx - cx, vy - cy
    ey = -dx * np.sin(thr) + dy * np.cos(thr)
    eth = (vth - cth + 180) % 360 - 180
    m = t > 2.0
    bins = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5.5)]
    out = []
    for lo, hi in bins:
        sel = m & (v >= lo) & (v < hi)
        out.append((f'{lo}-{hi}', np.sqrt(np.mean(ey[sel] ** 2)) * 100 if sel.sum() else np.nan,
                    np.sqrt(np.mean(eth[sel] ** 2)) if sel.sum() else np.nan, int(sel.sum())))
    return out, ey, eth, v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true', help='只跑测试 A')
    ap.add_argument('--speeds', type=float, nargs='+', default=[1.0, 2.0, 3.0, 5.0])
    ap.add_argument('--amp-small', type=float, default=0.05, help='小信号阶跃幅值 [m]')
    ap.add_argument('--amp-large', type=float, default=0.30, help='大信号阶跃幅值 [m]')
    args = ap.parse_args()

    # ── 测试 A: 小信号 (线性区) + 大信号 (饱和区) ──
    cases = [(args.amp_small, 0.005), (args.amp_large, 0.02)]   # (阶跃幅值, 调节带)
    results = {}
    traces = {}
    for ey0, band in cases:
        print(f'\n═══ 测试 A: 车道保持阶跃 {ey0*100:.0f}cm → 0 (调节带 ±{band*100:.1f}cm), 等速直线 ═══')
        for v_ref in args.speeds:
            print(f'\n  v_ref = {v_ref} m/s')
            print('  %-16s %9s %9s %9s %9s' % ('被控对象', '超调[%]', '调节[s]', '稳态[cm]', 'Δδ峰[rad]'))
            for tag, mk, _ in PLANTS:
                t, ey, eth, dl = lane_step(mk(), v_ref=v_ref, ey0=ey0)
                _ = eth
                m = step_metrics(t, ey, ey0, band)
                results[(ey0, v_ref, tag)] = (m, np.max(np.abs(dl)))
                if abs(v_ref - args.speeds[len(args.speeds) // 2]) < 1e-9:
                    traces[(ey0, tag)] = (t, ey)
                print('  %-16s %9.1f %9.3f %9.3f %9.3f'
                      % (tag, m['overshoot'], m['settle'], m['ss'] * 100, np.max(np.abs(dl))))

    if not args.quick:
        # ── 测试 B ──
        print('\n═══ 测试 B: 全场 6 航点跟踪 (新网络 gpmed_v4, 按速度分箱 RMS) ═══')
        nn = fs.NNPlanner(fs.load_nn_from_c())
        bin_lbl = '  '.join('%9s' % f'{lo}-{hi}m/s' for lo, hi in [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5.5)])
        table = []
        tmp_csv = OUT / '_bench_tmp.csv'
        for tag, mk, _ in PLANTS:
            d = run_track(nn, mk(), str(tmp_csv))
            bins, ey, eth, v = binned_errors(d)
            table.append((tag, bins))
        tmp_csv.unlink(missing_ok=True)   # 临时数据不留在 out/
        for unit, key in [('ey RMS [cm]', 1), ('eth RMS [deg]', 2)]:
            print(f'\n  {unit}')
            print('  %-16s %s' % ('被控对象', bin_lbl))
            for tag, bins in table:
                print('  %-16s %s' % (tag, '  '.join(
                    '%9.2f' % b[key] if not np.isnan(b[key]) else '        -' for b in bins)))

    # ── 图 ──
    _plot(results, traces, args.speeds, args.amp_small, args.amp_large)
    print(f'\nSaved: {OUT / "ctrl_bench.png"}')


def _plot(results, traces, speeds, amp_small, amp_large):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    BG = '#1a1a19'; SURF = '#22221f'; P = '#ffffff'; S = '#c3c2b7'; M = '#898781'; G = '#2c2c2a'
    COL = ['#3987e5', '#e66767', '#d95926', '#199e70', '#c98500', '#a855f7']
    plt.rcParams.update({'figure.facecolor': BG, 'axes.facecolor': SURF, 'axes.edgecolor': G,
                         'axes.labelcolor': S, 'text.color': P, 'xtick.color': M,
                         'ytick.color': M, 'grid.color': G, 'grid.alpha': 0.3,
                         'legend.facecolor': SURF, 'legend.edgecolor': G,
                         'legend.labelcolor': S, 'font.size': 9,
                         'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
                         'axes.unicode_minus': False})
    fig, axes = plt.subplots(1, 3, figsize=(19.2, 6.4))
    fig.patch.set_facecolor(BG)

    # (0) 小信号 ey(t) 响应曲线 (中间速度)
    ax = axes[0]
    vmid = speeds[len(speeds) // 2]
    for i, (tag, mk, _) in enumerate(PLANTS):
        if (amp_small, tag) in traces:
            t, ey = traces[(amp_small, tag)]
            m = t <= 1.5
            ax.plot(t[m], ey[m] * 100, color=COL[i], lw=1.8, label=tag)
    ax.axhline(0, color=M, lw=0.8, ls='--')
    ax.set_xlabel('t [s]'); ax.set_ylabel('横向误差 ey [cm]')
    ax.set_title(f'小信号响应 (v={vmid}m/s, 初始偏差 {amp_small*100:.0f}cm)', fontweight='bold')
    ax.legend(fontsize=7.5); ax.grid(True, alpha=0.3)

    # (1) 机理: yaw 极点 vs 速度
    ax = axes[1]
    vv = np.linspace(0.5, 5.5, 200)
    for i, (tag, mk, _) in enumerate(PLANTS):
        if '轮胎 C=' in tag:
            C = float(tag.split('C=')[1])
            ax.plot(vv, [tire_yaw_pole(v, C) for v in vv], color=COL[i], lw=1.8, label=tag)
    ax.axhline(WO_DESIGN, color=COL[0], lw=1.8, ls='--', label='设计模型 wo=15 (恒定)')
    ax.set_xlabel('速度 [m/s]'); ax.set_ylabel('yaw 极点 [rad/s]')
    ax.set_title('机理: 设计带宽恒定 vs 轮胎模型 ∝1/v', fontweight='bold')
    ax.legend(fontsize=7.5); ax.grid(True, alpha=0.3); ax.set_ylim(0, 50)

    # (2) 调节时间 vs 速度 (小信号)
    ax = axes[2]
    for i, (tag, mk, _) in enumerate(PLANTS):
        ys = [results[(amp_small, v, tag)][0]['settle'] for v in speeds]
        ax.plot(speeds, ys, 'o-', color=COL[i], lw=1.6, ms=5, label=tag)
    ax.set_xlabel('速度 [m/s]'); ax.set_ylabel('调节时间 [s]')
    ax.set_title(f'小信号调节时间 (带 ±0.5cm)', fontweight='bold')
    ax.legend(fontsize=7.5); ax.grid(True, alpha=0.3)
    fig.suptitle('控制器性能: 设计模型 (wo=15 一阶惯性) vs 轮胎侧滑模型  |  车道保持阶跃',
                 fontsize=13, fontweight='bold', color=P, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(str(OUT / 'ctrl_bench.png'), dpi=100, facecolor=BG)
    plt.close(fig)


if __name__ == '__main__':
    main()
