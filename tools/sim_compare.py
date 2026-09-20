"""
sim_compare.py — 仿真 vs 实车轨迹对比 (1920×1080, 深色主题)

用法: python sim_compare.py [real_csv] [sim_csv]
默认: run_data.csv (实车) vs sim_run.csv (full_sim.py 输出)

四图: 轨迹叠加 + 速度曲线 + 三项误差统计表 (ey/ex/eth STD+max)
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
TOOLS = PROJ / 'tools'
DATA = TOOLS / 'data'
OUT = TOOLS / 'out'
REAL = DATA / 'run_data.csv'
SIM = DATA / 'sim_run.csv'

# 补录: 当年 6 点坐标 (与 plot_run.py 一致)
WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

BG = '#1a1a19'; SURF = '#22221f'; P = '#ffffff'; S = '#c3c2b7'; M = '#898781'; G = '#2c2c2a'
RED = '#e66767'; BLUE = '#3987e5'; YELLOW = '#c98500'; AQUA = '#199e70'; ORANGE = '#d95926'
FIG_W, FIG_H, FIG_DPI = 12.8, 7.2, 150

STYLE = {'figure.facecolor': BG, 'axes.facecolor': SURF, 'axes.edgecolor': G,
         'axes.labelcolor': S, 'text.color': P, 'xtick.color': M, 'ytick.color': M,
         'grid.color': G, 'grid.alpha': 0.3, 'legend.facecolor': SURF,
         'legend.edgecolor': G, 'legend.labelcolor': S, 'font.size': 9}


def load(path):
    """同 plot_run.py: # 注释跳过, 列数自动探测"""
    raw = []; ncols = 7
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split(',')
            if len(p) > ncols:
                try:
                    _ = [float(x) for x in p]
                    ncols = len(p)
                    break
                except ValueError:
                    continue
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split(',')
            if len(p) >= ncols:
                try:
                    raw.append([float(x) for x in p[:ncols]])
                except ValueError:
                    continue
    b = np.array(raw)
    if len(b) == 0:
        raise ValueError(f'No valid data in {path}')
    return b[:, 0], b[:, 1], b[:, 2], b[:, 3], b[:, 4], b[:, 5], b[:, 6], b[:, 9], b[:, 10]


def errs(vx, vy, vth, cx, cy, cth):
    vth_rad = np.deg2rad(vth)
    dx, dy = vx - cx, vy - cy
    ex = dx * np.cos(vth_rad) + dy * np.sin(vth_rad)
    ey = -dx * np.sin(vth_rad) + dy * np.cos(vth_rad)
    eth = (vth - cth + 180) % 360 - 180
    return ex, ey, eth


def stat_lines(ex, ey, eth):
    return [f'ey  STD={np.std(ey)*100:.1f}cm  max={np.max(np.abs(ey))*100:.1f}cm',
            f'ex  STD={np.std(ex)*100:.1f}cm  max={np.max(np.abs(ex))*100:.1f}cm',
            f'eth STD={np.std(eth):.1f}deg  max={np.max(np.abs(eth)):.1f}deg']


def main():
    real_csv = Path(sys.argv[1]) if len(sys.argv) > 1 else REAL
    sim_csv = Path(sys.argv[2]) if len(sys.argv) > 2 else SIM
    out = Path(sys.argv[3]) if len(sys.argv) > 3 else OUT / 'sim_vs_real.png'
    t_r, vx_r, vy_r, cx_r, cy_r, vth_r, cth_r, vc_r, vv_r = load(real_csv)
    t_s, vx_s, vy_s, cx_s, cy_s, vth_s, cth_s, vc_s, vv_s = load(sim_csv)

    ex_r, ey_r, eth_r = errs(vx_r, vy_r, vth_r, cx_r, cy_r, cth_r)
    ex_s, ey_s, eth_s = errs(vx_s, vy_s, vth_s, cx_s, cy_s, cth_s)

    for tag, d in [('REAL', [ex_r, ey_r, eth_r]), ('SIM', [ex_s, ey_s, eth_s])]:
        print(f'[{tag}] ' + '  '.join(stat_lines(*d)))

    plt.rcParams.update(STYLE)
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], height_ratios=[1.15, 1],
                          hspace=0.32, wspace=0.22)

    # (0,0) 轨迹: 实车 vs 仿真 (仿真轨迹平移到实车起点)
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(cx_r, cy_r, color=RED, lw=1.4, label='Real car', alpha=0.9)
    ax.plot(vx_r, vy_r, color=RED, lw=0.7, alpha=0.35, ls='--', label='Real vst')
    ax.plot(cx_s, cy_s, color=AQUA, lw=1.2, label='Sim car', alpha=0.9)
    ax.plot(vx_s, vy_s, color=AQUA, lw=0.6, alpha=0.35, ls='--', label='Sim vst')
    ax.scatter(WPS[:, 0], WPS[:, 1], c=YELLOW, s=60, marker='x', zorder=5,
               linewidths=2, label='WP')
    for i, (wx, wy) in enumerate(WPS):
        ax.annotate('S' if i == 0 else str(i), (wx, wy),
                    textcoords='offset points', xytext=(6, 6),
                    color=YELLOW, fontsize=8, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory: real vs sim', fontweight='bold')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3); ax.margins(0.08)
    ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.0))

    # (0,1) 速度
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t_r, vc_r, color=RED, lw=1.2, label='v real')
    ax.plot(t_s, vc_s, color=AQUA, lw=1.2, label='v sim')
    ax.plot(t_s, vv_s, color=AQUA, lw=0.8, alpha=0.4, ls='--', label='vst sim')
    ax.set_xlabel('t [s]'); ax.set_ylabel('v [m/s]')
    ax.set_title('Velocity', fontweight='bold')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (1,0) ey + eth 叠加
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t_r, ey_r * 100, color=RED, lw=1.0, alpha=0.85, label='ey real [cm]')
    ax.plot(t_r, eth_r, color=RED, lw=0.7, alpha=0.4, ls='--', label='eth real [deg]')
    ax.plot(t_s, ey_s * 100, color=AQUA, lw=1.0, label='ey sim [cm]')
    ax.plot(t_s, eth_s, color=AQUA, lw=0.7, ls='--', label='eth sim [deg]')
    ax.axhline(0, color=M, lw=0.8, ls='--')
    ax.set_xlabel('t [s]')
    ax.set_title('ey + eth', fontweight='bold')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (1,1) 统计表
    ax = fig.add_subplot(gs[1, 1]); ax.axis('off')
    lines = ['   ey/ex/eth        REAL        SIM'] + \
            [' ' * 24] + \
            [f'  {ln[:17]:<17}{ln[17:]:<10}' for ln in stat_lines(ey_r, ex_r, eth_r)] + \
            [f'  {ln[:17]:<17}{ln[17:]:<10}' for ln in stat_lines(ey_s, ex_s, eth_s)]
    txt = '\n'.join(lines)
    ax.text(0.02, 0.98, txt, transform=ax.transAxes, ha='left', va='top',
            fontsize=11, color=P, family='monospace',
            bbox=dict(boxstyle='round,pad=0.6', facecolor=SURF, edgecolor=G, alpha=0.92))

    fig.suptitle(f'FullSim vs Real  |  {real_csv.name} ({len(t_r)}pts {t_r[-1]:.1f}s)  |  '
                 f'{sim_csv.name} ({len(t_s)}pts {t_s[-1]:.1f}s)',
                 fontsize=12, fontweight='bold', color=P, y=0.98)
    fig.subplots_adjust(right=0.76, top=0.92)

    fig.savefig(str(out), dpi=FIG_DPI, facecolor=BG, edgecolor='none')
    plt.close()
    print(f'Saved: {out}')


if __name__ == '__main__':
    main()
