"""
plot_resync.py — VST 重同步开/关对比图 (1920×1080, 深色主题)

四图: 全场轨迹 / home 局部放大 (10cm 撞线圈) / |VST−car| 时间曲线 / 末段 d_home
"""
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
ON = DATA / 'sim_run.csv'             # 重同步开 (与 OFF 同网络的对照)
OFF = DATA / 'sim_run_noresync.csv'   # 关重同步

WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

BG = '#1a1a19'; SURF = '#22221f'; P = '#ffffff'; S = '#c3c2b7'; M = '#898781'; G = '#2c2c2a'
RED = '#e66767'; BLUE = '#3987e5'; YELLOW = '#c98500'; AQUA = '#199e70'; ORANGE = '#d95926'
FIG_W, FIG_H, FIG_DPI = 12.8, 7.2, 150

STYLE = {'figure.facecolor': BG, 'axes.facecolor': SURF, 'axes.edgecolor': G,
         'axes.labelcolor': S, 'text.color': P, 'xtick.color': M, 'ytick.color': M,
         'grid.color': G, 'grid.alpha': 0.3, 'legend.facecolor': SURF,
         'legend.edgecolor': G, 'legend.labelcolor': S, 'font.size': 9,
         'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
         'axes.unicode_minus': False}


def load(path):
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
    return b[:, 0], b[:, 1], b[:, 2], b[:, 3], b[:, 4]


def main():
    d_real = load(REAL)
    d_on = load(ON)
    d_off = load(OFF)

    def gap(d):
        return np.hypot(d[1] - d[3], d[2] - d[4]) * 100

    plt.rcParams.update(STYLE)
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1], hspace=0.34, wspace=0.24)

    # (0,0) 全场轨迹
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(d_real[3], d_real[4], color=RED, lw=1.3, label='Real (resync 零触发)')
    ax.plot(d_on[3], d_on[4], color=AQUA, lw=1.0, label='Sim 重同步开')
    ax.plot(d_off[3], d_off[4], color=ORANGE, lw=1.0, label='Sim 关重同步 (绕回)')
    ax.scatter(WPS[:, 0], WPS[:, 1], c=YELLOW, s=60, marker='x', zorder=5, linewidths=2)
    for i, (wx, wy) in enumerate(WPS):
        ax.annotate('S' if i == 0 else str(i), (wx, wy), textcoords='offset points',
                    xytext=(6, 6), color=YELLOW, fontsize=8, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory', fontweight='bold')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3); ax.margins(0.08)
    ax.legend(fontsize=7.5, loc='upper left')

    # (0,1) home 局部放大
    ax = fig.add_subplot(gs[0, 1])
    for d, c, tag in [(d_real, RED, 'Real'), (d_on, AQUA, '重同步开'), (d_off, ORANGE, '关重同步')]:
        m = np.hypot(d[3], d[4]) < 1.15
        ax.plot(d[3][m], d[4][m], color=c, lw=1.4, label=tag)
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(0.10 * np.cos(th), 0.10 * np.sin(th), color=YELLOW, lw=1.2, ls='--',
            label='TOL_XY=10cm 撞线圈')
    ax.scatter([0], [0], c=YELLOW, s=50, marker='x', zorder=6, linewidths=2)
    # 实车进场方向箭头 (WP2→home)
    ax.annotate('', xy=(0.55, 0.23), xytext=(1.05, 0.44),
                arrowprops=dict(arrowstyle='->', color=S, lw=1.0, alpha=0.6))
    ax.text(0.78, 0.36, '来自 WP2', color=S, fontsize=8, alpha=0.7)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('home 附近放大 (绕回可见)', fontweight='bold')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7.5, loc='lower right')

    # (1,0) |VST−car| 距离
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(d_real[0], gap(d_real), color=RED, lw=1.2, label='Real')
    ax.plot(d_on[0], gap(d_on), color=AQUA, lw=1.0, label='Sim 重同步开')
    ax.plot(d_off[0], gap(d_off), color=ORANGE, lw=1.0, label='Sim 关重同步')
    ax.axhline(5.0, color=YELLOW, lw=1.0, ls='--', label='5cm 重同步阈值')
    ax.set_xlabel('t [s]'); ax.set_ylabel('|VST − car| [cm]')
    ax.set_title('VST-car 距离 (实车全程 <5cm, 零触发)', fontweight='bold')
    ax.set_ylim(0, 15); ax.legend(fontsize=7.5); ax.grid(True, alpha=0.3)

    # (1,1) 末段 d_home (第一次接近 → 错过 → 绕回 → 命中)
    ax = fig.add_subplot(gs[1, 1])
    for d, c, tag in [(d_real, RED, 'Real'), (d_on, AQUA, '重同步开'), (d_off, ORANGE, '关重同步')]:
        m = d[0] >= 8.0
        ax.plot(d[0][m], np.hypot(d[3][m], d[4][m]), color=c, lw=1.1, label=tag)
    ax.axhline(0.10, color=YELLOW, lw=1.0, ls='--', label='10cm 撞线圈')
    ax.set_xlabel('t [s]'); ax.set_ylabel('d_home [m]')
    ax.set_title('末段 d_home: 关重同步 → 减小增大再减小 (绕回)', fontweight='bold')
    ax.set_ylim(0, 3.5); ax.legend(fontsize=7.5); ax.grid(True, alpha=0.3)

    fig.suptitle('VST 重同步开/关对比  |  Resync: real=零触发兜底, sim-off 漂移→错过→绕回',
                 fontsize=13, fontweight='bold', color=P, y=0.98)
    out = OUT / 'resync_compare.png'
    fig.savefig(str(out), dpi=FIG_DPI, facecolor=BG, edgecolor='none')
    plt.close()
    print(f'Saved: {out}')


if __name__ == '__main__':
    main()
