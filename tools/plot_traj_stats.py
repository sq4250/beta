"""
plot_traj_stats.py — 纯轨迹图 (无数据波形) + 三项主要误差统计标注

用法: python plot_traj_stats.py [csv_file]   (默认 run_data.csv)
输出: <csv 同名>_traj.png, 控制台同时打印三项误差统计

CSV 格式同 plot_run.py: t, vst_x, vst_y, car_x, car_y, vst_th[deg], car_th[deg], ...
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
DATA = PROJ / 'tools' / 'run_data.csv'

# 补录: 当年 6 点坐标 (与 plot_run.py / animate_run.py 一致)
WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

BG = '#1a1a19'; SURF = '#22221f'; P = '#ffffff'; S = '#c3c2b7'; M = '#898781'; G = '#2c2c2a'
RED = '#e66767'; BLUE = '#3987e5'; YELLOW = '#c98500'

# 输出分辨率: figsize(12.8, 7.2) × dpi150 = 1920×1080 (16:9, 视频/报告友好)
FIG_W, FIG_H, FIG_DPI = 12.8, 7.2, 150

STYLE = {'figure.facecolor': BG, 'axes.facecolor': SURF, 'axes.edgecolor': G,
         'axes.labelcolor': S, 'text.color': P, 'xtick.color': M, 'ytick.color': M,
         'grid.color': G, 'grid.alpha': 0.3, 'legend.facecolor': SURF,
         'legend.edgecolor': G, 'legend.labelcolor': S, 'font.size': 9}


def load(path):
    """同 plot_run.py: # 注释行跳过, 列数自动探测, 取前 ncols 列"""
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
    t, vx, vy, cx, cy = b[:, 0], b[:, 1], b[:, 2], b[:, 3], b[:, 4]
    vth, cth = b[:, 5], b[:, 6]
    return t, cx, cy, cth, vx, vy, vth


def stats(t, cx, cy, cth, vx, vy, vth):
    """三项主要误差: ex/ey (VST 系投影, m) + eth (航向, deg)"""
    vth_rad = np.deg2rad(vth)
    dx = vx - cx
    dy = vy - cy
    ex = dx * np.cos(vth_rad) + dy * np.sin(vth_rad)
    ey = -dx * np.sin(vth_rad) + dy * np.cos(vth_rad)
    eth = (vth - cth + 180) % 360 - 180
    return ex, ey, eth


def stat_lines(ex, ey, eth):
    return [
        f'ey  STD={np.std(ey) * 100:.1f}cm  max={np.max(np.abs(ey)) * 100:.1f}cm',
        f'ex  STD={np.std(ex) * 100:.1f}cm  max={np.max(np.abs(ex)) * 100:.1f}cm',
        f'eth STD={np.std(eth):.1f}deg  max={np.max(np.abs(eth)):.1f}deg',
    ]


def main(path):
    t, cx, cy, cth, vx, vy, vth = load(path)
    ex, ey, eth = stats(t, cx, cy, cth, vx, vy, vth)
    lines = stat_lines(ex, ey, eth)
    print(f'Loaded {len(t)} pts, t=[{t[0]:.1f},{t[-1]:.1f}]s')
    for ln in lines:
        print(f'  {ln}')

    plt.rcParams.update(STYLE)
    fig = plt.figure(figsize=(FIG_W, FIG_H))
    fig.patch.set_facecolor(BG)
    ax = fig.add_subplot(111)

    ax.plot(cx, cy, color=RED, lw=1.2, label='Real', alpha=0.9)
    ax.plot(vx, vy, color=BLUE, lw=0.8, alpha=0.6, ls='--', label='Virtual')
    ax.scatter(WPS[:, 0], WPS[:, 1], c=YELLOW, s=60, marker='x', zorder=5,
               linewidths=2, label='WP')
    for i, (wx, wy) in enumerate(WPS):
        ax.annotate('S' if i == 0 else str(i), (wx, wy),
                    textcoords='offset points', xytext=(6, 6),
                    color=YELLOW, fontsize=8, fontweight='bold')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.margins(0.08)

    # 图例放图外右侧, 不与轨迹/标注框重叠 (轨迹起点在原点, 图内左下必压数据)
    ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.0))
    fig.subplots_adjust(right=0.76, top=0.92)

    # ── 误差统计标注框: 放在离轨迹质心最远的角, 尽量不压轨迹 ──
    x0, x1, y0, y1 = np.min(cx), np.max(cx), np.min(cy), np.max(cy)
    mx, my = np.mean(cx), np.mean(cy)
    corners = [(x0, y1, 'left', 'top'), (x1, y1, 'right', 'top'),
               (x0, y0, 'left', 'bottom'), (x1, y0, 'right', 'bottom')]
    corner = max(corners, key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2)
    _, _, halign, valign = corner

    ax.text(0.02 if halign == 'left' else 0.98,
            0.98 if valign == 'top' else 0.02,
            '\n'.join(lines),
            transform=ax.transAxes, ha=halign, va=valign,
            fontsize=10, color=P, family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor=SURF, edgecolor=G, alpha=0.92))

    fig.suptitle(f'Trajectory  |  {Path(path).name}  |  {len(t)}pts  {t[-1]:.1f}s',
                 fontsize=12, fontweight='bold', color=P, y=0.97)

    out = Path(path).with_name(Path(path).stem + '_traj.png')
    fig.savefig(str(out), dpi=FIG_DPI, facecolor=BG, edgecolor='none')
    plt.close()
    print(f'Saved: {out}')


if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else str(DATA)
    main(p)
