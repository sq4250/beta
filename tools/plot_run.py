"""
plot_run.py — 拖入 CSV 数据即可可视化

用法: python plot_run.py [csv_file]   (默认 run_data.csv)
      支持 7 列: t[s],car_x[m],car_y[m],car_th[deg],vst_x[m],vst_y[m],vst_th[deg]
"""
import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial import distance

PROJ = Path(__file__).resolve().parents[1]
DATA = PROJ / 'tools' / 'run_data.csv'

# ── 航点 (与 config.h 同步) ──
WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

# ── 颜色 ──
BG='#1a1a19'; SURF='#22221f'; P='#ffffff'; S='#c3c2b7'; M='#898781'; G='#2c2c2a'
RED='#e66767'; BLUE='#3987e5'; YELLOW='#c98500'; AQUA='#199e70'; ORANGE='#d95926'

def load(path):
    raw = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            p = line.split(',')
            if len(p) < 7: continue
            try: raw.append([float(x) for x in p[:7]])
            except ValueError: continue
    a = np.array(raw)
    if len(a) == 0:
        raise ValueError(f'No valid data in {path}')
    return (a[:,0], a[:,1], a[:,2], a[:,3], a[:,4], a[:,5], a[:,6])

def main(path):
    t,cx,cy,cth,vx,vy,vth = load(path)
    print(f'Loaded {len(t)} points, t=[{t[0]:.1f}, {t[-1]:.1f}]s')

    plt.rcParams.update({'figure.facecolor':BG,'axes.facecolor':SURF,'axes.edgecolor':G,
        'axes.labelcolor':S,'text.color':P,'xtick.color':M,'ytick.color':M,
        'grid.color':G,'grid.alpha':0.3,'legend.facecolor':SURF,'legend.edgecolor':G,
        'legend.labelcolor':S,'font.size':8,'axes.titlesize':10,'axes.labelsize':8})

    fig, axes = plt.subplots(2, 3, figsize=(16, 9)); fig.patch.set_facecolor(BG)

    # (0,0) 轨迹
    ax = axes[0,0]
    ax.plot(cx, cy, color=RED, lw=1.2, label='Real', alpha=0.85)
    ax.plot(vx, vy, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.scatter(WPS[:,0], WPS[:,1], c=YELLOW, s=70, marker='x', zorder=5,
               linewidths=2.5, label='Waypoint')
    for i,(wx,wy) in enumerate(WPS):
        lb = 'S' if i==0 else str(i)
        ax.annotate(lb, (wx,wy), textcoords='offset points', xytext=(8,8),
                    color=YELLOW, fontsize=8, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory', fontweight='bold')
    ax.set_aspect('equal'); ax.legend(fontsize=7, loc='lower left'); ax.grid(True, alpha=0.3)

    # (0,1) 实车航向
    ax = axes[0,1]
    ax.plot(t, cth, color=RED, lw=1.2, label='Real')
    ax.plot(t, vth, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Heading [deg]')
    ax.set_title('Heading', fontweight='bold'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (0,2) 航向误差
    ax = axes[0,2]
    eth = (vth - cth + 180) % 360 - 180
    ax.plot(t, eth, color=AQUA, lw=1.2)
    ax.axhline(0, color=M, lw=0.8, ls='--')
    ax.set_xlabel('t [s]'); ax.set_ylabel('eth [deg]')
    ax.set_title('Heading Error', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,0) 位置偏差 (sqrt((cx-vx)^2+(cy-vy)^2))
    ax = axes[1,0]
    pos_err = np.sqrt((cx-vx)**2 + (cy-vy)**2)
    ax.plot(t, pos_err, color=ORANGE, lw=1.5)
    ax.set_xlabel('t [s]'); ax.set_ylabel('pos_err [m]')
    ax.set_title('Position Error', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,1) 虚拟车速度 (差分法)
    ax = axes[1,1]
    if len(t) > 1:
        dt = np.diff(t); dx = np.diff(vx); dy = np.diff(vy)
        speed = np.sqrt((dx/dt)**2 + (dy/dt)**2)
        ax.plot(t[1:], speed, color=RED, lw=1.5)
    ax.set_xlabel('t [s]'); ax.set_ylabel('v [m/s]')
    ax.set_title('Virtual Car Speed', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,2) 航点距离
    ax = axes[1,2]
    for i, (wx, wy) in enumerate(WPS):
        d = np.sqrt((cx - wx)**2 + (cy - wy)**2)
        md = np.min(d)
        ax.text(0.05, 0.95 - i*0.13, f'WP{i} min={md*100:.1f}cm', transform=ax.transAxes,
                color=YELLOW, fontsize=9, va='top')
        ax.plot(t, d, lw=0.8, alpha=0.6, label=f'WP{i}')
    ax.set_xlabel('t [s]'); ax.set_ylabel('d [m]')
    ax.set_title('Distance to Waypoints', fontweight='bold')
    ax.legend(fontsize=6, loc='upper right'); ax.grid(True, alpha=0.3)

    fig.suptitle(f'Run: {Path(path).name}  |  {len(t)}pts  {t[-1]:.1f}s',
                 fontsize=11, fontweight='bold', color=P, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.94])

    out = Path(path).with_suffix('.png')
    fig.savefig(str(out), dpi=150, facecolor=BG, edgecolor='none'); plt.close()
    print(f'Saved: {out}')

    # 航点统计
    for i, (wx, wy) in enumerate(WPS):
        d = np.min(np.sqrt((cx - wx)**2 + (cy - wy)**2))
        print(f'  WP{i}: min_dist={d*100:.1f}cm')

if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else str(DATA)
    main(p)
