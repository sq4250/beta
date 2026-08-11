"""
plot_run.py — CSV 数据可视化, 分三图: 轨迹总览 / 横向 / 纵向

用法: python plot_run.py [csv_file]   (默认 run_data.csv)
"""
import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
DATA = PROJ / 'tools' / 'run_data.csv'

WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

BG='#1a1a19'; SURF='#22221f'; P='#ffffff'; S='#c3c2b7'; M='#898781'; G='#2c2c2a'
RED='#e66767'; BLUE='#3987e5'; YELLOW='#c98500'; AQUA='#199e70'; ORANGE='#d95926'
PURPLE='#a855f7'

STYLE = {'figure.facecolor':BG,'axes.facecolor':SURF,'axes.edgecolor':G,
    'axes.labelcolor':S,'text.color':P,'xtick.color':M,'ytick.color':M,
    'grid.color':G,'grid.alpha':0.3,'legend.facecolor':SURF,'legend.edgecolor':G,
    'legend.labelcolor':S,'font.size':8}

def load(path):
    raw = []; ncols = 7
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            p = line.split(',')
            if len(p) > ncols:
                try:
                    _ = [float(x) for x in p]; ncols = len(p); break
                except ValueError: continue
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            p = line.split(',')
            if len(p) >= ncols:
                try: raw.append([float(x) for x in p[:ncols]])
                except ValueError: continue
    b = np.array(raw)
    if len(b) == 0: raise ValueError(f'No valid data in {path}')
    t, vx, vy, cx, cy, vth, cth = b[:,0], b[:,1], b[:,2], b[:,3], b[:,4], b[:,5], b[:,6]
    eth_d = delta_fb = None
    if ncols >= 8: eth_d = b[:,7]
    if ncols >= 9: delta_fb = b[:,8]
    return t, cx, cy, cth, vx, vy, vth, eth_d, delta_fb

def sym_ylim(ax, data, margin=1.1, floor=0.1):
    r = max(abs(np.min(data)), abs(np.max(data)), floor) * margin
    ax.set_ylim(-r, r)

def main(path):
    t,cx,cy,cth,vx,vy,vth,eth_d,delta_fb = load(path)
    has_extra = eth_d is not None
    print(f'Loaded {len(t)} pts, t=[{t[0]:.1f},{t[-1]:.1f}]s  extra={has_extra}')

    vth_rad = np.deg2rad(vth)
    dx = vx - cx; dy = vy - cy
    ex =  dx * np.cos(vth_rad) + dy * np.sin(vth_rad)
    ey = -dx * np.sin(vth_rad) + dy * np.cos(vth_rad)
    eth = (vth - cth + 180) % 360 - 180

    plt.rcParams.update(STYLE)

    nrows = 2
    fig = plt.figure(figsize=(16, 10)); fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(nrows, 2, height_ratios=[1.2, 1.2],
                          width_ratios=[1, 1.3], hspace=0.30, wspace=0.3)

    # (0,0) 轨迹
    ax = fig.add_subplot(gs[0,0])
    ax.plot(cx, cy, color=RED, lw=1.2, label='Real', alpha=0.85)
    ax.plot(vx, vy, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.scatter(WPS[:,0], WPS[:,1], c=YELLOW, s=50, marker='x', zorder=5, linewidths=2, label='WP')
    for i,(wx,wy) in enumerate(WPS):
        ax.annotate('S' if i==0 else str(i), (wx,wy), textcoords='offset points',
                    xytext=(6,6), color=YELLOW, fontsize=8, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory', fontweight='bold')
    ax.set_aspect('equal'); ax.legend(fontsize=6, loc='lower left'); ax.grid(True, alpha=0.3)

    # (0,1) 航向
    ax = fig.add_subplot(gs[0,1])
    ax.plot(t, cth, color=RED, lw=1.2, label='Real')
    ax.plot(t, vth, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Heading [deg]')
    ax.set_title('Heading', fontweight='bold'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    if has_extra:
        # (1,:) ey + eth + eth_d + Δδ_fb 叠放 (full width, 自动缩放)
        # 缩放 eth_d, Δδ_fb 使量级与 ey[cm]/eth[deg] 大致对齐
        ref_rms = max(np.std(ey*100), np.std(np.abs(eth)), 0.5)
        s_eth_d = min(ref_rms / max(np.std(np.abs(eth_d)), 0.01), 40.0)
        s_dfb   = min(ref_rms / max(np.std(np.abs(delta_fb)), 0.001), 60.0)

        ax = fig.add_subplot(gs[1,:])
        ax.plot(t, ey*100,         color=ORANGE, lw=1.2, label='ey [cm]')
        ax.plot(t, eth,            color=AQUA,   lw=1.2, label='eth [deg]')
        ax.plot(t, eth_d * s_eth_d, color=PURPLE, lw=1.0, label=f'eth_d ×{s_eth_d:.0f} [rad/s]')
        ax.plot(t, delta_fb * s_dfb, color=YELLOW, lw=1.0, label=f'Δδ_fb ×{s_dfb:.0f} [rad]')
        ax.axhline(0, color=M, lw=0.8, ls='--')
        sym_ylim(ax, np.concatenate([ey*100, eth, eth_d * s_eth_d, delta_fb * s_dfb]))
        ax.set_xlabel('t [s]'); ax.set_ylabel('scaled')
        ax.set_title(f'Error States + Control  (ey[cm], eth[deg], eth_d×{s_eth_d:.0f}, Δδ_fb×{s_dfb:.0f})', fontweight='bold')
        ax.legend(fontsize=6, loc='upper left'); ax.grid(True, alpha=0.3)
    else:
        # (1,0) ey + eth
        ax = fig.add_subplot(gs[1,0])
        ax.plot(t, ey*100, color=ORANGE, lw=1.2, label='ey [cm]')
        ax.plot(t, eth, color=AQUA, lw=1.2, label='eth [deg]')
        sym_ylim(ax, np.concatenate([ey*100, eth]))
        ax.axhline(0, color=M, lw=0.8, ls='--')
        ax.set_xlabel('t [s]'); ax.set_ylabel('ey/eth')
        ax.set_title('ey + eth', fontweight='bold')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    fig.suptitle(f'Run: {Path(path).name}  |  {len(t)}pts  {t[-1]:.1f}s',
                 fontsize=11, fontweight='bold', color=P, y=0.98)

    out = Path(path).with_suffix('.png')
    fig.savefig(str(out), dpi=150, facecolor=BG, edgecolor='none'); plt.close()
    print(f'Saved: {out}')

    # 统计
    dist = np.sqrt(ex**2 + ey**2)
    print(f'  ey STD={np.std(ey)*100:.1f}cm max={np.max(np.abs(ey))*100:.1f}cm')
    print(f'  ex STD={np.std(ex)*100:.1f}cm max={np.max(np.abs(ex))*100:.1f}cm')
    print(f'  eth STD={np.std(eth):.1f}deg max={np.max(np.abs(eth)):.1f}deg')
    if has_extra:
        print(f'  eth_d STD={np.std(eth_d):.2f}rad/s max={np.max(np.abs(eth_d)):.2f}rad/s')
        print(f'  Δδ_fb STD={np.std(delta_fb):.3f}rad max={np.max(np.abs(delta_fb)):.3f}rad')

if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else str(DATA)
    main(p)
