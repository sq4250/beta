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
    servo = vst_delta = vst_yaw = car_yaw = gyro_z = w_ff = a_ff = vst_v = car_v = f_hat = None
    if ncols >= 14: servo, vst_delta, vst_yaw, car_yaw, gyro_z, w_ff, a_ff = b[:,7], b[:,8], b[:,9], b[:,10], b[:,11], b[:,12], b[:,13]
    if ncols >= 17: vst_v, car_v, f_hat = b[:,14], b[:,15], b[:,16]
    return t, cx, cy, cth, vx, vy, vth, servo, vst_delta, vst_yaw, car_yaw, gyro_z, w_ff, a_ff, vst_v, car_v, f_hat

def sym_ylim(ax, data, margin=1.1, floor=0.1):
    r = max(abs(np.min(data)), abs(np.max(data)), floor) * margin
    ax.set_ylim(-r, r)

def main(path):
    t,cx,cy,cth,vx,vy,vth,servo,vst_delta,vst_yaw,car_yaw,gyro_z,w_ff,a_ff,vst_v,car_v,f_hat = load(path)
    has_yaw = vst_yaw is not None
    has_ed  = vst_delta is not None
    has_ff  = w_ff is not None
    has_vf  = vst_v is not None
    print(f'Loaded {len(t)} pts, t=[{t[0]:.1f},{t[-1]:.1f}]s  yaw={has_yaw} ed={has_ed} vf={has_vf}')

    vth_rad = np.deg2rad(vth)
    dx = vx - cx; dy = vy - cy
    ex =  dx * np.cos(vth_rad) + dy * np.sin(vth_rad)
    ey = -dx * np.sin(vth_rad) + dy * np.cos(vth_rad)
    eth = (vth - cth + 180) % 360 - 180

    plt.rcParams.update(STYLE)

    # ═══════════════════════════════════════════════════
    # 图1: 轨迹总览
    # ═══════════════════════════════════════════════════
    fig1, ax = plt.subplots(figsize=(10, 10)); fig1.patch.set_facecolor(BG)
    ax.plot(cx, cy, color=RED, lw=1.2, label='Real', alpha=0.85)
    ax.plot(vx, vy, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.scatter(WPS[:,0], WPS[:,1], c=YELLOW, s=70, marker='x', zorder=5, linewidths=2.5, label='WP')
    for i,(wx,wy) in enumerate(WPS):
        ax.annotate('S' if i==0 else str(i), (wx,wy), textcoords='offset points',
                    xytext=(8,8), color=YELLOW, fontsize=9, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory', fontweight='bold', fontsize=12)
    ax.set_aspect('equal'); ax.legend(fontsize=8, loc='lower left'); ax.grid(True, alpha=0.3)
    out1 = Path(path).stem + '_traj.png'
    fig1.savefig(out1, dpi=150, facecolor=BG, edgecolor='none'); plt.close(fig1)
    print(f'Saved: {out1}')

    # ═══════════════════════════════════════════════════
    # 图2: 横向 (2行: 滑移 | ey+eth+eth_d+servo)
    # ═══════════════════════════════════════════════════
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8)); fig2.patch.set_facecolor(BG)

    # 上: 滑移 = car_yaw - gyro_z
    if has_yaw:
        slip = car_yaw - gyro_z
        ax1.plot(t, vst_yaw, color=BLUE, lw=1.0, label='vst_yaw (ref)')
        ax1.plot(t, car_yaw, color=RED, lw=1.0, ls='--', label='car_yaw (model)')
        ax1.plot(t, gyro_z, color=AQUA, lw=1.2, label='gyro_z (IMU)')
        ax1.legend(fontsize=7, ncol=3)
        ax1.set_ylabel('[rad/s]')
        ax1_t = ax1.twinx()
        ax1_t.plot(t, slip, color=ORANGE, lw=1.0, alpha=0.6, label='slip=car_yaw−gyro')
        ax1_t.legend(fontsize=7, loc='upper right')
        ax1_t.set_ylabel('slip [rad/s]', color=ORANGE)
        sym_ylim(ax1, np.concatenate([vst_yaw, car_yaw, gyro_z]))
        sym_ylim(ax1_t, slip)
        ax1.axhline(0, color=M, lw=0.8, ls='--')
    ax1.set_title('Yaw Rate & Slip', fontweight='bold'); ax1.grid(True, alpha=0.3)

    # 下: ey + eth + eth_d + servo 叠加
    if has_yaw:
        eth_d = vst_yaw - gyro_z
        ax2.plot(t, ey*100, color=ORANGE, lw=1.2, label='ey [cm]')
        ax2.plot(t, eth, color=AQUA, lw=1.2, label='eth [deg]')
        ax2.plot(t, eth_d, color=PURPLE, lw=1.2, label='eth_d [rad/s]')
        sym_ylim(ax2, np.concatenate([ey*100, eth, eth_d]))
        if has_ed:
            ed_delta = servo - vst_delta
            ax2_t = ax2.twinx()
            ax2_t.plot(t, ed_delta, color=YELLOW, lw=0.8, alpha=0.6, label='ed=carδ−vstδ [rad]')
            sym_ylim(ax2_t, ed_delta)
            ax2_t.set_ylabel('ed [rad]', color=YELLOW)
            if has_ff:
                ax2_t2 = ax2.twinx()
                ax2_t2.spines['right'].set_position(('outward', 50))
                ax2_t2.plot(t, w_ff, color=PURPLE, lw=0.8, alpha=0.5, ls=':', label='w_ff [rad/s]')
                sym_ylim(ax2_t2, w_ff)
                ax2_t2.set_ylabel('w_ff [rad/s]', color=PURPLE)
                h1,l1=ax2.get_legend_handles_labels()
                h2,l2=ax2_t.get_legend_handles_labels()
                h3,l3=ax2_t2.get_legend_handles_labels()
                ax2.legend(h1+h2+h3,l1+l2+l3,fontsize=6)
            else:
                h1,l1=ax2.get_legend_handles_labels(); h2,l2=ax2_t.get_legend_handles_labels()
                ax2.legend(h1+h2,l1+l2,fontsize=6)
    else:
        ax2.plot(t, ey*100, color=ORANGE, lw=1.2, label='ey [cm]')
        ax2.plot(t, eth, color=AQUA, lw=1.2, label='eth [deg]')
        ax2.legend(fontsize=7)
    ax2.axhline(0, color=M, lw=0.8, ls='--')
    ax2.set_xlabel('t [s]'); ax2.set_ylabel('ey/eth/eth_d')
    ax2.set_title('ey + eth + eth_d + Servo', fontweight='bold'); ax2.grid(True, alpha=0.3)

    out2 = Path(path).stem + '_lat.png'
    fig2.savefig(out2, dpi=150, facecolor=BG, edgecolor='none'); plt.close(fig2)
    print(f'Saved: {out2}')

    # ═══════════════════════════════════════════════════
    # 图3: 纵向 (速度 + Δv + f_hat + e_x)
    # ═══════════════════════════════════════════════════
    fig3, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8)); fig3.patch.set_facecolor(BG)

    if has_vf:
        ax1.plot(t, vst_v, color=BLUE, lw=1.2, label='vst_v (ref)')
        ax1.plot(t, car_v, color=RED, lw=1.2, label='car_v (meas)')
        ax1.legend(fontsize=7)
        ax1.set_ylabel('speed [m/s]')
    ax1.set_title('Speed: VST vs Car', fontweight='bold'); ax1.grid(True, alpha=0.3)

    if has_vf:
        dv = vst_v - car_v
        ax2.plot(t, ex*100, color=BLUE, lw=1.2, label='e_x [cm]')
        ax2.plot(t, dv, color=ORANGE, lw=1.2, label='Δv=vst−car [m/s]')
        sym_ylim(ax2, np.concatenate([ex*100, dv]))
        ax2_t = ax2.twinx()
        ax2_t.plot(t, f_hat, color=PURPLE, lw=1.0, alpha=0.7, label='f_hat [m/s²]')
        sym_ylim(ax2_t, f_hat)
        h1,l1=ax2.get_legend_handles_labels(); h2,l2=ax2_t.get_legend_handles_labels()
        ax2.legend(h1+h2,l1+l2,fontsize=6)
        ax2_t.set_ylabel('f_hat [m/s²]', color=PURPLE)
    ax2.axhline(0, color=M, lw=0.8, ls='--')
    ax2.set_xlabel('t [s]'); ax2.set_ylabel('e_x / Δv')
    ax2.set_title('e_x + Δv + f_hat', fontweight='bold'); ax2.grid(True, alpha=0.3)

    out3 = Path(path).stem + '_lon.png'
    fig3.savefig(out3, dpi=150, facecolor=BG, edgecolor='none'); plt.close(fig3)
    print(f'Saved: {out3}')

    # ── 统计 ──
    dist = np.sqrt(ex**2 + ey**2)
    print(f'  ey STD={np.std(ey)*100:.1f}cm max={np.max(np.abs(ey))*100:.1f}cm')
    print(f'  ex STD={np.std(ex)*100:.1f}cm max={np.max(np.abs(ex))*100:.1f}cm')
    print(f'  eth STD={np.std(eth):.1f}deg max={np.max(np.abs(eth)):.1f}deg')
    if has_yaw:
        eth_d = vst_yaw - gyro_z; slip = car_yaw - gyro_z
        print(f'  eth_d STD={np.std(eth_d):.2f}rad/s max={np.max(np.abs(eth_d)):.2f}rad/s')
        print(f'  slip  mean={np.mean(slip):.3f} max={np.max(np.abs(slip)):.3f}rad/s')
    if has_vf:
        print(f'  Δv   STD={np.std(vst_v-car_v):.2f}m/s')
        print(f'  f_hat [{np.min(f_hat):.2f},{np.max(f_hat):.2f}]')

if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else str(DATA)
    main(p)
