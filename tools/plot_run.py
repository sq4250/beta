"""
plot_run.py — 拖入 CSV 数据即可可视化

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
    if len(a) == 0: raise ValueError(f'No valid data in {path}')
    # 检测扩展列: t,vst_x,vst_y,car_x,car_y,vst_th,car_th,servo,omega_vst,omega_imu
    extra = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            p = line.split(',')
            if len(p) >= 10:
                try: extra = [float(x) for x in p[7:10]]; break
                except ValueError: continue
    has_extra = extra is not None
    if has_extra:
        raw2 = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): continue
                p = line.split(',')
                if len(p) >= 10:
                    try: raw2.append([float(x) for x in p[:10]])
                    except ValueError: continue
        b = np.array(raw2)
        t, vx, vy, cx, cy, vth, cth = b[:,0], b[:,1], b[:,2], b[:,3], b[:,4], b[:,5], b[:,6]
        servo, thd_model, thd_imu = b[:,7], b[:,8], b[:,9]
    else:
        t, cx, cy, cth, vx, vy, vth = a[:,0], a[:,1], a[:,2], a[:,3], a[:,4], a[:,5], a[:,6]
        servo = thd_model = thd_imu = None
    return t, cx, cy, cth, vx, vy, vth, servo, thd_model, thd_imu, has_extra

def main(path):
    t,cx,cy,cth,vx,vy,vth,servo,thd_model,thd_imu,has_extra = load(path)
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
    ax.scatter(WPS[:,0], WPS[:,1], c=YELLOW, s=70, marker='x', zorder=5, linewidths=2.5, label='Waypoint')
    for i,(wx,wy) in enumerate(WPS):
        ax.annotate('S' if i==0 else str(i), (wx,wy), textcoords='offset points',
                    xytext=(8,8), color=YELLOW, fontsize=8, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trajectory', fontweight='bold')
    ax.set_aspect('equal'); ax.legend(fontsize=7, loc='lower left'); ax.grid(True, alpha=0.3)

    # (0,1) 航向
    ax = axes[0,1]
    ax.plot(t, cth, color=RED, lw=1.2, label='Real')
    ax.plot(t, vth, color=BLUE, lw=0.8, alpha=0.5, ls='--', label='Virtual')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Heading [deg]')
    ax.set_title('Heading', fontweight='bold'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (0,2) 横向误差 + 航向误差 + 前轮转角 叠加
    ax = axes[0,2]
    ey = (cx - vx) * np.sin(np.deg2rad(vth)) - (cy - vy) * np.cos(np.deg2rad(vth))
    eth = (vth - cth + 180) % 360 - 180
    ax.plot(t, ey*100, color=ORANGE, lw=1.2, label='ey[cm]')
    ax.plot(t, eth, color=AQUA, lw=1.2, label='eth[deg]')
    if has_extra:
        ax2 = ax.twinx()
        ax2.plot(t, servo, color=YELLOW, lw=0.8, alpha=0.6, label='servo[rad]')
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1+h2, l1+l2, fontsize=6)
        ax2.set_ylabel('servo [rad]', color=YELLOW)
        # y=0 对齐: 左右轴都对称
        ey_r = max(abs(np.min(ey*100)), abs(np.max(ey*100)), 0.1) * 1.1
        eth_r = max(abs(np.min(eth)), abs(np.max(eth)), 0.1) * 1.1
        left_r = max(ey_r, eth_r)
        ax.set_ylim(-left_r, left_r)
        sr = max(abs(np.min(servo)), abs(np.max(servo)), 0.1) * 1.1
        ax2.set_ylim(-sr, sr)
    ax.axhline(0, color=M, lw=0.8, ls='--')
    ax.set_xlabel('t [s]'); ax.set_ylabel('ey/eth', color=AQUA)
    ax.set_title('ey + eth + Servo', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,0) 位置偏差
    ax = axes[1,0]
    pos_err = np.sqrt((cx-vx)**2 + (cy-vy)**2)
    ax.plot(t, pos_err, color=ORANGE, lw=1.5)
    ax.set_xlabel('t [s]'); ax.set_ylabel('pos_err [m]')
    ax.set_title('Position Error', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,1) 角速度: omega_vst vs omega_imu
    ax = axes[1,1]
    if has_extra:
        ax.plot(t, thd_model, color=BLUE, lw=1.0, ls='--', label='thd_model')
        ax.plot(t, thd_imu, color=RED, lw=1.2, label='thd_imu')
        ax.legend(fontsize=7)
    ax.set_xlabel('t [s]'); ax.set_ylabel('thd [rad/s]')
    ax.set_title('Yaw Rate: Model vs IMU', fontweight='bold'); ax.grid(True, alpha=0.3)

    # (1,2) 角速度误差
    ax = axes[1,2]
    if has_extra:
        thd_err = thd_model - thd_imu
        ax.plot(t, thd_err, color=PURPLE, lw=1.2)
    ax.axhline(0, color=M, lw=0.8, ls='--')
    ax.set_xlabel('t [s]'); ax.set_ylabel('thd_err [rad/s]')
    ax.set_title('Yaw Rate Error (eth_d)', fontweight='bold'); ax.grid(True, alpha=0.3)

    fig.suptitle(f'Run: {Path(path).name}  |  {len(t)}pts  {t[-1]:.1f}s',
                 fontsize=11, fontweight='bold', color=P, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.94])

    out = Path(path).with_suffix('.png')
    fig.savefig(str(out), dpi=150, facecolor=BG, edgecolor='none'); plt.close()
    print(f'Saved: {out}')

    # ── 跟踪精度 ──
    vth_rad = np.deg2rad(vth)
    ex = (vx - cx) * np.cos(vth_rad) + (vy - cy) * np.sin(vth_rad)
    ey = (cx - vx) * np.sin(vth_rad) - (cy - vy) * np.cos(vth_rad)
    eth_arr = (vth - cth + 180) % 360 - 180
    dist = np.sqrt(ex**2 + ey**2)
    print(f'  横向误差 STD:  {np.std(ey)*100:.1f} cm  (max={np.max(np.abs(ey))*100:.1f} cm)')
    print(f'  纵向误差 STD:  {np.std(ex)*100:.1f} cm  (max={np.max(np.abs(ex))*100:.1f} cm)')
    print(f'  跟踪距离 STD:  {np.std(dist)*100:.1f} cm  (max={np.max(dist)*100:.1f} cm)')
    print(f'  航向误差 STD:  {np.std(eth_arr):.1f} deg  (max={np.max(np.abs(eth_arr)):.1f} deg)')

if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else str(DATA)
    main(p)
