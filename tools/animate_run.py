"""
animate_run.py — CSV 轨迹动画 → MP4 视频

用法: python tools/animate_run.py [csv_file] [--fps 30] [--speed 2]

输出: tools/run_anim.mp4
"""
import sys, subprocess, tempfile, os, numpy as np
from pathlib import Path

PROJ = Path(__file__).resolve().parents[1]
DATA = PROJ / 'tools' / 'run_data.csv'
OUT  = PROJ / 'tools' / 'run_anim.mp4'

WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

BG='#1a1a19'; SURF='#22221f'; RED='#e66767'; BLUE='#3987e5'; YELLOW='#c98500'

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
    return a[:,0], a[:,3], a[:,4], a[:,1], a[:,2], a[:,6], a[:,5]  # t,cx,cy,vx,vy,cth,vth

def main():
    args = {'fps': 30, 'speed': 1.0}
    path = str(DATA)
    for a in sys.argv[1:]:
        if a.startswith('--fps='): args['fps'] = int(a.split('=')[1])
        elif a.startswith('--speed='): args['speed'] = float(a.split('=')[1])
        elif not a.startswith('[') and not a.startswith('--'):
            path = a
    t, cx, cy, vx, vy, cth, vth = load(path)
    n = len(t)
    dur = t[-1] - t[0]
    video_dur = dur / args['speed']
    total_frames = int(video_dur * args['fps'])
    frame_times = np.linspace(t[0], t[-1], total_frames)
    print(f'{n} pts, t=[{t[0]:.2f},{t[-1]:.2f}]s dur={dur:.1f}s, {args["fps"]}fps {args["speed"]}x → {total_frames} frames ({video_dur:.1f}s)')

    import matplotlib.patches as mpatches
    plt.rcParams.update({'figure.facecolor':BG, 'font.size':10, 'text.color':'#ffffff'})

    tmpdir = tempfile.mkdtemp()
    frame_no = 0

    # 轨迹边界
    all_x = np.concatenate([cx, vx, WPS[:,0]])
    all_y = np.concatenate([cy, vy, WPS[:,1]])
    x_mid = (all_x.min() + all_x.max()) / 2
    y_mid = (all_y.min() + all_y.max()) / 2
    span = max(all_x.max()-all_x.min(), all_y.max()-all_y.min(), 0.5) * 0.6

    for fi, ft in enumerate(frame_times):
        i = np.searchsorted(t, ft)  # 最近数据点
        if i >= n: i = n - 1
        fig, ax = plt.subplots(figsize=(10, 10))
        fig.patch.set_facecolor(BG); ax.set_facecolor(SURF)

        # 历史轨迹
        tail = max(0, i - args['fps'] * 3)
        ax.plot(cx[tail:i+1], cy[tail:i+1], color=RED, lw=1.5, alpha=0.8, label='Real')
        ax.plot(vx[tail:i+1], vy[tail:i+1], color=BLUE, lw=1.0, alpha=0.5, ls='--', label='Virtual')

        # 当前位置
        ax.scatter(cx[i], cy[i], c=RED, s=120, zorder=5, edgecolors='white', linewidth=1)
        ax.scatter(vx[i], vy[i], c=BLUE, s=80, zorder=5, marker='s', edgecolors='white', linewidth=1)

        # 车头方向
        L = 0.15
        for px, py, th, c in [(cx[i], cy[i], cth[i], RED), (vx[i], vy[i], vth[i], BLUE)]:
            trad = np.deg2rad(th)
            dx, dy = L * np.cos(trad), L * np.sin(trad)
            ax.annotate('', xy=(px+dx, py+dy), xytext=(px, py),
                        arrowprops=dict(arrowstyle='->', color=c, lw=2))

        # 航点
        ax.scatter(WPS[:,0], WPS[:,1], c=YELLOW, s=80, marker='x', zorder=4, linewidths=2)
        for j,(wx,wy) in enumerate(WPS):
            ax.annotate('S' if j==0 else str(j), (wx,wy), textcoords='offset points',
                        xytext=(6,6), color=YELLOW, fontsize=9, fontweight='bold')

        ax.set_xlim(x_mid-span, x_mid+span)
        ax.set_ylim(y_mid-span, y_mid+span)
        ax.set_aspect('equal')
        ax.set_title(f't={t[i]:.2f}s  |  car=({cx[i]:.2f},{cy[i]:.2f})  vst=({vx[i]:.2f},{vy[i]:.2f})',
                     color='#ffffff', fontweight='bold')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.2)
        ax.tick_params(colors='#898781')

        fname = f'{tmpdir}/frame_{frame_no:06d}.png'
        fig.savefig(fname, dpi=100, facecolor=BG, bbox_inches='tight', pad_inches=0.1)
        plt.close(fig)
        frame_no += 1
        if (fi+1) % 30 == 0:
            print(f'  frame {fi+1}/{total_frames}: t={t[i]:.1f}s')

    print(f'  {frame_no} frames, encoding...')
    cmd = (f'ffmpeg -y -framerate {args["fps"]} '
           f'-i "{tmpdir}/frame_%06d.png" '
           f'-vf "pad=ceil(iw/2)*2:ceil(ih/2)*2:0:0:color=#1a1a19" '
           f'-c:v libx264 -pix_fmt yuv420p -crf 23 '
           f'"{OUT}"')
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f'  ffmpeg error:\n{result.stderr}')
        raise RuntimeError(f'ffmpeg exited with {result.returncode}')
    # cleanup
    for i in range(frame_no):
        os.unlink(f'{tmpdir}/frame_{i:06d}.png')
    os.rmdir(tmpdir)
    print(f'Saved: {OUT}')

if __name__ == '__main__':
    main()
