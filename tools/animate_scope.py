"""
animate_scope.py — 小车 + 示波器动画 (MP4, 1920×1080 深色主题)

俯视图小车 (车身+前轮转角+VST 幽灵+3s 轨迹尾迹+航点状态)
+ 4 条示波器滚动面板: v / f̂ / ey / eth

用法: python tools/animate_scope.py [csv] [--fps 30] [--speed 1] [--window 4] [--out x.mp4]
      python tools/animate_scope.py sim_run.csv --speed 2          # 2 倍速
      python tools/animate_scope.py run_data.csv --preview 5       # 只存 t=5s 单帧 PNG

CSV: 车端/sim 12 列格式 (同 plot_run.py), 第 13 列 servo_delta[rad] 可选
(仿真 CSV 有 → 前轮转角真实画出; 实车 CSV 无 → 前轮按车身航向)
"""
import subprocess
import sys
import tempfile
import os
from pathlib import Path

import numpy as np

PROJ = Path(__file__).resolve().parents[1]
DATA = PROJ / 'tools' / 'sim_run.csv'

WPS = np.array([[0, 0], [1.715, 0.815], [3.445, 1.43], [4.565, 0.095],
                [2.965, -0.075], [3.70, -1.48], [1.91, -1.09]])

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

BG = '#1a1a19'; SURF = '#22221f'; P = '#ffffff'; S = '#c3c2b7'; M = '#898781'; G = '#2c2c2a'
RED = '#e66767'; BLUE = '#3987e5'; YELLOW = '#c98500'; AQUA = '#199e70'
ORANGE = '#d95926'; PURPLE = '#a855f7'

# 示波器通道 (固定量程, 稳定画面)
CH_V, CH_FHAT, CH_EY, CH_ETH = 'v', 'f_hat', 'ey', 'eth'
LIMITS = {CH_V: (0.0, 5.2), CH_FHAT: (-1.6, 0.2), CH_EY: (-8.0, 8.0), CH_ETH: (-10.0, 10.0)}
UNITS = {CH_V: '[m/s]', CH_FHAT: '[m/s²]', CH_EY: '[cm]', CH_ETH: '[deg]'}
STRIPS = [(CH_V, [('v_car', 'v_car', RED, '-'), ('v_vst', 'v_vst', BLUE, '--')]),
          (CH_FHAT, [('f_hat', 'f_hat', PURPLE, '-')]),
          (CH_EY, [('ey', None, ORANGE, '-')]),
          (CH_ETH, [('eth', None, AQUA, '-')])]

# 车体绘制缩放 (真实 0.15m 太小, 放大 2× 便于观看)
CAR_SCALE = 2.0
CAR_L, CAR_W = 0.15 * CAR_SCALE, 0.15 * CAR_SCALE
WHEEL_L = 0.055 * CAR_SCALE


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
    if len(b) == 0:
        raise ValueError(f'No valid data in {path}')
    d = dict(t=b[:, 0], vx=b[:, 1], vy=b[:, 2], cx=b[:, 3], cy=b[:, 4],
             vth=b[:, 5], cth=b[:, 6])
    if ncols >= 9:
        d['delta_fb'] = b[:, 8]
    if ncols >= 12:
        d['v_car'], d['v_vst'], d['f_hat'] = b[:, 9], b[:, 10], b[:, 11]
    d['servo'] = b[:, 12] if ncols >= 13 else None
    return d, ncols


def errs(d):
    vth = np.deg2rad(d['vth']); dx = d['vx'] - d['cx']; dy = d['vy'] - d['cy']
    ey = -dx * np.sin(vth) + dy * np.cos(vth)
    eth = (d['vth'] - d['cth'] + 180) % 360 - 180
    return ey * 100, eth


def car_artists(ax, color, lw=1.2, alpha=1.0):
    """返回 (body Polygon, 4 轮 Line2D, nose Line2D), 每帧 set 数据."""
    body = Polygon(np.zeros((4, 2)), closed=True, facecolor=color, alpha=alpha * 0.55,
                   edgecolor=color, linewidth=lw, zorder=6)
    wheels = [ax.plot([], [], color=color, lw=2.2, alpha=alpha, solid_capstyle='round',
                      zorder=7)[0] for _ in range(4)]
    nose = ax.plot([], [], color=color, lw=lw, alpha=alpha, zorder=7)[0]
    ax.add_patch(body)
    return body, wheels, nose


def set_car(body, wheels, nose, x, y, th_deg, steer_deg=None):
    th = np.deg2rad(th_deg)
    sd = np.deg2rad(steer_deg) if steer_deg is not None else 0.0
    c, s = np.cos(th), np.sin(th)
    R = np.array([[c, -s], [s, c]])
    corners = np.array([[-CAR_L / 2, -CAR_W / 2], [CAR_L / 2, -CAR_W / 2],
                        [CAR_L / 2, CAR_W / 2], [-CAR_L / 2, CAR_W / 2]])
    body.set_xy(corners @ R.T + np.array([x, y]))
    # 前轮 (θ+δ) ×2, 后轮 (θ) ×2, 前轴在 +0.75·L/2
    fx = 0.75 * CAR_L / 2
    for k, (sgn, ang) in enumerate([(1, th + sd), (-1, th + sd), (1, th), (-1, th)]):
        wx, wy = (fx if k < 2 else -fx), sgn * CAR_W / 2
        ca, sa = np.cos(ang), np.sin(ang)
        wheels[k].set_data([wx - ca * WHEEL_L / 2, wx + ca * WHEEL_L / 2],
                           [wy - sa * WHEEL_L / 2, wy + sa * WHEEL_L / 2])
    nose.set_data([x, x + c * CAR_L * 0.9], [y, y + s * CAR_L * 0.9])


def main():
    args = dict(fps=30, speed=1.0, window=4.0, out=None, preview=None)
    path = str(DATA)
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        a = argv[i]
        key, _, val = a.partition('=')
        if key == '--fps':
            args['fps'] = int(val) if val else int(argv[i + 1]); i += not bool(val)
        elif key == '--speed':
            args['speed'] = float(val) if val else float(argv[i + 1]); i += not bool(val)
        elif key == '--window':
            args['window'] = float(val) if val else float(argv[i + 1]); i += not bool(val)
        elif key == '--out':
            args['out'] = val if val else argv[i + 1]; i += not bool(val)
        elif key == '--preview':
            args['preview'] = float(val) if val else float(argv[i + 1]); i += not bool(val)
        elif not a.startswith('--'):
            path = a
        i += 1

    d, ncols = load(path)
    t = d['t']; n = len(t)
    ey, eth = errs(d)
    has_servo = d['servo'] is not None

    dur = t[-1] - t[0]
    video_dur = dur / args['speed']
    total_frames = max(int(video_dur * args['fps']), 1)
    frame_times = np.linspace(t[0], t[-1], total_frames)
    out = Path(args['out']) if args['out'] else Path(path).with_name(Path(path).stem + '_scope.mp4')
    print(f'{n} pts {ncols} cols, t=[{t[0]:.2f},{t[-1]:.2f}]s, {args["fps"]}fps '
          f'{args["speed"]}x → {total_frames} frames, servo={has_servo}')

    # ── 航点通过时间 (car 距 WP < 0.15m 首次) ──
    wp_pass = np.full(len(WPS), np.inf)
    for j, (wx, wy) in enumerate(WPS):
        dd = np.hypot(d['cx'] - wx, d['cy'] - wy)
        hit = np.where(dd < 0.15)[0]
        if len(hit):
            wp_pass[j] = t[hit[0]]

    # ── 场地边界 ──
    all_x = np.concatenate([d['cx'], d['vx'], WPS[:, 0]])
    all_y = np.concatenate([d['cy'], d['vy'], WPS[:, 1]])
    x_mid, y_mid = (all_x.min() + all_x.max()) / 2, (all_y.min() + all_y.max()) / 2
    span = max(all_x.max() - all_x.min(), all_y.max() - all_y.min(), 0.5) * 0.62

    plt.rcParams.update({'figure.facecolor': BG, 'axes.facecolor': SURF,
                         'axes.edgecolor': G, 'axes.labelcolor': S, 'text.color': P,
                         'xtick.color': M, 'ytick.color': M, 'grid.color': G,
                         'grid.alpha': 0.25, 'font.size': 9, 'legend.facecolor': SURF,
                         'legend.edgecolor': G, 'legend.labelcolor': S})

    fig = plt.figure(figsize=(19.2, 10.8))  # ×100dpi = 1920×1080
    fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.18, 1], wspace=0.06,
                          left=0.015, right=0.985, top=0.925, bottom=0.045)

    # ═══ 左: 俯视场地 ═══
    ax_f = fig.add_subplot(gs[0, 0])
    ax_f.set_xlim(x_mid - span, x_mid + span)
    ax_f.set_ylim(y_mid - span, y_mid + span)
    ax_f.set_aspect('equal')
    trail_c, = ax_f.plot([], [], color=RED, lw=1.6, alpha=0.85, label='Real', zorder=3)
    trail_v, = ax_f.plot([], [], color=BLUE, lw=1.1, alpha=0.5, ls='--', label='Virtual', zorder=3)
    for j, (wx, wy) in enumerate(WPS):
        ax_f.scatter([wx], [wy], c=YELLOW, s=75, marker='x', zorder=4, linewidths=2)
    wp_lbl = [ax_f.annotate('S' if j == 0 else str(j), (wx, wy),
                            textcoords='offset points', xytext=(7, 7),
                            color=YELLOW, fontsize=10, fontweight='bold', zorder=4)
              for j, (wx, wy) in enumerate(WPS)]
    body_v, wh_v, nose_v = car_artists(ax_f, BLUE, lw=1.0, alpha=0.55)
    body_c, wh_c, nose_c = car_artists(ax_f, RED, lw=1.5, alpha=1.0)
    ax_f.legend(fontsize=8, loc='upper left')
    ax_f.grid(True, alpha=0.2)
    title_f = ax_f.set_title('', fontweight='bold', color=P, loc='left')

    # ═══ 右: 示波器 4 条 ═══
    axes_s = [fig.add_subplot(gs[0, 1].subgridspec(4, 1)[i]) for i in range(4)]
    traces = {}
    for i, (ch, series) in enumerate(STRIPS):
        ax = axes_s[i]
        lo, hi = LIMITS[ch]
        ax.set_xlim(t[-1] - args['window'] - 0.2, t[-1] + 0.2)
        ax.set_ylim(lo, hi)
        ax.axhline(0, color=G, lw=1.0, alpha=0.8)
        ax.grid(axis='y', alpha=0.15)
        for name, col, color, ls in series:
            traces[(ch, name)] = ax.plot([], [], color=color, lw=1.3, ls=ls)[0]
        ax.set_ylabel(f'{ch} {UNITS[ch]}', color=S, fontsize=8.5, labelpad=2)
        ax.tick_params(labelsize=8)
        ax.set_xticks([]) if i < 3 else ax.set_xlabel('t [s]', color=S, fontsize=8.5)
        for spine in ['top', 'right']:
            ax.spines[spine].set_visible(False)
        ax.spines['left'].set_color(G); ax.spines['bottom'].set_color(G)
    # 扫描线 + 通道末值标签
    sweep = [ax.axvline(t[-1], color=P, lw=0.8, alpha=0.35) for ax in axes_s]
    tgl = [ax.text(0.995, 0.93, '', transform=ax.transAxes, ha='right', va='top',
                   color=P, fontsize=8, family='monospace') for ax in axes_s]
    fig.text(0.57, 0.955, 'OSCILLOSCOPE', color=S, fontsize=11, fontweight='bold')
    fig.suptitle('', fontsize=13, fontweight='bold', color=P, x=0.36, y=0.975)

    def render(fi, ft, save=True):
        i = int(np.searchsorted(t, ft))
        if i >= n:
            i = n - 1
        tnow = ft
        # 帧时间线性插值 (实车 10Hz 采样 → 平滑车体/示波器)
        ic = lambda k: np.interp(ft, t, d[k])
        cx_f, cy_f, cth_f = ic('cx'), ic('cy'), ic('cth')
        vx_f, vy_f, vth_f = ic('vx'), ic('vy'), ic('vth')
        # ── 场地 ──
        tail = int(np.searchsorted(t, max(tnow - 3.0, t[0])))
        trail_c.set_data(d['cx'][tail:i + 1], d['cy'][tail:i + 1])
        trail_v.set_data(d['vx'][tail:i + 1], d['vy'][tail:i + 1])
        set_car(body_c, wh_c, nose_c, cx_f, cy_f, cth_f,
                ic('servo') if has_servo else None)
        set_car(body_v, wh_v, nose_v, vx_f, vy_f, vth_f)
        for j, lbl in enumerate(wp_lbl):
            done = tnow >= wp_pass[j]
            lbl.set_color(M if done else YELLOW)
        # ── 示波器 ──
        w0 = tnow - args['window']
        sel = np.where(t >= w0)[0]
        if len(sel) < 2:
            sel = np.where(t >= t[0])[0][:2]
        sl = sel[0]
        for ch, series in STRIPS:
            for name, col, color, ls in series:
                y = None
                if col is None:
                    y = (ey if name == 'ey' else eth)
                elif col in d:
                    y = d[col]
                if y is None:
                    continue
                traces[(ch, name)].set_data(t[sl:i + 1], y[sl:i + 1])
        for ax in sweep:
            ax.set_xdata([tnow, tnow])
        for j, (ch, _) in enumerate(STRIPS):
            row = [f'{name}={ic(col):5.2f}' if col is not None and col in d else
                   f'{name}={np.interp(ft, t, ey if name == "ey" else eth):5.1f}'
                   for name, col, _, _ in STRIPS[j][1]]
            tgl[j].set_text('  '.join(row))
            axes_s[j].set_xlim(tnow - args['window'], tnow)
        fig.suptitle(f't={tnow:5.2f}s   v={ic("v_car"):.2f} m/s   '
                     f'f̂={ic("f_hat"):+.2f}', fontsize=13, fontweight='bold', color=P, x=0.36, y=0.975)
        if save:
            fig.savefig(f'{tmpdir}/frame_{fi:06d}.png', dpi=100, facecolor=BG)
            if (fi + 1) % 60 == 0:
                print(f'  frame {fi + 1}/{total_frames}: t={tnow:.1f}s')

    if args['preview'] is not None:
        render(0, args['preview'], save=False)
        png = out.with_suffix('.png')
        fig.savefig(str(png), dpi=100, facecolor=BG)
        plt.close(fig)
        print(f'Preview: {png}')
        return

    tmpdir = tempfile.mkdtemp()
    for fi, ft in enumerate(frame_times):
        render(fi, ft)

    print(f'  {total_frames} frames, encoding...')
    cmd = (f'ffmpeg -y -framerate {args["fps"]} -i "{tmpdir}/frame_%06d.png" '
           f'-vf "pad=ceil(iw/2)*2:ceil(ih/2)*2:0:0:color=#1a1a19" '
           f'-c:v libx264 -pix_fmt yuv420p -crf 23 "{out}"')
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if r.returncode != 0:
        print(f'  ffmpeg error:\n{r.stderr}')
        raise RuntimeError(f'ffmpeg exited with {r.returncode}')
    for k in range(total_frames):
        os.unlink(f'{tmpdir}/frame_{k:06d}.png')
    os.rmdir(tmpdir)
    plt.close(fig)
    print(f'Saved: {out} ({video_dur:.1f}s @ {args["fps"]}fps)')


if __name__ == '__main__':
    main()
