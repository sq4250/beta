"""
plot_tracking.py — 实车数据可视化: 轨迹 + 误差 + GIF 动画
用法: python tools/plot_tracking.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ===== 数据 (CSV 格式) =====
raw = """0.013,-0.000,-0.000,0.0000,0.000,0.0000,0.0004,-0.1206,0.000,-0.000,-0.0002
0.020,-0.000,0.000,0.0001,0.017,0.0000,0.0010,-0.1771,0.001,-0.000,-0.0012
0.844,1.212,-0.424,-0.1003,2.382,0.0757,0.0106,0.1700,1.215,-0.348,-0.0296
0.944,1.445,-0.424,0.1078,2.225,0.0898,0.0193,0.1916,1.441,-0.333,0.1579
1.044,1.658,-0.373,0.3652,2.131,0.0937,0.0323,0.2146,1.646,-0.277,0.3728
1.144,1.838,-0.276,0.6349,1.971,0.0879,0.0508,0.2620,1.819,-0.180,0.6433
1.244,1.978,-0.137,0.9332,1.939,0.0756,0.0689,0.2859,1.952,-0.043,0.9480
1.345,2.073,0.056,1.2781,2.315,0.0523,0.0660,-0.0280,2.059,0.139,1.0663
1.444,2.119,0.283,1.4051,2.395,-0.0132,0.0709,-0.0781,2.161,0.330,1.1042
1.544,2.168,0.535,1.3470,2.785,-0.0765,0.0673,-0.0189,2.261,0.562,1.2228
1.644,2.237,0.804,1.2986,2.773,-0.0920,0.0408,0.0503,2.333,0.814,1.3576
1.744,2.312,1.070,1.3095,2.664,-0.0587,0.0137,0.1386,2.369,1.069,1.5010
1.844,2.365,1.321,1.4392,2.329,-0.0015,0.0010,0.2122,2.365,1.313,1.6676
1.944,2.369,1.551,1.6695,2.249,0.0482,0.0082,0.2462,2.322,1.537,1.8552
2.044,2.319,1.758,1.9635,2.068,0.0811,0.0234,0.2926,2.240,1.731,2.0847
2.144,2.216,1.930,2.2642,2.000,0.0978,0.0471,0.3221,2.119,1.886,2.3719
2.246,2.061,2.055,2.6584,1.952,0.0961,0.0745,0.2631,1.954,2.001,2.6835
2.344,1.862,2.116,2.9944,2.340,0.0705,0.0721,-0.0941,1.776,2.081,2.7040
2.444,1.621,2.140,3.0349,2.483,-0.0106,0.0765,-0.0909,1.566,2.181,2.7151
2.544,1.361,2.178,2.9600,2.866,-0.0786,0.0715,-0.0334,1.329,2.273,2.8280
2.644,1.095,2.240,2.8737,2.807,-0.0916,0.0441,0.0630,1.078,2.335,2.9640
2.744,0.836,2.311,2.8897,2.621,-0.0503,0.0171,0.1686,0.830,2.359,3.1200
2.844,0.593,2.358,3.0387,2.363,0.0123,0.0084,0.2339,0.596,2.343,3.2947
2.944,0.372,2.355,3.2888,2.138,0.0648,0.0196,0.2868,0.384,2.288,3.4968
3.044,0.173,2.297,3.5673,1.986,0.1003,0.0412,0.3526,0.205,2.194,3.7546
3.144,0.008,2.186,3.8901,2.045,0.1221,0.0732,0.3732,0.066,2.060,4.0573
3.246,-0.117,2.023,4.2240,2.203,0.1317,0.1053,0.2786,-0.030,1.881,4.3530
3.344,-0.186,1.809,4.5711,2.491,0.1246,0.1156,0.1731,-0.081,1.683,4.5390
3.444,-0.179,1.553,4.8781,2.459,0.0808,0.1141,0.0292,-0.104,1.445,4.6785
3.544,-0.112,1.291,5.0124,2.836,0.0094,0.1105,-0.0772,-0.101,1.192,4.7493
3.644,-0.033,1.033,4.9834,2.735,-0.0590,0.1038,-0.1059,-0.088,0.936,4.7730
3.744,0.024,0.766,4.8537,2.672,-0.1003,0.0794,-0.0832,-0.072,0.690,4.7834
3.844,0.042,0.509,4.7174,2.323,-0.1014,0.0510,0.0003,-0.054,0.460,4.7948
3.944,0.032,0.280,4.6328,2.164,-0.0696,0.0303,0.0788,-0.033,0.250,4.8331
4.044,0.017,0.080,4.6665,1.842,-0.0306,0.0222,-0.0258,-0.011,0.062,4.8035
4.145,0.013,-0.096,4.7061,1.403,-0.0152,0.0142,-0.0029,-0.001,-0.111,4.7430
4.244,0.013,-0.222,4.7122,1.290,-0.0131,0.0157,-0.0300,0.000,-0.232,4.7105"""

lines = [l.strip() for l in raw.strip().split("\n")]
data = np.array([[float(x) for x in l.split(",")] for l in lines])
t = data[:, 0]
cx, cy, cth, cv = data[:, 1], data[:, 2], data[:, 3], data[:, 4]
ey, ex, delta = data[:, 5], data[:, 6], data[:, 7]
vx, vy, vth = data[:, 8], data[:, 9], data[:, 10]

WP = np.array([[2, 0], [2, 2], [0, 2], [0, 0]])

# Compute per-WP min distances
print("=== WP Min Distances (car) ===")
for i, wp in enumerate(WP):
    dists = np.sqrt((cx - wp[0])**2 + (cy - wp[1])**2)
    idx = np.argmin(dists)
    print(f"WP{i} ({wp[0]:.0f},{wp[1]:.0f}): min_d={dists[idx]*100:.1f}cm  @t={t[idx]:.3f}s  pos=({cx[idx]:.3f},{cy[idx]:.3f})")

# Compute vst min distances
print("\n=== WP Min Distances (vst) ===")
for i, wp in enumerate(WP):
    dists = np.sqrt((vx - wp[0])**2 + (vy - wp[1])**2)
    idx = np.argmin(dists)
    print(f"WP{i} ({wp[0]:.0f},{wp[1]:.0f}): min_d={dists[idx]*100:.1f}cm  @t={t[idx]:.3f}s  pos=({vx[idx]:.3f},{vy[idx]:.3f})")

# ============================================================
# Figure 1: Full dashboard
# ============================================================
fig, axes = plt.subplots(2, 3, figsize=(16, 10))
fig.suptitle("Beta-Ackerman Real-Car Test | 200Hz LQR+LADRC + 20Hz NN", fontsize=13, fontweight="bold")

# (0,0) Trajectory
ax = axes[0, 0]
ax.plot(WP[:, 0], WP[:, 1], "s-", color="gray", ms=12, alpha=0.4, label="waypoints")
for i, (wx, wy) in enumerate(WP):
    ax.annotate(f"WP{i}", (wx, wy), textcoords="offset points", xytext=(8, 8), fontsize=9, color="gray")
ax.plot(vx, vy, "b-", lw=1.2, alpha=0.5, label="virtual (vst)")
ax.plot(cx, cy, "r-", lw=2.0, alpha=0.85, label="real (car)")
for i, wp in enumerate(WP):
    dists = np.sqrt((cx - wp[0])**2 + (cy - wp[1])**2)
    idx = np.argmin(dists)
    ax.plot(cx[idx], cy[idx], "rx", ms=8, mew=2)
    ax.plot([cx[idx], wp[0]], [cy[idx], wp[1]], "r:", lw=0.8, alpha=0.6)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
ax.set_title("Trajectory (CCW square)")
ax.set_aspect("equal"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
ax.set_xlim(-0.5, 2.8); ax.set_ylim(-0.6, 2.6)

# (0,1) ey
ax = axes[0, 1]
ax.fill_between(t, 0, ey, alpha=0.12, color="red")
ax.plot(t, ey, "r-", lw=1.5)
ax.axhline(0, color="gray", lw=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel("ey [m]")
ax.set_title("Lateral Error ey  (+ = car outside ref, - = inside)")
ax.grid(True, alpha=0.3)

# (0,2) ex
ax = axes[0, 2]
ax.fill_between(t, 0, ex, alpha=0.12, color="green")
ax.plot(t, ex, "g-", lw=1.5)
ax.axhline(0, color="gray", lw=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel("ex [m]")
ax.set_title("Longitudinal Error ex")
ax.grid(True, alpha=0.3)

# (1,0) Speed
ax = axes[1, 0]
ax.plot(t, cv, "r-", lw=1.5, label="car.v")
ax.axhline(3.5, color="r", ls="--", alpha=0.3, label="V_MAX")
ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]")
ax.set_title("Speed")
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

# (1,1) delta
ax = axes[1, 1]
ax.plot(t, np.rad2deg(delta), "purple", lw=1.5)
ax.axhline(np.rad2deg(0.4636), color="r", ls="--", alpha=0.4, label=r"$\pm$26.6$^\circ$")
ax.axhline(-np.rad2deg(0.4636), color="r", ls="--", alpha=0.4)
ax.set_xlabel("t [s]"); ax.set_ylabel("delta [deg]")
ax.set_title("Steering Command")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# (1,2) theta comparison
ax = axes[1, 2]
ax.plot(t, cth, "r-", lw=1.5, label="car.theta")
ax.plot(t, vth, "b-", lw=1.0, alpha=0.6, label="vst.theta")
ax.set_xlabel("t [s]"); ax.set_ylabel("theta [rad]")
ax.set_title("Heading: car vs vst")
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig("tools/tracking_dashboard.png", dpi=150, bbox_inches="tight")
print("\nSaved: tools/tracking_dashboard.png")
plt.close()

# ============================================================
# Figure 2: Trajectory detail with headings
# ============================================================
fig2, ax2 = plt.subplots(figsize=(12, 8))
ax2.plot(WP[:, 0], WP[:, 1], "s-", color="gray", ms=15, alpha=0.3, lw=2, label="waypoints")
for i, (wx, wy) in enumerate(WP):
    ax2.annotate(f"WP{i} ({wx:.0f},{wy:.0f})", (wx, wy),
                 textcoords="offset points", xytext=(10, 10),
                 fontsize=10, color="gray", fontweight="bold")
ax2.plot(vx, vy, "b-", lw=1.5, alpha=0.5, label="virtual (vst)")
ax2.plot(cx, cy, "r-", lw=2.5, alpha=0.9, label="real (car)")

# Closest points + min distance labels
for i, wp in enumerate(WP):
    dists = np.sqrt((cx - wp[0])**2 + (cy - wp[1])**2)
    idx = np.argmin(dists)
    min_d = dists[idx] * 100  # cm
    ax2.plot(cx[idx], cy[idx], "o", ms=12, mew=2,
             markerfacecolor="yellow", markeredgecolor="red")
    ax2.plot([cx[idx], wp[0]], [cy[idx], wp[1]], "r--", lw=0.8, alpha=0.5)
    mid_x, mid_y = (cx[idx] + wp[0]) / 2, (cy[idx] + wp[1]) / 2
    ax2.annotate(f"{min_d:.1f}cm", (mid_x, mid_y), fontsize=10, color="red", ha="center",
                 bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.85))

# Heading arrows every 300ms
step = 3
for i in range(0, len(t), step):
    dx_c, dy_c = 0.04 * np.cos(cth[i]), 0.04 * np.sin(cth[i])
    ax2.arrow(cx[i], cy[i], dx_c, dy_c, head_width=0.02, head_length=0.02,
              fc="red", ec="red", alpha=0.35)
    dx_v, dy_v = 0.04 * np.cos(vth[i]), 0.04 * np.sin(vth[i])
    ax2.arrow(vx[i], vy[i], dx_v, dy_v, head_width=0.02, head_length=0.02,
              fc="blue", ec="blue", alpha=0.25)

ax2.set_xlabel("X [m]", fontsize=12); ax2.set_ylabel("Y [m]", fontsize=12)
ax2.set_title("Trajectory Detail — car vs virtual vs waypoints\n(heading arrows every 300ms, min distances in cm)",
              fontsize=12, fontweight="bold")
ax2.set_aspect("equal"); ax2.legend(fontsize=10); ax2.grid(True, alpha=0.3)
ax2.set_xlim(-0.5, 2.8); ax2.set_ylim(-0.6, 2.6)

plt.tight_layout()
fig2.savefig("tools/tracking_detail.png", dpi=150, bbox_inches="tight")
print("Saved: tools/tracking_detail.png")
plt.close()

# ============================================================
# Figure 3: GIF animation
# ============================================================
fig3, ax3 = plt.subplots(figsize=(8, 8))
ax3.set_xlim(-0.5, 2.8); ax3.set_ylim(-0.6, 2.6)
ax3.set_aspect("equal")
ax3.plot(WP[:, 0], WP[:, 1], "s-", color="gray", ms=10, alpha=0.3, lw=2)
for i, (wx, wy) in enumerate(WP):
    ax3.annotate(f"WP{i}", (wx, wy), textcoords="offset points",
                 xytext=(8, 8), fontsize=9, color="gray")

(vst_line,) = ax3.plot([], [], "b-", lw=1.2, alpha=0.5, label="virtual")
(car_line,) = ax3.plot([], [], "r-", lw=2.0, alpha=0.9, label="real")
(vst_dot,) = ax3.plot([], [], "bo", ms=6)
(car_dot,) = ax3.plot([], [], "ro", ms=8)
(err_line,) = ax3.plot([], [], "r:", lw=0.8, alpha=0.7)
time_text = ax3.text(0.02, 0.98, "", transform=ax3.transAxes, va="top",
                     fontsize=11, fontfamily="monospace",
                     bbox=dict(boxstyle="round", facecolor="white", alpha=0.85))
ax3.legend(fontsize=9, loc="lower right")
ax3.set_xlabel("X [m]"); ax3.set_ylabel("Y [m]")
ax3.grid(True, alpha=0.3)

N = len(t)
skip = max(1, N // 250)

def update(frame):
    n = min(frame * skip, N - 1)
    vst_line.set_data(vx[:n], vy[:n])
    car_line.set_data(cx[:n], cy[:n])
    vst_dot.set_data([vx[n]], [vy[n]])
    car_dot.set_data([cx[n]], [cy[n]])
    err_line.set_data([cx[n], vx[n]], [cy[n], vy[n]])
    time_text.set_text(
        f"t={t[n]:.2f}s  v={cv[n]:.1f}m/s\n"
        f"ey={ey[n]:+.3f}m  ex={ex[n]:+.3f}m\n"
        f"delta={np.rad2deg(delta[n]):+.1f}deg"
    )
    return vst_line, car_line, vst_dot, car_dot, err_line, time_text

ani = animation.FuncAnimation(fig3, update, frames=N // skip,
                               interval=50, blit=True, repeat=False)
ani.save("tools/tracking_anim.gif", writer="pillow", fps=18, dpi=90)
print(f"Saved: tools/tracking_anim.gif  ({N // skip} frames)")
plt.close()

print("\nDone!")
