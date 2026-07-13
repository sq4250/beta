"""
plot_run3.py — 方案A 实车数据可视化 + 三趟对比
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ===== 方案A 数据 (Qθ=1.0, R=0.10, wo=35) =====
raw = """0.844,1.197,-0.461,-0.1079,2.409,0.1134,0.0244,0.1764,1.215,-0.347,-0.0296
0.944,1.432,-0.461,0.1102,2.307,0.1252,0.0386,0.1947,1.441,-0.332,0.1577
1.044,1.646,-0.408,0.3748,2.091,0.1229,0.0568,0.2173,1.646,-0.276,0.3726
1.144,1.829,-0.307,0.6048,2.013,0.1075,0.0767,0.2818,1.820,-0.180,0.6432
1.244,1.976,-0.170,0.9150,1.966,0.0933,0.0966,0.3008,1.952,-0.043,0.9480
1.346,2.073,0.020,1.2745,2.169,0.0729,0.0963,-0.0368,2.044,0.136,1.1537
1.444,2.111,0.239,1.4521,2.339,0.0204,0.0940,-0.0751,2.132,0.323,1.1249
1.544,2.145,0.491,1.4100,2.739,-0.0581,0.0927,-0.0199,2.231,0.545,1.1898
1.644,2.197,0.766,1.3567,2.817,-0.1031,0.0627,0.0216,2.310,0.791,1.3236
1.744,2.261,1.039,1.3293,2.766,-0.0936,0.0247,0.0946,2.354,1.043,1.4680
1.844,2.316,1.290,1.4093,2.593,-0.0452,0.0043,0.1651,2.359,1.286,1.6339
1.944,2.337,1.524,1.5632,2.104,0.0143,-0.0001,0.2264,2.323,1.512,1.8194
2.044,2.312,1.735,1.8273,2.021,0.0662,0.0139,0.2916,2.249,1.709,2.0445
2.144,2.233,1.919,2.1307,1.993,0.1038,0.0404,0.3413,2.134,1.870,2.3280
2.246,2.101,2.064,2.4703,1.920,0.1245,0.0780,0.3187,1.974,1.991,2.6428
2.344,1.912,2.162,2.8218,2.421,0.1266,0.0723,0.0167,1.800,2.078,2.6734
2.444,1.683,2.214,2.9625,2.530,0.0711,0.0774,-0.0032,1.593,2.181,2.7016
2.544,1.425,2.258,2.9780,2.654,0.0069,0.0801,0.0173,1.357,2.276,2.8190
2.644,1.148,2.301,2.9961,2.895,-0.0301,0.0593,0.0483,1.106,2.340,2.9551
2.744,0.880,2.338,3.0202,2.670,-0.0292,0.0351,0.1077,0.857,2.367,3.1105
2.844,0.629,2.359,3.1171,2.466,0.0020,0.0193,0.1686,0.620,2.354,3.2845
2.944,0.399,2.347,3.2848,2.185,0.0435,0.0185,0.2305,0.405,2.301,3.4811
3.044,0.196,2.292,3.5292,2.014,0.0813,0.0324,0.3145,0.222,2.209,3.7298
3.144,0.024,2.192,3.8120,1.987,0.1128,0.0619,0.3570,0.078,2.078,4.0288
3.246,-0.113,2.038,4.1593,2.089,0.1350,0.0957,0.2799,-0.023,1.900,4.3275
3.344,-0.194,1.830,4.4904,2.250,0.1376,0.1116,0.1957,-0.079,1.703,4.5203
3.444,-0.212,1.577,4.7798,2.498,0.1133,0.1165,0.0880,-0.104,1.466,4.6684
3.544,-0.165,1.308,4.9594,3.104,0.0609,0.1109,-0.0132,-0.104,1.212,4.7466
3.644,-0.091,1.045,4.9949,2.725,-0.0028,0.1004,-0.0625,-0.091,0.955,4.7711
3.744,-0.021,0.782,4.9449,2.670,-0.0574,0.0814,-0.0764,-0.075,0.707,4.7837
3.844,0.027,0.528,4.8496,2.469,-0.0881,0.0553,-0.0394,-0.057,0.475,4.7939
3.944,0.049,0.297,4.7640,2.247,-0.0884,0.0319,0.0085,-0.036,0.264,4.8331
4.044,0.054,0.098,4.7194,1.977,-0.0693,0.0240,-0.0836,-0.013,0.075,4.8117"""

lines = [l.strip().split(",") for l in raw.strip().split("\n")]
data = np.array([[float(x) for x in l] for l in lines])
t = data[:, 0]
cx, cy, cth, cv = data[:, 1], data[:, 2], data[:, 3], data[:, 4]
ey, ex, delta = data[:, 5], data[:, 6], data[:, 7]
vx, vy, vth = data[:, 8], data[:, 9], data[:, 10]
WP = np.array([[2, 0], [2, 2], [0, 2], [0, 0]])

# Compute min distances
wp_min_d = []
for wp in WP:
    min_d = float("inf")
    for j in range(len(t)):
        d = np.sqrt((cx[j] - wp[0]) ** 2 + (cy[j] - wp[1]) ** 2)
        if d < min_d: min_d = d
    for j in range(len(t) - 1):
        for k in range(10):
            a = k / 10.0
            x = cx[j] + a * (cx[j + 1] - cx[j])
            y = cy[j] + a * (cy[j + 1] - cy[j])
            d = np.sqrt((x - wp[0]) ** 2 + (y - wp[1]) ** 2)
            if d < min_d: min_d = d
    wp_min_d.append(min_d * 100)

# ============================================================
# Figure 1: Dashboard
# ============================================================
fig, axes = plt.subplots(2, 3, figsize=(16, 10))
par_str = r"$Q_\theta$=1.0  $R_\omega$=0.10  $w_o$=35  $\alpha$=0.3"
fig.suptitle(f"方案A 实车测试 | {par_str} | WP min_d: "
             f"{wp_min_d[0]:.0f}/{wp_min_d[1]:.0f}/{wp_min_d[2]:.0f}/{wp_min_d[3]:.0f} cm",
             fontsize=12, fontweight="bold")

# (0,0) Trajectory
ax = axes[0, 0]
ax.plot(WP[:, 0], WP[:, 1], "s-", color="gray", ms=14, alpha=0.3, lw=2, label="waypoints")
for i, (wx, wy) in enumerate(WP):
    ax.annotate(f"WP{i}", (wx, wy), textcoords="offset points", xytext=(8, 8), fontsize=10, color="gray")
ax.plot(vx, vy, "b-", lw=1.2, alpha=0.5, label="virtual (vst)")
ax.plot(cx, cy, "r-", lw=2.2, alpha=0.88, label="real (car)")
for i, wp in enumerate(WP):
    dists = np.sqrt((cx - wp[0]) ** 2 + (cy - wp[1]) ** 2)
    idx = np.argmin(dists)
    ax.plot(cx[idx], cy[idx], "o", ms=9, mew=2, markerfacecolor="yellow", markeredgecolor="red")
    ax.plot([cx[idx], wp[0]], [cy[idx], wp[1]], "r--", lw=0.7, alpha=0.5)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
ax.set_title("Trajectory (CCW square)")
ax.set_aspect("equal"); ax.legend(fontsize=8, loc="lower left"); ax.grid(True, alpha=0.3)
ax.set_xlim(-0.5, 2.8); ax.set_ylim(-0.6, 2.6)

# (0,1) ey
ax = axes[0, 1]
ax.fill_between(t, 0, ey, alpha=0.12, color="red")
ax.plot(t, ey, "r-", lw=1.5)
ax.axhline(0, color="gray", lw=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel("ey [m]")
ax.set_title("Lateral Error ey  (+outside, -inside)")
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
# Grip-limited speed for delta=0.46
v_grip = np.sqrt(6 * 0.15 / 0.5)
ax.axhline(v_grip, color="orange", ls="--", alpha=0.5, label=f"grip v_max≈{v_grip:.1f} (@a_lat=6m/s²)")
ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]")
ax.set_title("Speed vs Grip Limit")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# (1,1) delta
ax = axes[1, 1]
ax.plot(t, np.rad2deg(delta), "purple", lw=1.5)
ax.axhline(np.rad2deg(0.4636), color="r", ls="--", alpha=0.4, label=r"$\pm$26.6°")
ax.axhline(-np.rad2deg(0.4636), color="r", ls="--", alpha=0.4)
ax.set_xlabel("t [s]"); ax.set_ylabel("delta [deg]")
ax.set_title("Steering Command")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# (1,2) a_lat estimate
ax = axes[1, 2]
a_lat_est = cv**2 * np.abs(np.tan(delta)) / 0.15
ax.plot(t, a_lat_est, "orange", lw=1.5)
ax.axhline(6.0, color="r", ls="--", alpha=0.5, label="grip limit ~6 m/s²")
ax.axhline(10.0, color="gray", ls=":", alpha=0.3, label="10 m/s²")
ax.fill_between(t, 6.0, a_lat_est, where=(a_lat_est > 6.0), alpha=0.15, color="red")
ax.set_xlabel("t [s]"); ax.set_ylabel("a_lat [m/s²]")
ax.set_title("Estimated Lateral Accel  v²|tanδ|/L")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig("tools/run3_dashboard.png", dpi=150, bbox_inches="tight")
print("Saved: tools/run3_dashboard.png")
plt.close()

# ============================================================
# Figure 2: 三趟精度对比 bar chart
# ============================================================
fig2, ax2 = plt.subplots(figsize=(10, 5))
wp_labels = ["WP0\n(2,0)", "WP1\n(2,2)", "WP2\n(0,2)", "WP3\n(0,0)"]
x = np.arange(len(wp_labels))
width = 0.22

# 数据
run1 = [3.1, 5.6, 4.9, 3.8]    # Qθ=0.5
run3 = [5.7, 10.3, 11.0, 11.2] # Qθ=1.0 (方案A)
run2 = [9.2, 8.2, 11.9, np.nan] # Qθ=2.0

bars1 = ax2.bar(x - width, run1, width, label="Qθ=0.5 R=0.05 wo=50 (最优)", color="#2ecc71", edgecolor="white")
bars3 = ax2.bar(x, run3, width, label="Qθ=1.0 R=0.10 wo=35 (方案A)", color="#f39c12", edgecolor="white")
bars2 = ax2.bar(x + width, run2, width, label="Qθ=2.0 R=0.06 wo=50", color="#e74c3c", edgecolor="white")

# 标注数值
for bars in [bars1, bars3, bars2]:
    for bar in bars:
        h = bar.get_height()
        if not np.isnan(h):
            ax2.text(bar.get_x() + bar.get_width() / 2., h + 0.3, f"{h:.1f}",
                     ha="center", va="bottom", fontsize=9, fontweight="bold")

ax2.set_ylabel("Min Distance [cm]", fontsize=12)
ax2.set_title("Waypoint Accuracy: 3 Runs vs LQR Parameters", fontsize=13, fontweight="bold")
ax2.set_xticks(x)
ax2.set_xticklabels(wp_labels)
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3, axis="y")
ax2.set_ylim(0, 16)

# 添加趋势标注
ax2.annotate("Qθ ↑ → accuracy ↓", xy=(2.5, 14), fontsize=11, color="red",
             bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))

plt.tight_layout()
fig2.savefig("tools/runs_comparison.png", dpi=150, bbox_inches="tight")
print("Saved: tools/runs_comparison.png")
plt.close()

# ============================================================
# Figure 3: a_lat vs grip — the real bottleneck
# ============================================================
fig3, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(14, 8), gridspec_kw={"height_ratios": [2, 1]})
fig3.suptitle("Physical Limit: Lateral Acceleration vs Tire Grip", fontsize=13, fontweight="bold")

a_lat_est = cv**2 * np.abs(np.tan(delta)) / 0.15
grip_limit = 6.0  # m/s²

ax_a.fill_between(t, grip_limit, a_lat_est, where=(a_lat_est > grip_limit),
                  alpha=0.2, color="red", label=f"超出抓地极限 (> {grip_limit} m/s²)")
ax_a.fill_between(t, 0, a_lat_est, where=(a_lat_est <= grip_limit),
                  alpha=0.15, color="green", label="在线性区")
ax_a.axhline(grip_limit, color="red", ls="--", lw=1.5, alpha=0.7, label=f"估算抓地极限 {grip_limit} m/s²")
ax_a.plot(t, a_lat_est, "orange", lw=1.5)
# 标记每个弯的 a_lat 峰值
for seg, (t0, t1) in [("弯1", (1.3, 2.1)), ("弯2", (2.3, 3.1)), ("弯3", (3.3, 4.1))]:
    mask = (t >= t0) & (t <= t1)
    if mask.sum() > 0:
        peak = np.max(a_lat_est[mask])
        t_peak = t[mask][np.argmax(a_lat_est[mask])]
        ax_a.annotate(f"{seg}\n{peak:.0f} m/s²\n({peak/9.8:.1f}g)", (t_peak, peak),
                      textcoords="offset points", xytext=(0, 12), fontsize=9, ha="center", color="red")
ax_a.set_ylabel("a_lat [m/s²]"); ax_a.set_xlabel("t [s]")
ax_a.legend(fontsize=9); ax_a.grid(True, alpha=0.3)
ax_a.set_ylim(0, 40)

# ey 与 a_lat 对照
ax_b.plot(t, ey * 100, "r-", lw=1.5, label="ey [cm]")
ax_b.axhline(0, color="gray", lw=0.5)
# 标注 a_lat > grip 的时段
for t0, t1 in [(1.0, 1.3), (1.3, 2.1), (2.3, 3.1), (3.3, 4.1)]:
    mask = (t >= t0) & (t <= t1)
    a_lat_peak = np.max(a_lat_est[mask]) if mask.sum() > 0 else 0
    if a_lat_peak > grip_limit:
        ax_b.axvspan(t0, t1, alpha=0.08, color="red")
ax_b.set_ylabel("ey [cm]"); ax_b.set_xlabel("t [s]")
ax_b.set_title("ey vs grip-saturated regions (red bands = a_lat > 6 m/s²)")
ax_b.legend(fontsize=9); ax_b.grid(True, alpha=0.3)

plt.tight_layout()
fig3.savefig("tools/grip_analysis.png", dpi=150, bbox_inches="tight")
print("Saved: tools/grip_analysis.png")
plt.close()

print("\nDone!")
