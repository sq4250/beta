"""
_plot_wo20_run.py — LQR wo=20 高增益+高阻尼 运行数据可视化
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ===== 数据 =====
CSV = """0.013,0.001,0.000,0.0000,0.027,-0.0000,-0.0002,-0.1218,0.000,-0.000,-0.0002
0.020,0.001,0.000,0.0000,0.024,-0.0000,0.0003,-0.1790,0.001,-0.000,-0.0012
0.844,1.187,-0.494,-0.1112,2.369,0.1474,0.0334,0.1779,1.216,-0.346,-0.0300
0.944,1.424,-0.498,0.0705,2.313,0.1605,0.0530,0.2535,1.442,-0.332,0.1573
1.044,1.643,-0.457,0.3161,2.148,0.1670,0.0777,0.2467,1.647,-0.276,0.3724
1.144,1.833,-0.365,0.5858,1.990,0.1557,0.1090,0.3316,1.820,-0.179,0.6433
1.244,1.982,-0.223,0.9382,2.012,0.1292,0.1368,0.2891,1.952,-0.043,0.9482
1.346,2.077,-0.034,1.2529,2.294,0.0990,0.1445,-0.1997,2.045,0.136,1.1539
1.444,2.135,0.195,1.3041,2.494,0.0597,0.1251,-0.0170,2.132,0.324,1.1251
1.544,2.233,0.442,1.1334,2.758,0.0394,0.1063,0.2211,2.231,0.545,1.1924
1.644,2.340,0.706,1.2581,2.874,0.0487,0.0867,0.1277,2.310,0.791,1.3220
1.744,2.409,0.980,1.3736,2.760,0.0596,0.0678,0.1973,2.355,1.043,1.4671
1.844,2.444,1.242,1.5243,2.480,0.0808,0.0594,0.1920,2.359,1.286,1.6337
1.944,2.428,1.486,1.7345,2.351,0.0938,0.0602,0.2288,2.323,1.512,1.8194
2.044,2.371,1.702,1.9338,2.239,0.1041,0.0705,0.3077,2.249,1.709,2.0439
2.144,2.270,1.886,2.2355,2.071,0.1083,0.0894,0.3010,2.135,1.870,2.3276
2.246,2.117,2.026,2.5577,2.183,0.0979,0.1108,0.2817,1.975,1.991,2.6452
2.344,1.919,2.113,2.8454,2.247,0.0865,0.0997,-0.2066,1.800,2.078,2.6755
2.444,1.688,2.199,2.6828,2.840,0.0569,0.0906,0.2287,1.592,2.181,2.7025
2.544,1.440,2.327,2.7102,2.620,0.0751,0.0737,0.1528,1.356,2.275,2.8198
2.644,1.186,2.410,2.9154,2.708,0.0840,0.0771,0.1216,1.105,2.340,2.9557
2.744,0.919,2.462,2.9975,2.690,0.0962,0.0717,0.2243,0.855,2.366,3.1108
2.844,0.661,2.475,3.1945,2.530,0.1143,0.0697,0.1955,0.618,2.353,3.2849
2.944,0.425,2.438,3.4020,2.328,0.1224,0.0748,0.2464,0.404,2.300,3.4817
3.044,0.218,2.360,3.6201,2.157,0.1269,0.0902,0.3457,0.221,2.208,3.7309
3.144,0.050,2.238,3.9409,2.078,0.1225,0.1161,0.3211,0.078,2.076,4.0301
3.246,-0.067,2.059,4.2985,2.147,0.1010,0.1344,0.1844,-0.023,1.899,4.3287
3.344,-0.130,1.838,4.5380,2.445,0.0763,0.1336,0.1368,-0.079,1.702,4.5211
3.444,-0.159,1.579,4.6549,2.755,0.0594,0.1233,0.1057,-0.105,1.465,4.6689
3.544,-0.162,1.298,4.7347,2.858,0.0553,0.1011,0.0506,-0.104,1.210,4.7469
3.644,-0.154,1.013,4.7438,2.844,0.0587,0.0737,0.0547,-0.091,0.954,4.7713
3.744,-0.146,0.740,4.7329,2.618,0.0689,0.0486,0.0614,-0.074,0.707,4.7835
3.844,-0.142,0.490,4.7290,2.365,0.0828,0.0317,0.0950,-0.057,0.475,4.7939
3.944,-0.137,0.268,4.7505,2.103,0.0992,0.0254,0.1166,-0.036,0.263,4.8333
4.044,-0.124,0.073,4.7997,1.822,0.1111,0.0171,-0.0996,-0.013,0.074,4.8118
4.145,-0.110,-0.098,4.7723,1.550,0.1068,0.0048,-0.0238,-0.003,-0.100,4.7376
4.244,-0.106,-0.222,4.7129,0.955,0.1038,0.0024,0.0045,-0.002,-0.222,4.6978
4.344,-0.109,-0.311,4.6623,0.417,0.1033,-0.0025,0.0355,-0.005,-0.311,4.6685
4.444,-0.111,-0.356,4.6377,0.012,0.1038,-0.0007,0.0770,-0.008,-0.361,4.6523
4.544,-0.112,-0.365,4.6355,0.117,0.1039,0.0012,0.1475,-0.008,-0.372,4.6488
4.644,-0.112,-0.370,4.6489,0.096,0.1039,-0.0045,0.1865,-0.008,-0.372,4.6488
4.744,-0.113,-0.371,4.6500,-0.043,0.1039,-0.0056,0.2157,-0.008,-0.372,4.6488
4.844,-0.112,-0.367,4.6472,-0.013,0.1039,-0.0020,0.2368,-0.008,-0.372,4.6488
4.944,-0.112,-0.367,4.6472,-0.006,0.1039,-0.0016,0.2486,-0.008,-0.372,4.6488
5.044,-0.112,-0.366,4.6469,-0.003,0.1039,-0.0011,0.2561,-0.008,-0.372,4.6488
5.144,-0.112,-0.366,4.6467,-0.003,0.1039,-0.0007,0.2607,-0.008,-0.372,4.6488
5.244,-0.112,-0.366,4.6466,-0.001,0.1039,-0.0005,0.2636,-0.008,-0.372,4.6488
5.344,-0.112,-0.366,4.6465,-0.000,0.1039,-0.0004,0.2655,-0.008,-0.372,4.6488
5.444,-0.112,-0.366,4.6464,-0.000,0.1039,-0.0003,0.2667,-0.008,-0.372,4.6488
5.544,-0.112,-0.365,4.6461,-0.000,0.1039,-0.0002,0.2676,-0.008,-0.372,4.6488
5.644,-0.112,-0.365,4.6458,-0.000,0.1039,-0.0001,0.2682,-0.008,-0.372,4.6488
5.744,-0.112,-0.365,4.6457,-0.000,0.1039,-0.0001,0.2686,-0.008,-0.372,4.6488
5.844,-0.112,-0.365,4.6466,-0.001,0.1039,0.0002,0.2679,-0.008,-0.372,4.6488
5.944,-0.112,-0.365,4.6476,0.000,0.1039,0.0001,0.2674,-0.008,-0.372,4.6488"""

t, x, y, th, v, ey, ex, delta, vx, vy, vth = [], [], [], [], [], [], [], [], [], [], []
for line in CSV.strip().split("\n"):
    parts = line.split(",")
    t.append(float(parts[0]))
    x.append(float(parts[1])); y.append(float(parts[2]))
    th.append(float(parts[3])); v.append(float(parts[4]))
    ey.append(float(parts[5])); ex.append(float(parts[6]))
    delta.append(float(parts[7]))
    vx.append(float(parts[8])); vy.append(float(parts[9]))
    vth.append(float(parts[10]))

t = np.array(t); x = np.array(x); y = np.array(y); th = np.array(th); v = np.array(v)
ey = np.array(ey); ex = np.array(ex); delta = np.array(delta)
vx = np.array(vx); vy = np.array(vy); vth = np.array(vth)

wp = np.array([[2, 0], [2, 2], [0, 2], [0, 0]])
# WP hit annotations: time, wp_index, min_distance
wp_hits = [
    (1.346, 0, 0.0850),
    (2.246, 1, 0.1228),
    (3.246, 2, 0.0911),
    (4.145, 3, 0.1441),
]

# ===== Dark theme for robotics/embedded context =====
BG = "#1a1a19"
SURFACE = "#22221f"
PRIMARY = "#ffffff"
SECONDARY = "#c3c2b7"
MUTED = "#898781"
GRID = "#2c2c2a"
BLUE = "#3987e5"
AQUA = "#199e70"
RED = "#e66767"
ORANGE = "#d95926"
YELLOW = "#c98500"

plt.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": SECONDARY,
    "text.color": PRIMARY, "xtick.color": MUTED, "ytick.color": MUTED,
    "grid.color": GRID, "grid.alpha": 0.4,
    "legend.facecolor": SURFACE, "legend.edgecolor": GRID,
    "legend.labelcolor": SECONDARY, "font.size": 8,
    "axes.titlesize": 10, "axes.labelsize": 8,
})

fig, axes = plt.subplots(2, 3, figsize=(16, 9))
fig.patch.set_facecolor(BG)

# ── (0, 0) Trajectory ──
ax = axes[0, 0]
ax.plot(x, y, color=RED, lw=1.5, label="Real car")
ax.plot(vx, vy, color=BLUE, lw=1.0, alpha=0.6, linestyle="--", label="Virtual car")
ax.scatter(wp[:, 0], wp[:, 1], c=YELLOW, s=60, marker="s", zorder=5,
           edgecolors="none", label="Waypoint")
for i, (tx, ty) in enumerate(wp):
    ax.annotate(f"WP{i}", (tx, ty), textcoords="offset points", xytext=(8, 8),
                color=YELLOW, fontsize=8, fontweight="bold")
for _t_hit, wpi, _min_d in wp_hits:
    ax.add_patch(plt.Circle(wp[wpi], 0.05, fill=False, edgecolor=AQUA,
                            lw=1.0, linestyle=":", alpha=0.7))
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
ax.set_title("Trajectory", fontweight="bold")
ax.set_aspect("equal"); ax.legend(fontsize=7, loc="lower left")
ax.grid(True, alpha=0.3)

# ── (0, 1) Lateral error ──
ax = axes[0, 1]
ax.fill_between(t, 0, ey, color=BLUE, alpha=0.15)
ax.plot(t, ey, color=BLUE, lw=1.5)
ax.axhline(0, color=MUTED, lw=0.8, linestyle="--")
ax.set_xlabel("t [s]"); ax.set_ylabel("ey [m]")
ax.set_title("Lateral Error", fontweight="bold")
ax.grid(True, alpha=0.3)
peak_i = np.argmax(np.abs(ey))
ax.annotate(f"{ey[peak_i]:.3f}m", (t[peak_i], ey[peak_i]),
            textcoords="offset points", xytext=(0, 12), color=BLUE, fontsize=7, ha="center")

# ── (0, 2) Longitudinal error ──
ax = axes[0, 2]
ax.fill_between(t, 0, ex, color=AQUA, alpha=0.12)
ax.plot(t, ex, color=AQUA, lw=1.5)
ax.axhline(0, color=MUTED, lw=0.8, linestyle="--")
ax.set_xlabel("t [s]"); ax.set_ylabel("ex [m]")
ax.set_title("Longitudinal Error", fontweight="bold")
ax.grid(True, alpha=0.3)

# ── (1, 0) Speed ──
ax = axes[1, 0]
ax.plot(t, v, color=ORANGE, lw=1.8)
ax.axhline(3.5, color=RED, lw=0.8, linestyle="--", alpha=0.5, label="v_max=3.5")
ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]")
ax.set_title("Speed", fontweight="bold")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# ── (1, 1) Steering ──
ax = axes[1, 1]
ax.plot(t, np.rad2deg(delta), color=YELLOW, lw=1.5)
ax.axhline(26.5, color=RED, lw=0.8, linestyle="--", alpha=0.5, label=r"$\delta_{max}$=26.5°")
ax.axhline(-26.5, color=RED, lw=0.8, linestyle="--", alpha=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\delta$ [°]")
ax.set_title("Steering Angle", fontweight="bold")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# ── (1, 2) Heading ──
ax = axes[1, 2]
ax.plot(t, np.rad2deg(th), color=RED, lw=1.2, label="Real")
ax.plot(t, np.rad2deg(vth), color=BLUE, lw=0.8, alpha=0.5, linestyle="--", label="Virtual")
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\theta$ [°]")
ax.set_title("Heading", fontweight="bold")
ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# ── Suptitle ──
ey_rms = np.sqrt(np.mean(ey ** 2))
ex_rms = np.sqrt(np.mean(ex ** 2))
min_ds = [h[2] for h in wp_hits]
fig.suptitle(
    f"LQR  wo=20  R=0.005  Q=diag([1,0.5,0.10,0])  α=0.75  (high-gain + high-damping)  |  "
    f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  "
    f"WP min_d=[{min_ds[0]:.3f}, {min_ds[1]:.3f}, {min_ds[2]:.3f}, {min_ds[3]:.3f}]m",
    fontsize=11, fontweight="bold", color=PRIMARY, y=0.98)

plt.tight_layout(rect=[0, 0, 1, 0.94])
out = "tools/lqr_wo20_run_dashboard.png"
fig.savefig(out, dpi=150, facecolor=BG, edgecolor="none")
plt.close()
print(f"Saved: {out}")
print(f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v):.2f}m/s")
for _t_hit, wpi, min_d in wp_hits:
    print(f"  WP{wpi} @t={_t_hit:.2f}s  min_d={min_d:.4f}m")
