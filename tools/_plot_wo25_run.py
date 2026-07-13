"""
_plot_wo25_run.py — LQR wo=25 激进高增益+高阻尼 运行数据可视化 (针对转向不足)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV = """0.013,0.001,0.000,0.0000,0.030,-0.0000,-0.0002,-0.1225,0.000,-0.000,-0.0002
0.020,0.001,-0.000,-0.0001,0.071,-0.0000,0.0000,-0.1801,0.001,-0.000,-0.0012
0.844,1.188,-0.496,-0.1316,2.423,0.1490,0.0323,0.1696,1.215,-0.347,-0.0296
0.944,1.424,-0.504,0.0584,2.328,0.1664,0.0538,0.2535,1.441,-0.332,0.1577
1.044,1.645,-0.468,0.2832,2.136,0.1769,0.0798,0.2769,1.646,-0.276,0.3726
1.144,1.837,-0.379,0.5887,2.072,0.1691,0.1143,0.2929,1.820,-0.180,0.6432
1.244,1.988,-0.240,0.8982,2.122,0.1442,0.1477,0.3062,1.952,-0.043,0.9480
1.346,2.090,-0.048,1.2535,2.381,0.1165,0.1513,-0.2868,2.044,0.136,1.1537
1.444,2.151,0.183,1.2739,2.454,0.0789,0.1286,0.0745,2.132,0.323,1.1249
1.544,2.252,0.423,1.1488,2.790,0.0640,0.1170,0.1553,2.231,0.545,1.1923
1.644,2.349,0.689,1.2779,2.878,0.0623,0.1004,0.1575,2.310,0.791,1.3219
1.744,2.423,0.966,1.3575,2.799,0.0744,0.0800,0.2085,2.355,1.042,1.4670
1.844,2.457,1.234,1.5407,2.634,0.0933,0.0683,0.1773,2.359,1.286,1.6335
1.944,2.443,1.482,1.7135,2.438,0.1077,0.0670,0.2444,2.324,1.511,1.8191
2.044,2.389,1.698,1.9468,2.129,0.1191,0.0818,0.2564,2.249,1.709,2.0440
2.144,2.284,1.881,2.2346,2.062,0.1160,0.1024,0.3362,2.135,1.870,2.3274
2.246,2.127,2.020,2.6048,2.017,0.0985,0.1216,0.2263,1.975,1.991,2.6449
2.344,1.926,2.107,2.8365,2.266,0.0843,0.1084,-0.2061,1.800,2.078,2.6752
2.444,1.697,2.191,2.7175,2.724,0.0546,0.1010,0.2002,1.592,2.181,2.7024
2.544,1.441,2.308,2.7788,2.966,0.0581,0.0820,0.0756,1.356,2.275,2.8197
2.644,1.177,2.386,2.8909,2.713,0.0582,0.0732,0.2060,1.105,2.340,2.9557
2.744,0.912,2.443,2.9944,2.676,0.0771,0.0651,0.1933,0.855,2.366,3.1108
2.844,0.655,2.453,3.2033,2.488,0.0934,0.0607,0.1929,0.618,2.353,3.2848
2.944,0.419,2.420,3.3664,2.231,0.1076,0.0633,0.2750,0.404,2.300,3.4823
3.044,0.214,2.346,3.6183,2.091,0.1179,0.0793,0.3302,0.221,2.208,3.7312
3.144,0.047,2.225,3.9308,2.037,0.1166,0.1049,0.3213,0.078,2.076,4.0303
3.246,-0.072,2.052,4.2814,2.193,0.1025,0.1261,0.1510,-0.023,1.899,4.3289
3.344,-0.136,1.834,4.5203,2.427,0.0814,0.1286,0.1708,-0.079,1.702,4.5213
3.444,-0.172,1.575,4.6349,2.707,0.0721,0.1191,0.0984,-0.104,1.464,4.6690
3.544,-0.177,1.295,4.7339,2.842,0.0701,0.0987,0.0584,-0.104,1.210,4.7469
3.644,-0.171,1.012,4.7326,2.810,0.0759,0.0740,0.0654,-0.091,0.954,4.7713
3.744,-0.165,0.739,4.7313,2.656,0.0879,0.0499,0.0613,-0.074,0.706,4.7835
3.844,-0.161,0.489,4.7293,2.390,0.1016,0.0325,0.1057,-0.057,0.474,4.7938
3.944,-0.155,0.266,4.7574,2.104,0.1176,0.0262,0.1061,-0.036,0.263,4.8328
4.044,-0.141,0.072,4.8104,1.874,0.1277,0.0177,-0.1249,-0.013,0.074,4.8119
4.145,-0.124,-0.099,4.7861,1.529,0.1211,0.0038,-0.0380,-0.003,-0.100,4.7341
4.244,-0.119,-0.227,4.7162,1.113,0.1155,-0.0039,-0.0096,-0.003,-0.222,4.6842
4.344,-0.121,-0.313,4.6680,0.642,0.1129,-0.0065,0.0054,-0.008,-0.312,4.6476
4.444,-0.123,-0.358,4.6342,0.069,0.1119,-0.0066,0.0552,-0.011,-0.361,4.6274
4.544,-0.124,-0.370,4.6316,0.044,0.1118,-0.0077,0.1384,-0.012,-0.372,4.6229
4.644,-0.124,-0.369,4.6326,-0.050,0.1118,-0.0075,0.2027,-0.012,-0.372,4.6229
4.744,-0.124,-0.366,4.6320,-0.035,0.1118,-0.0038,0.2415,-0.012,-0.372,4.6229
4.844,-0.124,-0.364,4.6317,-0.010,0.1118,-0.0019,0.2620,-0.012,-0.372,4.6229
4.944,-0.124,-0.363,4.6318,-0.003,0.1118,-0.0014,0.2721,-0.012,-0.372,4.6229
5.044,-0.124,-0.363,4.6317,-0.002,0.1118,-0.0011,0.2780,-0.012,-0.372,4.6229
5.144,-0.124,-0.363,4.6312,-0.003,0.1118,-0.0007,0.2817,-0.012,-0.372,4.6229
5.244,-0.124,-0.362,4.6307,-0.005,0.1118,-0.0002,0.2839,-0.012,-0.372,4.6229
5.344,-0.124,-0.362,4.6301,-0.005,0.1119,0.0001,0.2855,-0.012,-0.372,4.6229
5.444,-0.124,-0.362,4.6299,-0.000,0.1119,0.0002,0.2860,-0.012,-0.372,4.6229
5.544,-0.124,-0.362,4.6302,-0.000,0.1119,0.0002,0.2858,-0.012,-0.372,4.6229
5.644,-0.124,-0.362,4.6315,0.004,0.1119,0.0002,0.2843,-0.012,-0.372,4.6229"""

t, x, y, th, v, ey, ex, delta, vx, vy, vth = [], [], [], [], [], [], [], [], [], [], []
for line in CSV.strip().split("\n"):
    p = line.split(",")
    t.append(float(p[0])); x.append(float(p[1])); y.append(float(p[2]))
    th.append(float(p[3])); v.append(float(p[4]))
    ey.append(float(p[5])); ex.append(float(p[6])); delta.append(float(p[7]))
    vx.append(float(p[8])); vy.append(float(p[9])); vth.append(float(p[10]))

t=np.array(t); x=np.array(x); y=np.array(y); th=np.array(th); v=np.array(v)
ey=np.array(ey); ex=np.array(ex); delta=np.array(delta)
vx=np.array(vx); vy=np.array(vy); vth=np.array(vth)
wp=np.array([[2,0],[2,2],[0,2],[0,0]])
wp_hits=[(1.346,0,0.1030),(2.246,1,0.1317),(3.246,2,0.0896),(4.145,3,0.1577)]

# ===== Dark theme =====
BG="#1a1a19"; SURF="#22221f"; P="#ffffff"; S="#c3c2b7"; M="#898781"; G="#2c2c2a"
BLUE="#3987e5"; AQUA="#199e70"; RED="#e66767"; ORANGE="#d95926"; YELLOW="#c98500"

plt.rcParams.update({"figure.facecolor":BG,"axes.facecolor":SURF,"axes.edgecolor":G,
    "axes.labelcolor":S,"text.color":P,"xtick.color":M,"ytick.color":M,
    "grid.color":G,"grid.alpha":0.4,"legend.facecolor":SURF,"legend.edgecolor":G,
    "legend.labelcolor":S,"font.size":8,"axes.titlesize":10,"axes.labelsize":8})

fig,axes=plt.subplots(2,3,figsize=(16,9)); fig.patch.set_facecolor(BG)

# (0,0) Trajectory
ax=axes[0,0]
ax.plot(x,y,color=RED,lw=1.5,label="Real"); ax.plot(vx,vy,color=BLUE,lw=1.0,alpha=0.6,ls="--",label="Virtual")
ax.scatter(wp[:,0],wp[:,1],c=YELLOW,s=60,marker="s",zorder=5,edgecolors="none",label="WP")
for i,(tx,ty) in enumerate(wp): ax.annotate(f"WP{i}",(tx,ty),textcoords="offset points",xytext=(8,8),color=YELLOW,fontsize=8,fontweight="bold")
for _,wpi,_ in wp_hits: ax.add_patch(plt.Circle(wp[wpi],0.05,fill=False,edgecolor=AQUA,lw=1.0,ls=":",alpha=0.7))
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_title("Trajectory",fontweight="bold")
ax.set_aspect("equal"); ax.legend(fontsize=7,loc="lower left"); ax.grid(True,alpha=0.3)

# (0,1) Lateral error
ax=axes[0,1]
ax.fill_between(t,0,ey,color=BLUE,alpha=0.15); ax.plot(t,ey,color=BLUE,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls="--"); pk=np.argmax(np.abs(ey))
ax.annotate(f"{ey[pk]:.3f}m",(t[pk],ey[pk]),textcoords="offset points",xytext=(0,12),color=BLUE,fontsize=7,ha="center")
ax.set_xlabel("t [s]"); ax.set_ylabel("ey [m]"); ax.set_title("Lateral Error",fontweight="bold"); ax.grid(True,alpha=0.3)

# (0,2) Longitudinal error
ax=axes[0,2]
ax.fill_between(t,0,ex,color=AQUA,alpha=0.12); ax.plot(t,ex,color=AQUA,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls="--")
ax.set_xlabel("t [s]"); ax.set_ylabel("ex [m]"); ax.set_title("Longitudinal Error",fontweight="bold"); ax.grid(True,alpha=0.3)

# (1,0) Speed
ax=axes[1,0]
ax.plot(t,v,color=ORANGE,lw=1.8); ax.axhline(3.5,color=RED,lw=0.8,ls="--",alpha=0.5,label="v_max=3.5")
ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]"); ax.set_title("Speed",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

# (1,1) Steering
ax=axes[1,1]
ax.plot(t,np.rad2deg(delta),color=YELLOW,lw=1.5)
ax.axhline(26.5,color=RED,lw=0.8,ls="--",alpha=0.5,label=r"$\delta_{max}$=26.5°"); ax.axhline(-26.5,color=RED,lw=0.8,ls="--",alpha=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\delta$ [°]"); ax.set_title("Steering Angle",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

# (1,2) Heading
ax=axes[1,2]
ax.plot(t,np.rad2deg(th),color=RED,lw=1.2,label="Real")
ax.plot(t,np.rad2deg(vth),color=BLUE,lw=0.8,alpha=0.5,ls="--",label="Virtual")
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\theta$ [°]"); ax.set_title("Heading",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ey_rms=np.sqrt(np.mean(ey**2)); ex_rms=np.sqrt(np.mean(ex**2))
min_ds=[h[2] for h in wp_hits]
fig.suptitle(f"LQR  wo=25  R=0.003  Q=diag([1,0.8,0.15,0])  α=0.65  (aggressive high-gain)  |  "
    f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  "
    f"WP min_d=[{min_ds[0]:.3f},{min_ds[1]:.3f},{min_ds[2]:.3f},{min_ds[3]:.3f}]m",
    fontsize=11,fontweight="bold",color=P,y=0.98)
plt.tight_layout(rect=[0,0,1,0.94])
out="tools/lqr_wo25_run_dashboard.png"
fig.savefig(out,dpi=150,facecolor=BG,edgecolor="none"); plt.close()
print(f"Saved: {out}")
print(f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v):.2f}m/s")
for _t,wi,md in wp_hits: print(f"  WP{wi} @t={_t:.2f}s  min_d={md:.4f}m")
