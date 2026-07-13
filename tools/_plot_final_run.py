"""
_plot_final_run.py — LQR wo=40 最终方案 运行数据可视化
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV = """0.013,0.001,-0.000,-0.0000,0.024,-0.0000,-0.0001,-0.1270,0.000,-0.000,-0.0002
0.020,0.001,-0.000,-0.0003,0.020,-0.0000,0.0003,-0.1866,0.001,-0.000,-0.0012
0.845,1.112,-0.567,-0.2268,2.716,0.2223,0.1104,0.2500,1.227,-0.347,-0.0219
0.945,1.380,-0.602,-0.0272,2.803,0.2562,0.1162,0.2710,1.452,-0.330,0.1663
1.045,1.645,-0.568,0.2809,2.617,0.2702,0.1204,0.2651,1.656,-0.273,0.3835
1.145,1.879,-0.466,0.5531,2.451,0.2629,0.1370,0.3544,1.827,-0.174,0.6574
1.245,2.047,-0.307,0.9685,2.283,0.2282,0.1716,0.2565,1.957,-0.036,0.9637
1.347,2.151,-0.104,1.2036,2.255,0.1933,0.1820,-0.2394,2.043,0.135,1.1684
1.445,2.220,0.112,1.2008,2.388,0.1723,0.1632,0.2808,2.133,0.333,1.1314
1.545,2.338,0.335,1.1079,2.656,0.1797,0.1660,0.0223,2.231,0.555,1.1961
1.645,2.435,0.585,1.2052,2.763,0.1741,0.1787,0.3736,2.309,0.801,1.3249
1.745,2.530,0.852,1.3066,2.935,0.1968,0.1817,0.1765,2.353,1.052,1.4708
1.845,2.579,1.139,1.4742,2.909,0.2113,0.1709,0.3224,2.356,1.295,1.6383
1.945,2.581,1.426,1.6996,2.906,0.2300,0.1556,0.0951,2.320,1.519,1.8244
2.045,2.514,1.687,1.9141,2.606,0.2258,0.1493,0.4337,2.244,1.715,2.0516
2.145,2.398,1.902,2.2570,2.356,0.2127,0.1668,0.2263,2.129,1.875,2.3362
2.247,2.222,2.054,2.5725,2.216,0.1738,0.1889,0.3808,1.977,1.990,2.6402
2.345,2.016,2.151,2.8026,2.416,0.1625,0.1678,-0.2209,1.793,2.080,2.6815
2.445,1.798,2.245,2.6138,2.496,0.1478,0.1659,0.3922,1.585,2.182,2.7053
2.545,1.570,2.373,2.7248,2.798,0.1619,0.1790,0.0584,1.349,2.275,2.8213
2.645,1.315,2.475,2.7665,2.822,0.1732,0.1882,0.3197,1.098,2.339,2.9576
2.745,1.042,2.558,2.9427,2.905,0.1981,0.1872,0.2039,0.849,2.366,3.1135
2.845,0.752,2.589,3.1268,2.914,0.2152,0.1723,0.2691,0.613,2.351,3.2875
2.945,0.470,2.558,3.3857,2.780,0.2213,0.1550,0.2102,0.399,2.298,3.4859
3.045,0.225,2.462,3.6442,2.556,0.2079,0.1501,0.3353,0.217,2.205,3.7374
3.145,0.036,2.314,3.9956,2.300,0.1809,0.1645,0.2460,0.075,2.073,4.0372
3.246,-0.085,2.121,4.2894,2.253,0.1411,0.1781,0.1772,-0.021,1.904,4.3240
3.345,-0.154,1.906,4.4704,2.364,0.1124,0.1894,0.2539,-0.080,1.698,4.5228
3.445,-0.204,1.664,4.5713,2.709,0.1072,0.1985,0.1094,-0.105,1.461,4.6698
3.545,-0.230,1.399,4.6194,2.746,0.1193,0.1965,0.1896,-0.104,1.207,4.7475
3.645,-0.254,1.114,4.6450,2.961,0.1531,0.1730,0.1072,-0.091,0.951,4.7719
3.745,-0.273,0.816,4.6547,3.018,0.1902,0.1259,0.1072,-0.074,0.704,4.7839
3.845,-0.282,0.519,4.7119,2.840,0.2202,0.0656,0.1033,-0.057,0.472,4.7945
3.945,-0.274,0.249,4.7891,2.447,0.2377,0.0175,0.0569,-0.036,0.261,4.8335
4.045,-0.245,0.028,4.8873,1.970,0.2360,-0.0211,-0.2246,-0.012,0.072,4.8111
4.146,-0.211,-0.155,4.8604,1.428,0.2097,-0.0546,-0.0083,-0.003,-0.094,4.7371
4.245,-0.196,-0.287,4.8058,1.081,0.1909,-0.0682,-0.0874,-0.003,-0.224,4.6859
4.345,-0.189,-0.366,4.7834,0.454,0.1783,-0.0641,-0.0583,-0.007,-0.313,4.6506
4.445,-0.187,-0.401,4.7609,0.447,0.1724,-0.0528,-0.0412,-0.011,-0.362,4.6310
4.545,-0.186,-0.408,4.7558,-0.066,0.1713,-0.0493,0.0386,-0.012,-0.373,4.6267
4.645,-0.187,-0.404,4.7518,-0.114,0.1719,-0.0453,0.2111,-0.012,-0.373,4.6267
4.745,-0.187,-0.393,4.7557,-0.171,0.1732,-0.0348,0.3556,-0.012,-0.373,4.6267
4.845,-0.188,-0.373,4.7324,-0.215,0.1757,-0.0145,0.4636,-0.012,-0.373,4.6267
4.945,-0.188,-0.353,4.6745,-0.140,0.1772,0.0052,0.4636,-0.012,-0.373,4.6267
5.045,-0.187,-0.340,4.6300,-0.101,0.1775,0.0184,0.4492,-0.012,-0.373,4.6267
5.145,-0.186,-0.333,4.6112,-0.026,0.1775,0.0246,0.3961,-0.012,-0.373,4.6267
5.245,-0.186,-0.333,4.6135,0.016,0.1775,0.0248,0.3572,-0.012,-0.373,4.6267
5.345,-0.186,-0.335,4.6208,0.017,0.1775,0.0229,0.3369,-0.012,-0.373,4.6267
5.445,-0.187,-0.336,4.6233,0.004,0.1775,0.0220,0.3343,-0.012,-0.373,4.6267
5.545,-0.187,-0.336,4.6228,0.000,0.1775,0.0219,0.3378,-0.012,-0.373,4.6267
5.645,-0.187,-0.336,4.6245,0.016,0.1775,0.0215,0.3335,-0.012,-0.373,4.6267
5.745,-0.187,-0.338,4.6303,0.006,0.1775,0.0197,0.3226,-0.012,-0.373,4.6267
5.845,-0.187,-0.339,4.6303,0.009,0.1775,0.0195,0.3269,-0.012,-0.373,4.6267
5.945,-0.187,-0.340,4.6348,0.019,0.1775,0.0181,0.3164,-0.012,-0.373,4.6267"""

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
wp_hits=[(1.347,0,0.1848),(2.247,1,0.2317),(3.246,2,0.1484),(4.146,3,0.2467)]

# Dark theme
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

# Stats
ey_active = ey[(t>0.8)&(t<4.2)]
ex_active = ex[(t>0.8)&(t<4.2)]
ey_rms=np.sqrt(np.mean(ey_active**2)); ex_rms=np.sqrt(np.mean(ex_active**2))
v_active = v[(t>0.8)&(t<4.2)]
min_ds=[h[2] for h in wp_hits]
fig.suptitle(f"LQR  wo=40  R=0.0012  Q=diag([1,1.8,0.32,0])  α=0.52  |  "
    f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v_active):.2f}m/s  "
    f"WP=[{min_ds[0]:.3f},{min_ds[1]:.3f},{min_ds[2]:.3f},{min_ds[3]:.3f}]m",
    fontsize=10,fontweight="bold",color=P,y=0.98)
plt.tight_layout(rect=[0,0,1,0.94])
out="tools/lqr_final_run_dashboard.png"
fig.savefig(out,dpi=150,facecolor=BG,edgecolor="none"); plt.close()
print(f"Saved: {out}")
print(f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v_active):.2f}m/s  v_max={np.max(v_active):.2f}m/s")
for _t,wi,md in wp_hits: print(f"  WP{wi} @t={_t:.2f}s  min_d={md:.4f}m")
