"""
_plot_wo30_run.py — LQR wo=30 超高增益+高阻尼 运行数据可视化
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV = """0.013,-0.000,-0.000,0.0000,0.004,0.0000,0.0003,-0.1242,0.000,-0.000,-0.0002
0.020,0.000,-0.000,-0.0001,0.022,0.0000,0.0009,-0.1826,0.001,-0.000,-0.0012
0.844,1.201,-0.463,-0.1074,2.407,0.1147,0.0204,0.1366,1.215,-0.348,-0.0296
0.944,1.435,-0.467,0.0561,2.304,0.1303,0.0365,0.2898,1.441,-0.333,0.1579
1.044,1.652,-0.430,0.3013,2.131,0.1445,0.0586,0.2237,1.646,-0.277,0.3728
1.144,1.842,-0.343,0.5501,2.029,0.1434,0.0881,0.3623,1.819,-0.180,0.6433
1.244,1.990,-0.209,0.9175,2.004,0.1275,0.1203,0.2862,1.952,-0.043,0.9480
1.345,2.094,-0.016,1.2032,2.542,0.1053,0.1188,-0.1933,2.059,0.139,1.0663
1.444,2.185,0.208,1.1212,2.529,0.0769,0.1090,0.2090,2.161,0.330,1.1042
1.544,2.298,0.455,1.2043,2.840,0.0706,0.0987,0.0928,2.261,0.562,1.2228
1.644,2.385,0.730,1.3004,2.902,0.0677,0.0828,0.1931,2.333,0.814,1.3576
1.744,2.445,1.006,1.4312,2.751,0.0800,0.0682,0.1369,2.369,1.069,1.5010
1.844,2.461,1.272,1.5859,2.578,0.0900,0.0609,0.2297,2.365,1.313,1.6676
1.944,2.436,1.513,1.7816,2.260,0.1029,0.0643,0.1805,2.322,1.537,1.8552
2.044,2.365,1.725,1.9931,2.191,0.1058,0.0749,0.3487,2.240,1.731,2.0847
2.144,2.256,1.904,2.2721,2.102,0.1076,0.0947,0.2933,2.119,1.886,2.3719
2.246,2.096,2.034,2.6387,2.023,0.0933,0.1146,0.1973,1.954,2.000,2.6826
2.344,1.892,2.110,2.8786,2.245,0.0768,0.1010,-0.2589,1.777,2.081,2.7030
2.444,1.660,2.199,2.6911,2.497,0.0552,0.0874,0.2192,1.567,2.181,2.7147
2.544,1.419,2.310,2.7837,2.785,0.0622,0.0840,0.0530,1.330,2.273,2.8271
2.644,1.147,2.376,3.0024,2.775,0.0526,0.0706,0.0695,1.080,2.335,2.9627
2.744,0.883,2.416,3.0027,2.598,0.0554,0.0605,0.2585,0.831,2.360,3.1189
2.844,0.632,2.417,3.2598,2.451,0.0665,0.0554,0.1311,0.597,2.344,3.2941
2.944,0.399,2.377,3.3766,2.268,0.0762,0.0522,0.3149,0.385,2.290,3.4962
3.044,0.198,2.299,3.6689,2.068,0.0885,0.0619,0.2788,0.206,2.195,3.7535
3.144,0.040,2.171,3.9688,2.012,0.0874,0.0789,0.3457,0.067,2.061,4.0561
3.246,-0.073,1.997,4.3123,2.181,0.0809,0.0940,0.1180,-0.029,1.882,4.3523
3.344,-0.131,1.776,4.5512,2.279,0.0649,0.0908,0.1493,-0.081,1.685,4.5391
3.444,-0.158,1.528,4.6497,2.557,0.0573,0.0896,0.1033,-0.103,1.447,4.6783
3.544,-0.160,1.261,4.7386,2.725,0.0563,0.0805,0.0580,-0.101,1.194,4.7500
3.644,-0.151,0.983,4.7434,2.766,0.0607,0.0603,0.0562,-0.088,0.938,4.7727
3.744,-0.144,0.714,4.7410,2.576,0.0703,0.0382,0.0447,-0.071,0.692,4.7830
3.844,-0.137,0.468,4.7406,2.302,0.0812,0.0230,0.1020,-0.054,0.461,4.7944
3.944,-0.128,0.250,4.7693,2.093,0.0945,0.0187,0.0865,-0.033,0.251,4.8316
4.044,-0.112,0.057,4.8179,1.879,0.1015,0.0099,-0.1820,-0.011,0.063,4.8021
4.145,-0.097,-0.104,4.7401,1.483,0.0909,-0.0034,-0.0549,-0.006,-0.103,4.6809
4.244,-0.103,-0.218,4.5862,0.822,0.0873,-0.0096,0.0046,-0.015,-0.215,4.5986
4.344,-0.114,-0.288,4.5141,0.007,0.0880,-0.0089,-0.0220,-0.026,-0.294,4.5404
4.444,-0.121,-0.320,4.4657,-0.539,0.0888,-0.0063,0.0791,-0.034,-0.333,4.5115
4.544,-0.123,-0.326,4.4596,-0.371,0.0891,-0.0072,0.1763,-0.035,-0.338,4.5080
4.644,-0.124,-0.331,4.5016,0.061,0.0890,-0.0114,0.1142,-0.035,-0.338,4.5080
4.744,-0.124,-0.331,4.5269,0.139,0.0890,-0.0106,0.0781,-0.035,-0.338,4.5080
4.844,-0.124,-0.333,4.5361,-0.000,0.0889,-0.0132,0.0706,-0.035,-0.338,4.5080
4.944,-0.124,-0.329,4.5331,-0.051,0.0890,-0.0100,0.0987,-0.035,-0.338,4.5080
5.044,-0.123,-0.326,4.5311,-0.016,0.0891,-0.0064,0.1090,-0.035,-0.338,4.5080
5.144,-0.123,-0.324,4.5299,-0.016,0.0891,-0.0049,0.1085,-0.035,-0.338,4.5080
5.244,-0.123,-0.323,4.5284,-0.018,0.0892,-0.0032,0.1104,-0.035,-0.338,4.5080
5.344,-0.122,-0.322,4.5266,-0.001,0.0892,-0.0022,0.1103,-0.035,-0.338,4.5080
5.444,-0.122,-0.321,4.5268,-0.016,0.0892,-0.0011,0.1089,-0.035,-0.338,4.5080
5.544,-0.122,-0.320,4.5265,-0.000,0.0892,-0.0005,0.1079,-0.035,-0.338,4.5080
5.644,-0.122,-0.320,4.5263,-0.001,0.0892,-0.0003,0.1066,-0.035,-0.338,4.5080
5.744,-0.122,-0.320,4.5258,-0.000,0.0892,-0.0003,0.1066,-0.035,-0.338,4.5080
5.844,-0.122,-0.320,4.5269,-0.000,0.0892,-0.0003,0.1042,-0.035,-0.338,4.5080"""

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
wp_hits=[(1.345,0,0.0945),(2.246,1,0.1050),(3.246,2,0.0708),(4.145,3,0.1254)]

BG="#1a1a19"; SURF="#22221f"; P="#ffffff"; S="#c3c2b7"; M="#898781"; G="#2c2c2a"
BLUE="#3987e5"; AQUA="#199e70"; RED="#e66767"; ORANGE="#d95926"; YELLOW="#c98500"

plt.rcParams.update({"figure.facecolor":BG,"axes.facecolor":SURF,"axes.edgecolor":G,
    "axes.labelcolor":S,"text.color":P,"xtick.color":M,"ytick.color":M,
    "grid.color":G,"grid.alpha":0.4,"legend.facecolor":SURF,"legend.edgecolor":G,
    "legend.labelcolor":S,"font.size":8,"axes.titlesize":10,"axes.labelsize":8})

fig,axes=plt.subplots(2,3,figsize=(16,9)); fig.patch.set_facecolor(BG)

ax=axes[0,0]
ax.plot(x,y,color=RED,lw=1.5,label="Real"); ax.plot(vx,vy,color=BLUE,lw=1.0,alpha=0.6,ls="--",label="Virtual")
ax.scatter(wp[:,0],wp[:,1],c=YELLOW,s=60,marker="s",zorder=5,edgecolors="none",label="WP")
for i,(tx,ty) in enumerate(wp): ax.annotate(f"WP{i}",(tx,ty),textcoords="offset points",xytext=(8,8),color=YELLOW,fontsize=8,fontweight="bold")
for _,wpi,_ in wp_hits: ax.add_patch(plt.Circle(wp[wpi],0.05,fill=False,edgecolor=AQUA,lw=1.0,ls=":",alpha=0.7))
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_title("Trajectory",fontweight="bold")
ax.set_aspect("equal"); ax.legend(fontsize=7,loc="lower left"); ax.grid(True,alpha=0.3)

ax=axes[0,1]
ax.fill_between(t,0,ey,color=BLUE,alpha=0.15); ax.plot(t,ey,color=BLUE,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls="--"); pk=np.argmax(np.abs(ey))
ax.annotate(f"{ey[pk]:.3f}m",(t[pk],ey[pk]),textcoords="offset points",xytext=(0,12),color=BLUE,fontsize=7,ha="center")
ax.set_xlabel("t [s]"); ax.set_ylabel("ey [m]"); ax.set_title("Lateral Error",fontweight="bold"); ax.grid(True,alpha=0.3)

ax=axes[0,2]
ax.fill_between(t,0,ex,color=AQUA,alpha=0.12); ax.plot(t,ex,color=AQUA,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls="--")
ax.set_xlabel("t [s]"); ax.set_ylabel("ex [m]"); ax.set_title("Longitudinal Error",fontweight="bold"); ax.grid(True,alpha=0.3)

ax=axes[1,0]
ax.plot(t,v,color=ORANGE,lw=1.8); ax.axhline(3.5,color=RED,lw=0.8,ls="--",alpha=0.5,label="v_max=3.5")
ax.set_xlabel("t [s]"); ax.set_ylabel("v [m/s]"); ax.set_title("Speed",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,1]
ax.plot(t,np.rad2deg(delta),color=YELLOW,lw=1.5)
ax.axhline(26.5,color=RED,lw=0.8,ls="--",alpha=0.5,label=r"$\delta_{max}$=26.5°"); ax.axhline(-26.5,color=RED,lw=0.8,ls="--",alpha=0.5)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\delta$ [°]"); ax.set_title("Steering Angle",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,2]
ax.plot(t,np.rad2deg(th),color=RED,lw=1.2,label="Real")
ax.plot(t,np.rad2deg(vth),color=BLUE,lw=0.8,alpha=0.5,ls="--",label="Virtual")
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\theta$ [°]"); ax.set_title("Heading",fontweight="bold")
ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ey_rms=np.sqrt(np.mean(ey**2)); ex_rms=np.sqrt(np.mean(ex**2))
min_ds=[h[2] for h in wp_hits]
fig.suptitle(f"LQR  wo=30  R=0.002  Q=diag([1,1.2,0.22,0])  α=0.60  (ultra-high-gain)  |  "
    f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  "
    f"WP min_d=[{min_ds[0]:.3f},{min_ds[1]:.3f},{min_ds[2]:.3f},{min_ds[3]:.3f}]m",
    fontsize=11,fontweight="bold",color=P,y=0.98)
plt.tight_layout(rect=[0,0,1,0.94])
out="tools/lqr_wo30_run_dashboard.png"
fig.savefig(out,dpi=150,facecolor=BG,edgecolor="none"); plt.close()
print(f"Saved: {out}")
print(f"eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v):.2f}m/s")
for _t,wi,md in wp_hits: print(f"  WP{wi} @t={_t:.2f}s  min_d={md:.4f}m")
