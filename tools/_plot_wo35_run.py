import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

CSV = """0.013,0.000,-0.000,-0.0000,0.017,-0.0000,-0.0000,-0.1264,0.000,-0.000,-0.0002
0.020,0.001,-0.000,-0.0003,0.024,-0.0000,0.0005,-0.1859,0.001,-0.000,-0.0012
0.844,1.107,-0.575,-0.2498,2.621,0.2282,0.1112,0.2900,1.215,-0.348,-0.0296
0.944,1.369,-0.612,-0.0249,2.607,0.2625,0.1247,0.2862,1.441,-0.333,0.1579
1.044,1.627,-0.583,0.2635,2.521,0.2771,0.1390,0.2728,1.646,-0.277,0.3733
1.144,1.856,-0.484,0.5406,2.424,0.2642,0.1634,0.4030,1.819,-0.180,0.6436
1.244,2.033,-0.326,0.9514,2.277,0.2309,0.1911,0.1992,1.952,-0.043,0.9481
1.346,2.138,-0.115,1.2083,2.301,0.1874,0.1935,-0.1882,2.044,0.135,1.1539
1.444,2.215,0.102,1.1757,2.433,0.1708,0.1737,0.2374,2.132,0.323,1.1249
1.544,2.334,0.328,1.0707,2.728,0.1744,0.1741,0.2131,2.231,0.545,1.1923
1.644,2.445,0.605,1.2887,2.738,0.1762,0.1583,0.2049,2.309,0.791,1.3246
1.744,2.528,0.880,1.3128,2.866,0.1888,0.1563,0.2282,2.353,1.043,1.4691
1.844,2.566,1.164,1.5366,2.995,0.1992,0.1485,0.2291,2.358,1.287,1.6341
1.944,2.552,1.440,1.7241,2.652,0.2046,0.1372,0.2263,2.322,1.513,1.8193
2.044,2.477,1.685,2.0011,2.474,0.1919,0.1368,0.3450,2.247,1.710,2.0441
2.144,2.360,1.890,2.2354,2.246,0.1770,0.1509,0.2633,2.133,1.871,2.3282
2.246,2.189,2.033,2.6036,2.108,0.1388,0.1715,0.3661,1.973,1.992,2.6430
2.344,1.983,2.132,2.7816,2.321,0.1320,0.1493,-0.2074,1.799,2.079,2.6736
2.444,1.772,2.228,2.6042,2.396,0.1176,0.1532,0.3946,1.592,2.182,2.7018
2.544,1.538,2.365,2.7001,2.656,0.1410,0.1544,0.0280,1.357,2.276,2.8192
2.644,1.275,2.460,2.8163,2.700,0.1471,0.1553,0.3718,1.105,2.341,2.9551
2.744,1.008,2.543,2.9387,2.956,0.1777,0.1585,0.1224,0.856,2.368,3.1105
2.844,0.730,2.561,3.1489,2.738,0.1877,0.1504,0.3832,0.619,2.354,3.2851
2.944,0.464,2.542,3.3557,2.640,0.2055,0.1472,0.1480,0.404,2.301,3.4821
3.044,0.232,2.447,3.6375,2.426,0.1907,0.1504,0.4636,0.221,2.209,3.7311
3.144,0.037,2.313,3.9106,2.280,0.1796,0.1664,0.2732,0.078,2.077,4.0301
3.246,-0.091,2.122,4.3015,2.349,0.1461,0.1824,0.1739,-0.023,1.900,4.3287
3.344,-0.165,1.905,4.4352,2.288,0.1227,0.1908,0.2677,-0.079,1.703,4.5212
3.444,-0.217,1.649,4.5941,2.729,0.1198,0.1888,0.1302,-0.104,1.465,4.6678
3.544,-0.241,1.379,4.6247,2.987,0.1304,0.1837,0.1632,-0.104,1.211,4.7459
3.644,-0.260,1.091,4.6744,2.971,0.1591,0.1570,0.0919,-0.091,0.955,4.7707
3.744,-0.276,0.802,4.6466,2.878,0.1924,0.1203,0.1254,-0.075,0.708,4.7831
3.844,-0.282,0.518,4.7265,2.796,0.2192,0.0718,0.1249,-0.058,0.476,4.7939
3.944,-0.274,0.257,4.7752,2.234,0.2363,0.0308,0.0943,-0.036,0.264,4.8336
4.044,-0.247,0.040,4.8770,1.871,0.2373,-0.0039,-0.1890,-0.013,0.075,4.8139
4.145,-0.217,-0.145,4.8446,1.671,0.2149,-0.0407,-0.0320,-0.003,-0.099,4.7352
4.244,-0.204,-0.272,4.7773,0.315,0.1993,-0.0546,-0.0118,-0.003,-0.221,4.6843
4.344,-0.201,-0.362,4.7267,0.694,0.1896,-0.0608,-0.0316,-0.007,-0.311,4.6469
4.444,-0.201,-0.401,4.6963,-0.273,0.1850,-0.0580,0.0486,-0.011,-0.361,4.6262
4.544,-0.201,-0.413,4.6872,0.323,0.1841,-0.0573,0.1369,-0.012,-0.372,4.6216
4.644,-0.201,-0.407,4.7220,-0.222,0.1845,-0.0530,0.2362,-0.012,-0.372,4.6216
4.744,-0.201,-0.406,4.7744,-0.019,0.1846,-0.0514,0.1446,-0.012,-0.372,4.6216
4.844,-0.202,-0.393,4.7857,-0.052,0.1866,-0.0390,0.3719,-0.012,-0.372,4.6216
4.944,-0.202,-0.383,4.7661,-0.154,0.1881,-0.0293,0.4636,-0.012,-0.372,4.6216
5.044,-0.203,-0.366,4.7214,-0.167,0.1903,-0.0128,0.4636,-0.012,-0.372,4.6216
5.144,-0.203,-0.351,4.6685,-0.132,0.1914,0.0029,0.4636,-0.012,-0.372,4.6216
5.244,-0.202,-0.342,4.6372,-0.060,0.1917,0.0124,0.4594,-0.012,-0.372,4.6216
5.344,-0.202,-0.338,4.6263,-0.013,0.1918,0.0158,0.4253,-0.012,-0.372,4.6216"""

t,x,y,th,v,ey,ex,delta,vx,vy,vth=[],[],[],[],[],[],[],[],[],[],[]
for l in CSV.strip().split('\n'):
    p=l.split(',')
    t.append(float(p[0])); x.append(float(p[1])); y.append(float(p[2]))
    th.append(float(p[3])); v.append(float(p[4]))
    ey.append(float(p[5])); ex.append(float(p[6])); delta.append(float(p[7]))
    vx.append(float(p[8])); vy.append(float(p[9])); vth.append(float(p[10]))

t=np.array(t); x=np.array(x); y=np.array(y); th=np.array(th); v=np.array(v)
ey=np.array(ey); ex=np.array(ex); delta=np.array(delta)
vx=np.array(vx); vy=np.array(vy); vth=np.array(vth)
wp=np.array([[2,0],[2,2],[0,2],[0,0]])
hits=[(1.346,0,0.1813),(2.246,1,0.1947),(3.246,2,0.1541),(4.145,3,0.2505)]

BG='#1a1a19'; SURF='#22221f'; P='#ffffff'; S='#c3c2b7'; M='#898781'; G='#2c2c2a'
BLUE='#3987e5'; AQUA='#199e70'; RED='#e66767'; ORANGE='#d95926'; YELLOW='#c98500'
plt.rcParams.update({'figure.facecolor':BG,'axes.facecolor':SURF,'axes.edgecolor':G,
    'axes.labelcolor':S,'text.color':P,'xtick.color':M,'ytick.color':M,
    'grid.color':G,'grid.alpha':0.4,'legend.facecolor':SURF,'legend.edgecolor':G,
    'legend.labelcolor':S,'font.size':8,'axes.titlesize':10,'axes.labelsize':8})

fig,axes=plt.subplots(2,3,figsize=(16,9)); fig.patch.set_facecolor(BG)

ax=axes[0,0]
ax.plot(x,y,color=RED,lw=1.5,label='Real')
ax.plot(vx,vy,color=BLUE,lw=1.0,alpha=0.6,ls='--',label='Virtual')
ax.scatter(wp[:,0],wp[:,1],c=YELLOW,s=60,marker='s',zorder=5,edgecolors='none',label='WP')
for i,(tx,ty) in enumerate(wp):
    ax.annotate(f'WP{i}',(tx,ty),textcoords='offset points',xytext=(8,8),
                color=YELLOW,fontsize=8,fontweight='bold')
for _,wpi,_ in hits:
    ax.add_patch(plt.Circle(wp[wpi],0.05,fill=False,edgecolor=AQUA,lw=1.0,ls=':',alpha=0.7))
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title('Trajectory',fontweight='bold')
ax.set_aspect('equal'); ax.legend(fontsize=7,loc='lower left'); ax.grid(True,alpha=0.3)

ax=axes[0,1]
ax.fill_between(t,0,ey,color=BLUE,alpha=0.15); ax.plot(t,ey,color=BLUE,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls='--')
ax.set_xlabel('t [s]'); ax.set_ylabel('ey [m]')
ax.set_title('Lateral Error',fontweight='bold'); ax.grid(True,alpha=0.3)

ax=axes[0,2]
ax.fill_between(t,0,ex,color=AQUA,alpha=0.12); ax.plot(t,ex,color=AQUA,lw=1.5)
ax.axhline(0,color=M,lw=0.8,ls='--')
ax.set_xlabel('t [s]'); ax.set_ylabel('ex [m]')
ax.set_title('Longitudinal Error',fontweight='bold'); ax.grid(True,alpha=0.3)

ax=axes[1,0]
ax.plot(t,v,color=ORANGE,lw=1.8)
ax.axhline(3.5,color=RED,lw=0.8,ls='--',alpha=0.5,label='v_max=3.5')
ax.set_xlabel('t [s]'); ax.set_ylabel('v [m/s]')
ax.set_title('Speed',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,1]
ax.plot(t,np.rad2deg(delta),color=YELLOW,lw=1.5)
ax.axhline(26.5,color=RED,lw=0.8,ls='--',alpha=0.5,label=r'$\delta_{max}$=26.5')
ax.axhline(-26.5,color=RED,lw=0.8,ls='--',alpha=0.5)
ax.set_xlabel('t [s]'); ax.set_ylabel(r'$\delta$ [deg]')
ax.set_title('Steering Angle',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,2]
ax.plot(t,np.rad2deg(th),color=RED,lw=1.2,label='Real')
ax.plot(t,np.rad2deg(vth),color=BLUE,lw=0.8,alpha=0.5,ls='--',label='Virtual')
ax.set_xlabel('t [s]'); ax.set_ylabel(r'$\theta$ [deg]')
ax.set_title('Heading',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

mask=(t>0.8)&(t<4.2)
ey_rms=np.sqrt(np.mean(ey[mask]**2))
ex_rms=np.sqrt(np.mean(ex[mask]**2))
md=[h[2] for h in hits]
fig.suptitle(
    f'LQR  wo=30 R=.001  R=0.0012  Q=diag([1,1.8,0.30,0])  a=0.52  |  '
    f'eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v[mask]):.2f}m/s  '
    f'WP=[{md[0]:.3f},{md[1]:.3f},{md[2]:.3f},{md[3]:.3f}]m',
    fontsize=10,fontweight='bold',color=P,y=0.98)
plt.tight_layout(rect=[0,0,1,0.94])
fig.savefig('tools/lqr_wo35_run_dashboard.png',dpi=150,facecolor=BG,edgecolor='none')
plt.close()
print(f'eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v[mask]):.2f}m/s  v_max={np.max(v[mask]):.2f}m/s')
for _t,wi,md in hits: print(f'  WP{wi} @t={_t:.2f}s  min_d={md:.4f}m')
