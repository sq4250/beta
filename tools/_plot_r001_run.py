import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

CSV = """0.013,-0.000,0.000,-0.0000,-0.002,0.0000,0.0003,-0.1277,0.000,-0.000,-0.0002
0.020,-0.000,0.000,-0.0005,0.001,0.0000,0.0011,-0.1876,0.001,-0.000,-0.0012
0.844,1.099,-0.586,-0.2706,2.664,0.2392,0.1191,0.3352,1.215,-0.348,-0.0296
0.944,1.360,-0.633,-0.0644,2.706,0.2814,0.1373,0.2724,1.441,-0.333,0.1579
1.044,1.624,-0.610,0.2301,2.571,0.3011,0.1521,0.3190,1.646,-0.277,0.3728
1.144,1.857,-0.522,0.5176,2.462,0.2954,0.1846,0.3055,1.819,-0.180,0.6433
1.244,2.037,-0.367,0.8905,2.308,0.2578,0.2223,0.3782,1.952,-0.043,0.9480
1.345,2.155,-0.166,1.1883,2.382,0.2237,0.2297,-0.3106,2.044,0.135,1.1519
1.444,2.238,0.066,1.1413,2.496,0.2056,0.1933,0.4139,2.134,0.321,1.1141
1.544,2.369,0.291,1.0443,3.115,0.2165,0.1942,0.1369,2.234,0.542,1.1897
1.644,2.477,0.585,1.3348,3.198,0.2085,0.1691,0.1813,2.313,0.788,1.3243
1.744,2.549,0.854,1.3186,2.982,0.2080,0.1766,0.2323,2.357,1.040,1.4687
1.844,2.579,1.139,1.5767,2.852,0.2080,0.1697,0.2898,2.361,1.284,1.6339
1.944,2.570,1.409,1.6975,2.748,0.2111,0.1683,0.1568,2.325,1.509,1.8192
2.044,2.498,1.658,1.9487,2.495,0.1971,0.1659,0.4593,2.251,1.707,2.0426
2.144,2.387,1.877,2.2221,2.405,0.1869,0.1746,0.1262,2.137,1.869,2.3256
2.246,2.205,2.024,2.6123,2.229,0.1394,0.1863,0.4226,1.977,1.990,2.6431
2.344,2.009,2.129,2.7138,2.371,0.1401,0.1715,-0.1766,1.802,2.077,2.6740
2.444,1.788,2.241,2.5885,2.703,0.1365,0.1612,0.3692,1.594,2.180,2.7018
2.544,1.539,2.383,2.7242,3.220,0.1588,0.1504,0.0145,1.358,2.275,2.8195
2.644,1.264,2.479,2.8269,2.667,0.1644,0.1392,0.3782,1.107,2.340,2.9553
2.744,0.998,2.556,2.9553,2.682,0.1925,0.1456,0.1326,0.857,2.367,3.1101
2.844,0.727,2.578,3.1212,2.687,0.2055,0.1483,0.3849,0.620,2.353,3.2840
2.944,0.464,2.560,3.3538,2.570,0.2246,0.1524,0.1479,0.405,2.300,3.4813
3.044,0.229,2.474,3.5867,2.425,0.2157,0.1632,0.4636,0.222,2.209,3.7297
3.144,0.035,2.345,3.9192,2.269,0.2020,0.1894,0.2389,0.078,2.077,4.0286
3.246,-0.096,2.156,4.2475,2.395,0.1636,0.2122,0.2538,-0.023,1.900,4.3273
3.344,-0.179,1.942,4.4221,2.587,0.1427,0.2261,0.2253,-0.079,1.703,4.5202
3.444,-0.233,1.673,4.5909,2.545,0.1364,0.2123,0.1553,-0.105,1.465,4.6684
3.544,-0.259,1.402,4.6207,2.770,0.1471,0.2072,0.1609,-0.104,1.211,4.7469
3.644,-0.281,1.115,4.6497,2.878,0.1788,0.1829,0.1337,-0.091,0.954,4.7725
3.744,-0.300,0.819,4.6559,2.981,0.2160,0.1399,0.0779,-0.074,0.707,4.7844
3.844,-0.308,0.524,4.7063,2.895,0.2456,0.0808,0.1461,-0.057,0.475,4.7939
3.944,-0.302,0.256,4.7857,2.498,0.2651,0.0345,0.0438,-0.036,0.264,4.8318
4.044,-0.274,0.036,4.8777,2.226,0.2649,-0.0040,-0.2305,-0.013,0.075,4.8106
4.145,-0.244,-0.140,4.8316,1.277,0.2418,-0.0362,0.0858,-0.003,-0.100,4.7316
4.244,-0.234,-0.279,4.7680,0.933,0.2284,-0.0603,-0.0919,-0.004,-0.222,4.6806
4.344,-0.229,-0.363,4.7672,1.124,0.2170,-0.0620,-0.0832,-0.009,-0.311,4.6433
4.444,-0.228,-0.398,4.7418,0.199,0.2109,-0.0552,0.0486,-0.013,-0.361,4.6226
4.544,-0.227,-0.405,4.7649,-0.327,0.2096,-0.0543,0.1887,-0.014,-0.372,4.6180
4.644,-0.227,-0.409,4.7964,0.255,0.2093,-0.0564,0.0902,-0.014,-0.372,4.6180
4.744,-0.228,-0.405,4.8766,-0.265,0.2099,-0.0541,0.1663,-0.014,-0.372,4.6180
4.844,-0.229,-0.397,4.8966,-0.079,0.2122,-0.0460,0.2989,-0.014,-0.372,4.6180
4.944,-0.232,-0.382,4.8778,-0.184,0.2161,-0.0317,0.4636,-0.014,-0.372,4.6180
5.044,-0.234,-0.363,4.8176,-0.148,0.2206,-0.0127,0.4636,-0.014,-0.372,4.6180
5.144,-0.236,-0.347,4.7517,-0.153,0.2235,0.0038,0.4636,-0.014,-0.372,4.6180
5.244,-0.236,-0.334,4.7060,-0.104,0.2249,0.0168,0.4636,-0.014,-0.372,4.6180
5.344,-0.236,-0.327,4.6761,-0.030,0.2255,0.0237,0.4625,-0.014,-0.372,4.6180
5.444,-0.236,-0.326,4.6767,0.010,0.2255,0.0244,0.3915,-0.014,-0.372,4.6180
5.544,-0.236,-0.328,4.6812,0.007,0.2254,0.0232,0.3572,-0.014,-0.372,4.6180
5.644,-0.236,-0.329,4.6846,0.023,0.2254,0.0223,0.3464,-0.014,-0.372,4.6180
5.744,-0.236,-0.330,4.6909,0.006,0.2252,0.0204,0.3306,-0.014,-0.372,4.6180
5.844,-0.236,-0.331,4.6910,0.000,0.2252,0.0202,0.3479,-0.014,-0.372,4.6180
5.944,-0.236,-0.331,4.6945,0.021,0.2252,0.0195,0.3354,-0.014,-0.372,4.6180
6.044,0.000,0.000,0.0000,0.000,0.0000,0.0000,0.0000,0.000,0.000,0.0000"""

t,x,y,th,v,ey,ex,delta,vx,vy,vth=[],[],[],[],[],[],[],[],[],[],[]
for l in CSV.strip().split('\n'):
    p=l.split(',')
    if float(p[0])==0 and float(p[4])==0: continue
    t.append(float(p[0])); x.append(float(p[1])); y.append(float(p[2]))
    th.append(float(p[3])); v.append(float(p[4]))
    ey.append(float(p[5])); ex.append(float(p[6])); delta.append(float(p[7]))
    vx.append(float(p[8])); vy.append(float(p[9])); vth.append(float(p[10]))

t=np.array(t); x=np.array(x); y=np.array(y); th=np.array(th); v=np.array(v)
ey=np.array(ey); ex=np.array(ex); delta=np.array(delta)
vx=np.array(vx); vy=np.array(vy); vth=np.array(vth)
wp=np.array([[2,0],[2,2],[0,2],[0,0]])
hits=[(1.345,0,0.2280),(2.246,1,0.2101),(3.246,2,0.1852),(4.145,3,0.2767)]
mask=(t>0.8)&(t<4.2)
ey_rms=np.sqrt(np.mean(ey[mask]**2)); ex_rms=np.sqrt(np.mean(ex[mask]**2))

BG='#1a1a19'; SURF='#22221f'; P='#ffffff'; S='#c3c2b7'; M='#898781'; G='#2c2c2a'
BLUE='#3987e5'; AQUA='#199e70'; RED='#e66767'; ORANGE='#d95926'; YELLOW='#c98500'
plt.rcParams.update({'figure.facecolor':BG,'axes.facecolor':SURF,'axes.edgecolor':G,
    'axes.labelcolor':S,'text.color':P,'xtick.color':M,'ytick.color':M,
    'grid.color':G,'grid.alpha':0.4,'legend.facecolor':SURF,'legend.edgecolor':G,
    'legend.labelcolor':S,'font.size':8,'axes.titlesize':10,'axes.labelsize':8})
fig,axes=plt.subplots(2,3,figsize=(16,9)); fig.patch.set_facecolor(BG)

ax=axes[0,0]; ax.plot(x,y,color=RED,lw=1.5,label='Real'); ax.plot(vx,vy,color=BLUE,lw=1.0,alpha=0.6,ls='--',label='Virtual')
ax.scatter(wp[:,0],wp[:,1],c=YELLOW,s=60,marker='s',zorder=5,edgecolors='none',label='WP')
for i,(tx,ty) in enumerate(wp): ax.annotate(f'WP{i}',(tx,ty),textcoords='offset points',xytext=(8,8),color=YELLOW,fontsize=8,fontweight='bold')
for _,wpi,_ in hits: ax.add_patch(plt.Circle(wp[wpi],0.05,fill=False,edgecolor=AQUA,lw=1.0,ls=':',alpha=0.7))
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_title('Trajectory',fontweight='bold'); ax.set_aspect('equal'); ax.legend(fontsize=7,loc='lower left'); ax.grid(True,alpha=0.3)

ax=axes[0,1]; ax.fill_between(t,0,ey,color=BLUE,alpha=0.15); ax.plot(t,ey,color=BLUE,lw=1.5); ax.axhline(0,color=M,lw=0.8,ls='--')
ax.set_xlabel('t [s]'); ax.set_ylabel('ey [m]'); ax.set_title('Lateral Error',fontweight='bold'); ax.grid(True,alpha=0.3)

ax=axes[0,2]; ax.fill_between(t,0,ex,color=AQUA,alpha=0.12); ax.plot(t,ex,color=AQUA,lw=1.5); ax.axhline(0,color=M,lw=0.8,ls='--')
ax.set_xlabel('t [s]'); ax.set_ylabel('ex [m]'); ax.set_title('Longitudinal Error',fontweight='bold'); ax.grid(True,alpha=0.3)

ax=axes[1,0]; ax.plot(t,v,color=ORANGE,lw=1.8); ax.axhline(3.5,color=RED,lw=0.8,ls='--',alpha=0.5,label='v_max=3.5')
ax.set_xlabel('t [s]'); ax.set_ylabel('v [m/s]'); ax.set_title('Speed',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,1]; ax.plot(t,np.rad2deg(delta),color=YELLOW,lw=1.5)
ax.axhline(26.5,color=RED,lw=0.8,ls='--',alpha=0.5,label=r'$\delta_{max}$=26.5'); ax.axhline(-26.5,color=RED,lw=0.8,ls='--',alpha=0.5)
ax.set_xlabel('t [s]'); ax.set_ylabel(r'$\delta$ [deg]'); ax.set_title('Steering Angle',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

ax=axes[1,2]; ax.plot(t,np.rad2deg(th),color=RED,lw=1.2,label='Real'); ax.plot(t,np.rad2deg(vth),color=BLUE,lw=0.8,alpha=0.5,ls='--',label='Virtual')
ax.set_xlabel('t [s]'); ax.set_ylabel(r'$\theta$ [deg]'); ax.set_title('Heading',fontweight='bold'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)

md=[h[2] for h in hits]
fig.suptitle(f'LQR  wo=30  R=0.001  Q=diag([1,0.8,0.28,0])  a=0.55  (极限ey增益)  |  eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v[mask]):.2f}m/s  WP=[{md[0]:.3f},{md[1]:.3f},{md[2]:.3f},{md[3]:.3f}]m',fontsize=10,fontweight='bold',color=P,y=0.98)
plt.tight_layout(rect=[0,0,1,0.94])
fig.savefig('tools/lqr_r001_run_dashboard.png',dpi=150,facecolor=BG,edgecolor='none'); plt.close()
print(f'eyRMS={ey_rms:.4f}m  exRMS={ex_rms:.4f}m  v_avg={np.mean(v[mask]):.2f}m/s  v_max={np.max(v[mask]):.2f}m/s')
for _t,wi,md in hits: print(f'  WP{wi} @t={_t:.2f}s  min_d={md:.4f}m')
