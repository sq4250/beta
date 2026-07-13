"""
sim_vs_real.py — 理想仿真 vs 实车数据对比
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.linalg import solve_continuous_are

PROJECT = Path(__file__).resolve().parents[1]
import sys; sys.path.insert(0, str(PROJECT/'tools'))
from bicycle_model import BicycleParams, check_hit_substep

# ===== 参数 (与C代码完全一致) =====
p = BicycleParams(); L = p.wheelbase
DT = 1/200; DT_OUTER = 0.05; SUB = 10; TOL = 0.05
Q_HGD = 1.0; R_OMG = 0.10; WO_LQR = 35; ALPHA_LQR = 0.3
ALPHA = 1.5; B0 = 30.0; WO = 15; KP = 16; KD = 8
WP = np.array([[2,0],[2,2],[0,2],[0,0]], dtype=np.float32)

# ===== NN模型 =====
npz = np.load(str(PROJECT/'tools'/'micro_deploy.npz'))
W1=npz['fc1_weight'].astype(np.float32); b1=npz['fc1_bias'].astype(np.float32)
W2=npz['fc2_weight'].astype(np.float32); b2=npz['fc2_bias'].astype(np.float32)
W3=npz['fc3_weight'].astype(np.float32); b3=npz['fc3_bias'].astype(np.float32)

def nn(r):
    x=np.maximum(0,r@W1.T+b1); x=np.maximum(0,x@W2.T+b2); r=x@W3.T+b3
    return np.array([np.clip(r[0],-4,4),np.clip(r[1],-14,14)],dtype=np.float32)

def bf8(s,g1,g2,g3):
    x,y,th,v,d=s; ct,st=np.cos(th),np.sin(th)
    def bf(g): dx,dy=g[0]-x,g[1]-y; return dx*ct+dy*st,-dx*st+dy*ct
    dx1,dy1=bf(g1);dx2,dy2=bf(g2);dx3,dy3=bf(g3)
    return np.array([v,d,dx1,dy1,dx2,dy2,dx3,dy3],dtype=np.float32)

def mcu_step(s,a):
    x,y,th,v,d=s
    a0=np.clip(a[0],-4,4); o=np.clip(a[1],-14,14)
    vn=np.clip(v+a0*DT,0,3.5); dn=np.clip(d+o*DT,-p.delta_max,p.delta_max)
    xn=x+v*np.cos(th)*DT; yn=y+v*np.sin(th)*DT
    thn=np.arctan2(np.sin(th+v*np.tan(d)/L*DT),np.cos(th+v*np.tan(d)/L*DT))
    return np.array([xn,yn,thn,vn,dn],dtype=np.float32)

# ===== LQR增益表 =====
SP_BP = np.array([0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0])
lr = np.zeros((len(SP_BP),4))
for i,v in enumerate(SP_BP):
    A=np.array([[0,v,0,0],[0,0,1,0],[0,0,-WO_LQR,WO_LQR*v/L],[0,0,0,0]])
    B=np.array([[0.],[0.],[0.],[1.]])
    lr[i]=(np.linalg.inv(np.array([[R_OMG]]))@B.T@solve_continuous_are(A,B,np.diag([1.,Q_HGD,0.,0.]),np.array([[R_OMG]]))).flatten()

def lqr5g(v,ey,eyd,eth,ethd,ed):
    vc=np.clip(abs(v),0.1,4.0); K=np.array([np.interp(vc,SP_BP,lr[:,j]) for j in range(4)])
    g=np.array([K[0],K[1]*ALPHA_LQR/vc,K[1]*(1-ALPHA_LQR),K[2],K[3]])
    return g@[ey,eyd,eth,ethd,ed]

# ===== 主仿真 =====
vst = np.zeros(5,dtype=np.float32); rs  = np.zeros(5,dtype=np.float32)
wpi=0; reached=np.zeros(len(WP),bool); nn_a=nn_omg=0.; z1=0.; fh=0.; up=0.
vst_prev_seg = vst[:2].copy()
sim_log = []

for ctrl in range(int(10/DT_OUTER)):
    t=ctrl*DT_OUTER
    if wpi<len(WP) and not reached[wpi] and check_hit_substep(vst_prev_seg, vst[:2], WP[wpi], TOL):
        reached[wpi]=True; wpi+=1
    while wpi<len(WP) and reached[wpi]: wpi+=1
    if wpi>=len(WP): break
    ci=wpi; Nw=len(WP); ni=min(ci+1,Nw-1); n2i=min(ci+2,Nw-1)
    inp=bf8(vst,WP[ci],WP[ni],WP[n2i]); nn_a,nn_omg=nn(inp)
    vst_prev_seg = vst[:2].copy()

    for sc in range(SUB):
        vst=mcu_step(vst,np.array([nn_a,nn_omg]))
        ct=np.cos(vst[2]); st_=np.sin(vst[2])
        ex=(vst[0]-rs[0])*ct+(vst[1]-rs[1])*st_
        ey=(rs[0]-vst[0])*st_-(rs[1]-vst[1])*ct

        vm=rs[3]; eo=vm-z1
        z1 += (-ALPHA*z1 + B0*up + fh + 2.0*WO*eo) * DT
        fh += (WO*WO*eo) * DT
        u0 = KP*ex + KD*(vst[3]-z1) + nn_a + ALPHA*z1 - fh
        thr = u0 / B0; thr = np.clip(thr,-1,1); up = thr

        rs[3] += (-ALPHA*rs[3] + B0*thr) * DT
        rs[3] = np.clip(rs[3],0,3.5)

        eth=(vst[2]-rs[2]+np.pi)%(2*np.pi)-np.pi; ed=vst[4]-rs[4]
        eyd=rs[3]*np.sin(eth); ethd=vst[3]*np.tan(vst[4])/L-rs[3]*np.tan(rs[4])/L
        omg_fb=lqr5g(vst[3],ey,eyd,eth,ethd,ed); omg_cmd=nn_omg+omg_fb

        rs[4] += omg_cmd*DT; rs[4]=np.clip(rs[4],-p.delta_max,p.delta_max)
        rs[2] += rs[3]*np.tan(rs[4])/L*DT
        rs[0] += rs[3]*np.cos(rs[2])*DT
        rs[1] += rs[3]*np.sin(rs[2])*DT
        rs[2] = np.arctan2(np.sin(rs[2]), np.cos(rs[2]))

        if sc % 2 == 0:
            sim_log.append([t + sc*DT, rs[0], rs[1], rs[2], rs[3], ey, ex, rs[4], vst[0], vst[1], vst[2]])

sim_log = np.array(sim_log)
print(f'Sim: eyRMS={np.sqrt(np.mean(sim_log[:,5]**2))*100:.1f}cm  exRMS={np.sqrt(np.mean(sim_log[:,6]**2))*100:.1f}cm')

# ===== 实车数据 =====
real_raw = """0.844,1.197,-0.461,-0.1079,2.409,0.1134,0.0244,0.1764,1.215,-0.347,-0.0296
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
real_lines = [l.strip().split(",") for l in real_raw.strip().split("\n")]
real_data = np.array([[float(x) for x in l] for l in real_lines])
print(f'Real: eyRMS={np.sqrt(np.mean(real_data[:,5]**2))*100:.1f}cm  exRMS={np.sqrt(np.mean(real_data[:,6]**2))*100:.1f}cm')

# ===== 对比图 =====
fig, axes = plt.subplots(2, 3, figsize=(18, 11))
params_str = "Qth=%.1f  Rw=%.2f  wo=%d  a=%.1f" % (Q_HGD, R_OMG, WO_LQR, ALPHA_LQR)
fig.suptitle("Simulation vs Real Car | %s | sim eyRMS=%.0fcm  real eyRMS=%.0fcm" % (
    params_str, np.sqrt(np.mean(sim_log[:,5]**2))*100, np.sqrt(np.mean(real_data[:,5]**2))*100),
    fontsize=12, fontweight="bold")

titles = ["Trajectory XY", "Lateral Error ey [cm]", "Longitudinal Error ex [cm]",
          "Speed [m/s]", "Steering delta [rad]", "Heading theta [rad]"]

for idx, ax in enumerate(axes.flat):
    if idx == 0:
        ax.plot(WP[:,0], WP[:,1], "s-", color="gray", ms=10, alpha=0.3, lw=2, label="waypoints")
        for i, (wx, wy) in enumerate(WP):
            ax.annotate("WP%d" % i, (wx, wy), textcoords="offset points", xytext=(8, 8), fontsize=9, color="gray")
        ax.plot(sim_log[:,9], sim_log[:,10], "b-", lw=1.0, alpha=0.5, label="vst (sim)")
        ax.plot(sim_log[:,1], sim_log[:,2], "c-", lw=1.8, alpha=0.8, label="car (sim)")
        ax.plot(real_data[:,8], real_data[:,9], "b--", lw=0.8, alpha=0.4, label="vst (real)")
        ax.plot(real_data[:,1], real_data[:,2], "r-", lw=2.2, alpha=0.9, label="car (real)")
        ax.set_aspect("equal"); ax.set_xlim(-0.5, 2.8); ax.set_ylim(-0.6, 2.6)
    elif idx == 1:
        ax.fill_between(sim_log[:,0], 0, sim_log[:,5]*100, alpha=0.08, color="cyan")
        ax.plot(sim_log[:,0], sim_log[:,5]*100, "c-", lw=1.5, label="ey sim")
        ax.plot(real_data[:,0], real_data[:,5]*100, "r-", lw=1.8, label="ey real")
        ax.axhline(0, color="gray", lw=0.5)
    elif idx == 2:
        ax.fill_between(sim_log[:,0], 0, sim_log[:,6]*100, alpha=0.08, color="cyan")
        ax.plot(sim_log[:,0], sim_log[:,6]*100, "c-", lw=1.5, label="ex sim")
        ax.plot(real_data[:,0], real_data[:,6]*100, "r-", lw=1.8, label="ex real")
        ax.axhline(0, color="gray", lw=0.5)
    elif idx == 3:
        ax.plot(sim_log[:,0], sim_log[:,4], "c-", lw=1.5, label="v sim")
        ax.plot(real_data[:,0], real_data[:,4], "r-", lw=1.8, label="v real")
    elif idx == 4:
        ax.plot(sim_log[:,0], sim_log[:,7], "c-", lw=1.5, label="delta sim")
        ax.plot(real_data[:,0], real_data[:,7], "r-", lw=1.8, label="delta real")
    elif idx == 5:
        ax.plot(sim_log[:,0], sim_log[:,3], "c-", lw=1.5, label="theta sim")
        ax.plot(real_data[:,0], real_data[:,3], "r-", lw=1.8, label="theta real")
    ax.set_title(titles[idx]); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_xlabel("t [s]")

plt.tight_layout()
fig.savefig("tools/sim_vs_real.png", dpi=150, bbox_inches="tight")
print("Saved: tools/sim_vs_real.png")
plt.close()
print("Done!")
