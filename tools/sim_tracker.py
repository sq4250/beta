"""
sim_tracker.py — NN规划器 + LQR横向 + LADRC纵向 闭环仿真

Plant = LADRC设计模型 (油门→速度 一阶惯性):
  v_dot = -alpha*v + b0*throttle        ← 纵向
  theta_dot = v*tan(delta)/L            ← 横向
  delta_dot = omega                     ← 转向

用法: python sim_tracker.py
"""
import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys = __import__('sys'); sys.path.insert(0, str(PROJECT/'tools'))
from bicycle_model import BicycleParams, check_hit_substep

# ============================================================
p = BicycleParams(); L = p.wheelbase
DT = 1/300; DT_OUTER = 0.05; SUB = 15
TOL = 0.05

# LQR (横向)
Q_HGD = 0.5; R_OMG = 0.05; WO_LQR = 50; ALPHA_LQR = 0.3

# LADRC设计模型 (纵向): v_dot = -alpha*v + b0*throttle
ALPHA = 2.0
B0    = 7.0
WO    = 15
KP    = 8
KD    = 6

# 航点
WP = np.array([[4,0],[4,2],[0,2],[0,4],[2,6],[5,5],[6,2],[3,-1]], dtype=np.float32)

# ============================================================
# NN模型
# ============================================================
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

# ============================================================
# LQR增益表
# ============================================================
SP_BP = np.array([0.1,0.5,1.0,1.5,2.0,2.5,3.0])
lr = np.zeros((7,4))
for i,v in enumerate(SP_BP):
    A=np.array([[0,v,0,0],[0,0,1,0],[0,0,-WO_LQR,WO_LQR*v/L],[0,0,0,0]]); B=np.array([[0.],[0.],[0.],[1.]])
    lr[i]=(np.linalg.inv(np.array([[R_OMG]]))@B.T@solve_continuous_are(A,B,np.diag([1.,Q_HGD,0.,0.]),np.array([[R_OMG]]))).flatten()

def lqr5g(v,ey,eyd,eth,ethd,ed):
    vc=np.clip(abs(v),0.1,3.); K=np.array([np.interp(vc,SP_BP,lr[:,j]) for j in range(4)])
    g=np.array([K[0],K[1]*ALPHA_LQR/vc,K[1]*(1-ALPHA_LQR),K[2],K[3]])
    return g@[ey,eyd,eth,ethd,ed]

# ============================================================
# 主仿真
# ============================================================
vst = np.zeros(5,dtype=np.float32)  # 虚拟车 (300Hz MCUSim)
rs  = np.zeros(5,dtype=np.float32)  # 真实车 (一阶惯性+自行车)
wpi=0; reached=np.zeros(len(WP),bool)
nn_a=nn_omg=0.
z1=0.; fh=0.; up=0.  # LADRC state
ey_log=[]; ex_log=[]

for ctrl in range(int(12/DT_OUTER)):
    t=ctrl*DT_OUTER
    while wpi<len(WP) and reached[wpi]: wpi+=1
    if wpi>=len(WP): break
    ci=wpi; Nw=len(WP); ni=min(ci+1,Nw-1); n2i=min(ci+2,Nw-1)
    inp=bf8(vst,WP[ci],WP[ni],WP[n2i]); nn_a,nn_omg=nn(inp)
    vst_prev=vst[:2].copy()

    for sc in range(SUB):
        vst=mcu_step(vst,np.array([nn_a,nn_omg]))

        # 误差投影
        ct=np.cos(vst[2]); st_=np.sin(vst[2])
        ex=(vst[0]-rs[0])*ct+(vst[1]-rs[1])*st_          # ref-real
        ey=(rs[0]-vst[0])*st_-(rs[1]-vst[1])*ct          # ref-real

        # ── LADRC 纵向 ──
        vm=rs[3]; eo=vm-z1
        z1 += (-ALPHA*z1 + B0*up + fh + 2.0*WO*eo) * DT
        fh += (WO*WO*eo) * DT
        u0 = KP*ex + KD*(vst[3]-z1) + nn_a + ALPHA*z1 - fh
        thr = u0 / B0
        thr = np.clip(thr,-1,1); up = thr

        # Plant 纵向 = LADRC模型
        rs[3] += (-ALPHA*rs[3] + B0*thr) * DT
        rs[3] = np.clip(rs[3],0,3.5)

        # ── LQR 横向 ──
        # e = r - y, 前馈+反馈叠加
        eth=(vst[2]-rs[2]+np.pi)%(2*np.pi)-np.pi; ed=vst[4]-rs[4]
        eyd=rs[3]*np.sin(eth); ethd=vst[3]*np.tan(vst[4])/L-rs[3]*np.tan(rs[4])/L
        omg_fb=lqr5g(vst[3],ey,eyd,eth,ethd,ed)
        omg_cmd=nn_omg+omg_fb

        # Plant 横向 = 自行车
        rs[4] += omg_cmd*DT; rs[4]=np.clip(rs[4],-p.delta_max,p.delta_max)
        rs[2] += rs[3]*np.tan(rs[4])/L*DT
        rs[0] += rs[3]*np.cos(rs[2])*DT
        rs[1] += rs[3]*np.sin(rs[2])*DT

        ey_log.append(ey); ex_log.append(ex)

    if not reached[ci] and check_hit_substep(vst_prev,vst[:2],WP[ci],TOL):
        reached[ci]=True; print(f'  WP{ci} @t={t:.2f}s')

eya=np.array(ey_log); exa=np.array(ex_log)
print(f'eyRMS={np.sqrt(np.mean(eya**2)):.4f}m  exRMS={np.sqrt(np.mean(exa**2)):.4f}m  wps={sum(reached)}')
print(f'LADRC: alpha={ALPHA} b0={B0} wo={WO} kp={KP} kd={KD}')
print(f'LQR:   q_hdg={Q_HGD} r={R_OMG} wo_lqr={WO_LQR}')

# ============================================================
n=len(ey_log); ft=np.linspace(0,n*DT,n)
fvx=[];fvy=[];frx=[];fry=[];fey=[];fex=[]
vst2=np.zeros(5,dtype=np.float32); rs2=np.zeros(5,dtype=np.float32)
wpi2=0; reached2=np.zeros(len(WP),bool)
nn_a2=nn_omg2=0.; z12=0.; fh2=0.; up2=0.
for ctrl in range(int(12/DT_OUTER)):
    while wpi2<len(WP) and reached2[wpi2]: wpi2+=1
    if wpi2>=len(WP): break
    ci2=wpi2; Nw2=len(WP); ni2=min(ci2+1,Nw2-1); n2i2=min(ci2+2,Nw2-1)
    inp2=bf8(vst2,WP[ci2],WP[ni2],WP[n2i2]); nn_a2,nn_omg2=nn(inp2)
    vst_prev2=vst2[:2].copy()
    for sc in range(SUB):
        vst2=mcu_step(vst2,np.array([nn_a2,nn_omg2]))
        ct2=np.cos(vst2[2]); st2=np.sin(vst2[2])
        ex2=(vst2[0]-rs2[0])*ct2+(vst2[1]-rs2[1])*st2
        ey2=(rs2[0]-vst2[0])*st2-(rs2[1]-vst2[1])*ct2
        vm2=rs2[3]; eo2=vm2-z12
        z12+=(-ALPHA*z12+B0*up2+fh2+2.0*WO*eo2)*DT; fh2+=(WO*WO*eo2)*DT
        u02=KP*ex2+KD*(vst2[3]-z12)+nn_a2+ALPHA*z12-fh2; thr2=u02/B0
        thr2=np.clip(thr2,-1,1); up2=thr2
        rs2[3]+=(-ALPHA*rs2[3]+B0*thr2)*DT; rs2[3]=np.clip(rs2[3],0,3.5)
        eth2=(vst2[2]-rs2[2]+np.pi)%(2*np.pi)-np.pi; ed2=vst2[4]-rs2[4]
        eyd2=rs2[3]*np.sin(eth2); ethd2=vst2[3]*np.tan(vst2[4])/L-rs2[3]*np.tan(rs2[4])/L
        omg_fb2=lqr5g(vst2[3],ey2,eyd2,eth2,ethd2,ed2); omg_cmd2=nn_omg2+omg_fb2
        rs2[4]+=omg_cmd2*DT; rs2[4]=np.clip(rs2[4],-p.delta_max,p.delta_max)
        rs2[2]+=rs2[3]*np.tan(rs2[4])/L*DT; rs2[0]+=rs2[3]*np.cos(rs2[2])*DT; rs2[1]+=rs2[3]*np.sin(rs2[2])*DT
    fvx.append(vst2[0]);fvy.append(vst2[1]);frx.append(rs2[0]);fry.append(rs2[1])
    fey.append(-(rs2[0]-vst2[0])*np.sin(vst2[2])+(rs2[1]-vst2[1])*np.cos(vst2[2]))
    fex.append((vst2[0]-rs2[0])*np.cos(vst2[2])+(vst2[1]-rs2[1])*np.sin(vst2[2]))
    if not reached2[ci2] and check_hit_substep(vst_prev2,vst2[:2],WP[ci2],TOL):
        reached2[ci2]=True

fvx=np.array(fvx);fvy=np.array(fvy);frx=np.array(frx);fry=np.array(fry)
fey=np.array(fey);fex=np.array(fex); ft2=np.linspace(0,len(fvx)*DT_OUTER,len(fvx))

fig,(ax1,ax2)=plt.subplots(1,2,figsize=(13,5.5))
fig.suptitle(f'LQR+LADRC (alpha={ALPHA} b0={B0}) | eyRMS={np.sqrt(np.mean(fey**2)):.3f}m exRMS={np.sqrt(np.mean(fex**2)):.3f}m',fontsize=10)
ax1.plot(WP[:,0],WP[:,1],'s-',color='gray',ms=8,alpha=.4,label='wp')
ax1.set_xlim(-1,8);ax1.set_ylim(-2,8);ax1.set_aspect('equal')
ax1.set_xlabel('X (m)');ax1.set_ylabel('Y (m)');ax1.legend(fontsize=8)
vtr,=ax1.plot([],[],'b-',lw=1.5,alpha=.6,label='virtual')
rtr,=ax1.plot([],[],'r-',lw=2.,alpha=.9,label='real')
vd,=ax1.plot([],[],'bo',ms=6);rd,=ax1.plot([],[],'r.',ms=8)
ax2.set_xlim(0,12);ax2.set_ylim(-0.3,0.3)
ax2.set_xlabel('t (s)');ax2.set_ylabel('error (m)')
ax2.axhline(0,color='gray',lw=.5)
el,=ax2.plot([],[],'r-',lw=1.5,label='e_y')
xl,=ax2.plot([],[],'g-',lw=1.0,alpha=0.7,label='e_x')
ax2.legend(fontsize=8)

N=len(ft2);skip=max(1,N//300)
def upd(f):
    n=min(f*skip,N-1)
    vtr.set_data(fvx[:n],fvy[:n]);rtr.set_data(frx[:n],fry[:n])
    vd.set_data([fvx[n]],[fvy[n]]);rd.set_data([frx[n]],[fry[n]])
    el.set_data(ft2[:n],fey[:n]);xl.set_data(ft2[:n],fex[:n])
    return vtr,rtr,vd,rd,el,xl
ani=anim.FuncAnimation(fig,upd,frames=N//skip,interval=50,blit=False,repeat=False)
ani.save(str(PROJECT/'tools'/'tracker_sim.gif'),writer='pillow',fps=18,dpi=85)
print(f'GIF: tools/tracker_sim.gif')
plt.close()
