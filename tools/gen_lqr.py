"""
gen_lqr.py — LQR 增益表生成器

用法: python gen_lqr.py [--wo 30] [--R 0.002] [--Qth 1.2] [--Qthd 0.22] [--alpha 0.60]
输出: lqr_gains.h 格式的 C 数组
"""
import numpy as np
from scipy.linalg import solve_continuous_are
import argparse

L = 0.15
Q_EY = 1.0
SPEEDS = [0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]

def gen(wo, R, qth, qthd, alpha):
    print(f"/* lqr_gains.h — 5-term LQR gains, offline computed */")
    print(f"/* Cost: J = e_y^2 + {qth}*e_theta^2 + {qthd}*e_theta_dot^2 + {R}*omega^2, wo={wo}, alpha={alpha} */")
    print(f"#ifndef LQR_GAINS_H")
    print(f"#define LQR_GAINS_H")
    print(f"#include \"common.h\"")
    print()
    print(f"#define LQR_SPEED_POINTS {len(SPEEDS)}")
    print()
    bp = ", ".join(f"{v:.1f}f" for v in SPEEDS)
    print(f"static const f32 lqr_speed_bp[] = {{{bp}}};")
    print(f"static const f32 lqr_gains[{len(SPEEDS)}][5] = {{")

    for v in SPEEDS:
        A = np.array([[0, v, 0, 0], [0, 0, 1, 0], [0, 0, -wo, wo*v/L], [0, 0, 0, 0]])
        B = np.array([[0], [0], [0], [1]])
        Q = np.diag([Q_EY, qth, qthd, 0.0])
        Rm = np.array([[R]])
        P = solve_continuous_are(A, B, Q, Rm)
        K = (np.linalg.inv(Rm) @ B.T @ P).flatten()
        g0, g1, g2, g3, g4 = K[0], K[1]*alpha/v, K[1]*(1-alpha), K[2], K[3]
        print(f"    {{{g0:10.6f}f, {g1:10.6f}f, {g2:10.6f}f, {g3:10.6f}f, {g4:10.6f}f}},  /* v={v:.1f} */")

    print(f"}};")
    print()
    print(f"#endif")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--wo",    type=float, default=30)
    p.add_argument("--R",     type=float, default=0.002)
    p.add_argument("--Qth",   type=float, default=1.2)
    p.add_argument("--Qthd",  type=float, default=0.22)
    p.add_argument("--alpha", type=float, default=0.60)
    args = p.parse_args()
    gen(args.wo, args.R, args.Qth, args.Qthd, args.alpha)
