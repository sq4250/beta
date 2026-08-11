"""
tune_lqr.py — LQR 参数交互调参，实时显示增益和极点

用法: python tools/tune_lqr.py [Q_ey] [Q_eth] [Q_ethd] [R] [wo] [L]
      不带参数使用默认值，带参数直接计算

示例: python tools/tune_lqr.py 0 5 0.5 1 15 0.15
"""
import sys
import numpy as np
from scipy.linalg import solve_continuous_are

DEFAULTS = {"q_ey": 0, "q_eth": 5, "q_ethd": 0.5, "R": 1.0, "wo": 15.0, "L": 0.15}

def compute(q_ey, q_eth, q_ethd, R, wo, L):
    print(f'J = {q_ey}·ey² + {q_eth}·eth² + {q_ethd}·eth_d² + {R}·Δδ²')
    print(f'wo = {wo} rad/s  L = {L}m  3-state + wo 一阶惯性')
    print()
    print(f'{"v":>5s}  {"K_ey":>8s}  {"K_eth":>8s}  {"K_ethd":>8s}  {"poles":>32s}  {"damp":>5s}')
    print('-' * 85)
    for v in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]:
        A = np.array([[0, v, 0],
                      [0, 0, 1],
                      [0, 0, -wo]])
        B = np.array([[0], [0], [-wo * v / L]])
        Q = np.diag([q_ey, q_eth, q_ethd])
        P = solve_continuous_are(A, B, Q, np.array([[R]]))
        K_riccati = (np.linalg.solve(np.array([[R]]), B.T @ P)).flatten()
        K_stored = -K_riccati  # flip for u = +K·x convention
        Acl = A - B @ K_riccati.reshape(1, 3)
        ev = np.sort(np.real(np.linalg.eigvals(Acl)))
        damp_ratio = abs(ev[1]) / abs(ev[0]) if ev[0] != 0 else float('inf')
        if abs(ev[1]) < 1e-9: damp_ratio = 1.0
        p_str = f'{ev[0]:.1f} {ev[1]:.1f} | {ev[2]:.1f}'
        print(f'{v:5.1f}  {K_stored[0]:8.3f}  {K_stored[1]:8.3f}  {K_stored[2]:8.3f}  {p_str:>32s}  {damp_ratio:4.1f}x')

    print()
    spd = "0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0"
    print(f'生成命令:')
    print(f'  python tools/gen_lqr.py --q-ey {q_ey} --q-eth {q_eth} --q-ethd {q_ethd} -r {R} --wo {wo} --L {L} --speeds "{spd}" -o project/code/core/lqr_gains.h')

if __name__ == '__main__':
    args = sys.argv[1:]
    params = {
        "q_ey":   float(args[0]) if len(args) > 0 else DEFAULTS["q_ey"],
        "q_eth":  float(args[1]) if len(args) > 1 else DEFAULTS["q_eth"],
        "q_ethd": float(args[2]) if len(args) > 2 else DEFAULTS["q_ethd"],
        "R":      float(args[3]) if len(args) > 3 else DEFAULTS["R"],
        "wo":     float(args[4]) if len(args) > 4 else DEFAULTS["wo"],
        "L":      float(args[5]) if len(args) > 5 else DEFAULTS["L"],
    }
    compute(**params)
