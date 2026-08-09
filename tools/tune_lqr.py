"""
tune_lqr.py — LQR 参数交互调参，实时显示增益和极点

用法: python tools/tune_lqr.py [Q_ey] [Q_eth] [Q_ethd] [R] [L]
      不带参数使用默认值，带参数直接计算

示例: python tools/tune_lqr.py 10 0 5 1 0.15
"""
import sys
import numpy as np
from scipy.linalg import solve_continuous_are

DEFAULTS = {"q_ey": 10, "q_eth": 0, "q_ethd": 5, "R": 1.0, "L": 0.15, "qi": 0.05}

def compute(q_ey, q_eth, q_ethd, R, L, qi=0.05):
    print(f'J = {qi}·ey_int² + {q_ey}·ey² + {q_eth}·eth² + {q_ethd}·eth_d² + {R}·ω²')
    print(f'L = {L}m   R = {R}   4态直驱')
    print()
    print(f'{"v":>5s}  {"Ki":>8s}  {"K_ey":>8s}  {"K_eth":>8s}  {"K_ethd":>8s}  {"poles":>32s}  {"damp":>5s}')
    print('-' * 90)
    for v in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]:
        A = np.array([[0, 1, 0, 0],
                      [0, 0, v, 0],
                      [0, 0, 0, 1],
                      [0, 0, 0, 0]])
        B = np.array([[0], [0], [0], [v / L]])
        Q = np.diag([qi, q_ey, q_eth, q_ethd])
        P = solve_continuous_are(A, B, Q, np.array([[R]]))
        K = (np.linalg.solve(np.array([[R]]), B.T @ P)).flatten()
        Acl = A - B @ K.reshape(1, 4)
        ev = np.sort(np.real(np.linalg.eigvals(Acl)))
        damp = abs(ev[0] / ev[-2])
        p_str = f'{ev[0]:.1f} {ev[1]:.1f} | {ev[2]:.1f} {ev[3]:.1f}'
        print(f'{v:5.1f}  {K[0]:8.4f}  {K[1]:8.2f}  {K[2]:8.2f}  {K[3]:8.2f}  {p_str:>32s}  {damp:4.1f}x')

    print()
    # compare to gen_lqr command
    spd = "0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0"
    print(f'生成命令:')
    print(f'  python tools/gen_lqr.py --simple --q-ey-int {qi} --q-ey {q_ey} --q-eth {q_eth} --q-ethd {q_ethd} -r {R} --L {L} --speeds "{spd}" > project/code/core/lqr_gains.h')

if __name__ == '__main__':
    args = sys.argv[1:]
    params = {
        "q_ey":   float(args[0]) if len(args) > 0 else DEFAULTS["q_ey"],
        "q_eth":  float(args[1]) if len(args) > 1 else DEFAULTS["q_eth"],
        "q_ethd": float(args[2]) if len(args) > 2 else DEFAULTS["q_ethd"],
        "R":      float(args[3]) if len(args) > 3 else DEFAULTS["R"],
        "L":      float(args[4]) if len(args) > 4 else DEFAULTS["L"],
    }
    compute(**params)
