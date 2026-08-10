/**
 * lateral.c — LQR 横向跟踪器 (200Hz), 3-state + wo 一阶惯性
 *
 * 3-state 模型: [ey, eth, eth_d], Δδ → car yaw accel via -wo·eth_d - wo·v/L·Δδ
 *
 * 约定:
 *   误差 r−y (VST−Car), RotZ(−θ_vst) 投影到 VST 系
 *   生成存 K_stored = −K_riccati, 代码用 u_fb = +K_stored·x
 *
 * 3 gains: [K_ey, K_eth, K_ethd]
 *   Δδ_fb = K_ey·ey + K_eth·eth + K_ethd·eth_d
 */
#include "lateral.h"
#include "lqr_gains.h"
#include "utils.h"

#define N_GAINS 3

void lateral_reset(void) {
    /* no integral state to reset */
}

static void lqr_lookup(f32 g[N_GAINS], f32 v) {
    f32 vc = v;
    if (vc < LQR_V_MIN) vc = LQR_V_MIN;
    if (vc > 5.0f) vc = 5.0f;

    u32 lo = 0, hi = LQR_SPEED_POINTS - 1;
    while (lo < hi) {
        u32 mid = (lo + hi) / 2;
        if (lqr_speed_bp[mid] < vc) lo = mid + 1;
        else hi = mid;
    }
    if (lo == 0) {
        for (u32 j = 0; j < N_GAINS; ++j) g[j] = lqr_gains[0][j];
    } else if (lo >= LQR_SPEED_POINTS) {
        for (u32 j = 0; j < N_GAINS; ++j) g[j] = lqr_gains[LQR_SPEED_POINTS - 1][j];
    } else {
        f32 v_lo = lqr_speed_bp[lo - 1], v_hi = lqr_speed_bp[lo];
        f32 frac = (vc - v_lo) / (v_hi - v_lo);
        for (u32 j = 0; j < N_GAINS; ++j)
            g[j] = lqr_gains[lo - 1][j] + frac * (lqr_gains[lo][j] - lqr_gains[lo - 1][j]);
    }
}

f32 lateral_step(const CarState *rs, const CarState *vst, f32 gyro_z, f32 ey) {
    f32 eth   = wrap_pi(vst->theta - rs->theta);
    f32 eth_d = bicycle_curvature(vst->v, vst->delta) - gyro_z;

    f32 g[N_GAINS]; lqr_lookup(g, vst->v);
    f32 delta_fb = g[0]*ey + g[1]*eth + g[2]*eth_d;   /* u_fb = +K_stored·x */
    return delta_fb;
}
