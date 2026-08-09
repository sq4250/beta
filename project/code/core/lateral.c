/**
 * lateral.c — LQR 横向跟踪器 (200Hz), 4-term simple with ey integral
 *
 * 4-state direct model: [ey_int, ey, eth, eth_d]
 *   ω_cmd → car yaw accel via v/L  (no ed, no wo yaw-rate lag)
 *
 * 4 gains: [Ki, K_ey, K_eth, K_ethd]
 *   omega_fb = Ki·ey_int + K_ey·ey + K_eth·eth + K_ethd·eth_d
 */
#include "lateral.h"
#include "lqr_gains.h"
#include "utils.h"

#define N_GAINS 4
#define EY_INT_MAX  0.5f    /* anti-windup clamp [m·s] */

static f32 g_ey_int;

void lateral_reset(void) {
    g_ey_int = 0.0f;
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

f32 lateral_step(const CarState *rs, const CarState *vst, f32 omega_ff, f32 gyro_z, f32 ey
    ) {
    f32 eth   = wrap_pi(vst->theta - rs->theta);
    f32 eth_d = bicycle_curvature(vst->v, vst->delta) - gyro_z;

    /* ey integral with anti-windup */
    g_ey_int += ey * CTRL_DT;
    if      (g_ey_int >  EY_INT_MAX) g_ey_int =  EY_INT_MAX;
    else if (g_ey_int < -EY_INT_MAX) g_ey_int = -EY_INT_MAX;

    f32 g[N_GAINS]; lqr_lookup(g, vst->v);
    f32 omega_fb = g[0]*g_ey_int + g[1]*ey + g[2]*eth + g[3]*eth_d;
    return omega_ff + omega_fb;
}
