/**
 * lateral.c — LQR 横向跟踪器 (200Hz)
 */
#include "lateral.h"
#include "lqr_gains.h"
#include "utils.h"

static void lqr_lookup(f32 g[5], f32 v) {
    f32 vc = v;
    if (vc < LQR_V_MIN) vc = LQR_V_MIN;
    if (vc > 3.0f) vc = 3.0f;

    u32 lo = 0, hi = LQR_SPEED_POINTS - 1;
    while (lo < hi) {
        u32 mid = (lo + hi) / 2;
        if (lqr_speed_bp[mid] < vc) lo = mid + 1;
        else hi = mid;
    }
    if (lo == 0) {
        for (u32 j = 0; j < 5; ++j) g[j] = lqr_gains[0][j];
    } else if (lo >= LQR_SPEED_POINTS) {
        for (u32 j = 0; j < 5; ++j) g[j] = lqr_gains[LQR_SPEED_POINTS - 1][j];
    } else {
        f32 v_lo = lqr_speed_bp[lo - 1], v_hi = lqr_speed_bp[lo];
        f32 frac = (vc - v_lo) / (v_hi - v_lo);
        for (u32 j = 0; j < 5; ++j)
            g[j] = lqr_gains[lo - 1][j] + frac * (lqr_gains[lo][j] - lqr_gains[lo - 1][j]);
    }
}

f32 lateral_step(const CarState *rs, const CarState *vst, f32 omega_ff, f32 gyro_z,
                 f32 *ex, f32 *ey
    ) {
    f32 ct = cosf(vst->theta), st = sinf(vst->theta);
    *ex = (vst->x - rs->x) * ct + (vst->y - rs->y) * st;
    *ey = (rs->x - vst->x) * st - (rs->y - vst->y) * ct;

    f32 eth  = wrap_pi(vst->theta - rs->theta);
    f32 ey_d = rs->v * sinf(eth);
    f32 eth_d = bicycle_curvature(vst->v, vst->delta) - gyro_z;
    f32 ed    = vst->delta - rs->delta;

    f32 g[5]; lqr_lookup(g, vst->v);
    f32 omega_fb = g[0]*(*ey) + g[1]*ey_d + g[2]*eth + g[3]*eth_d + g[4]*ed;
    return omega_ff + omega_fb;
}
