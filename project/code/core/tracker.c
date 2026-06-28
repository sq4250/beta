/**
 * tracker.c — LQR 横向跟踪器 (200Hz)
 */

#include "tracker.h"
#include "lqr_gains.h"
#include "ins.h"
#include "utils.h"

static f32 g_ey = 0.0f;
static f32 g_ex = 0.0f;
static f32 g_omega_cmd = 0.0f;

void tracker_init(void) {
    g_ey = 0.0f;
    g_ex = 0.0f;
    g_omega_cmd = 0.0f;
}

void tracker_lqr_gains(f32 g[5], f32 v) {
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

f32 tracker_step(const CarState *rs, const CarState *vst, f32 omega_ff) {
    f32 ct = cosf(vst->theta), st = sinf(vst->theta);
    g_ex = (vst->x - rs->x) * ct + (vst->y - rs->y) * st;
    f32 e_y = -(rs->x - vst->x) * st + (rs->y - vst->y) * ct;

    f32 eth  = wrap_pi(rs->theta - vst->theta);
    f32 ey_d = rs->v * sinf(eth);
    f32 eth_d = ins_theta_rate()                    // 真实车角速度 (IMU 直接测量)
              - bicycle_curvature(vst->v, vst->delta);  // 虚拟车角速度 (模型)
    f32 ed    = rs->delta - vst->delta;

    f32 g[5]; tracker_lqr_gains(g, vst->v);
    f32 omega_fb = g[0]*e_y + g[1]*ey_d + g[2]*eth + g[3]*eth_d + g[4]*ed;
    f32 omega_cmd = omega_ff - omega_fb;

    g_ey = e_y;
    g_omega_cmd = omega_cmd;
    return omega_cmd;
}

f32 tracker_get_ey(void)     { return g_ey; }
f32 tracker_get_ex(void)     { return g_ex; }
f32 tracker_get_omega_cmd(void) { return g_omega_cmd; }
