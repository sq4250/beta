/**
 * longitudinal.c — 纵向位置 LADRC + 阿克曼差速
 *
 * 模型: v_dot = -alpha*v + b0*throttle + f
 * LESO: z1_dot = b0*u_prev - alpha*z1 + z2 + 2*wo*(v - z1)
 *        z2_dot = wo² * (v - z1)
 * 控制: u0 = kp*e_x + kd*(v_ref - z1) + a_ref + alpha*z1 - z2
 *        thr = u0 * inv_b0  →  clamp  →  差速分配
 */
#include "longitudinal.h"
#include "utils.h"

static f32 g_z1 = 0.0f;      // LESO 速度估计
static f32 g_z2 = 0.0f;      // LESO 总扰动估计
static f32 g_u_prev = 0.0f;  // 上周期 throttle (延迟反馈抗饱和)
static f32 g_inv_b0;         // 1/b0 (逆元, init 时计算; b0 可调)

void longitudinal_init(void) {
    g_z1 = 0.0f; g_z2 = 0.0f; g_u_prev = 0.0f;
    g_inv_b0 = 1.0f / LONG_B0;
}

void longitudinal_step(f32 *thr_L, f32 *thr_R,
                       f32 v_meas, f32 v_ref, f32 a_ref, f32 e_x, f32 delta) {

    f32 e = v_meas - g_z1;
    f32 z1_dot = LONG_B0 * g_u_prev - LONG_ALPHA * g_z1 + g_z2 + 2.0f * LONG_WO * e;
    f32 z2_dot = LONG_WO * LONG_WO * e;
    g_z1 += z1_dot * LONG_DT;
    g_z2 += z2_dot * LONG_DT;

    f32 u0 = LONG_KP * e_x + LONG_KD * (v_ref - g_z1) + a_ref + LONG_ALPHA * g_z1 - g_z2;
    f32 thr = clamp(u0 * g_inv_b0, -1.0f, 1.0f);
    g_u_prev = thr;  // 延迟反馈

    f32 abs_d = delta; if (abs_d < 0.0f) abs_d = -abs_d;
    if (abs_d < 1e-4f) {
        *thr_L = thr; *thr_R = thr;
    } else {
        // 阿克曼差速: thr_L/R = thr * (R∓w/2)/R = thr * (1 ∓ w/2 * 1/R)
        // 1/R = tanδ/L = tanδ * INV_WHEELBASE, 全乘, 零除
        f32 inv_R = tanf(delta) * INV_WHEELBASE;
        f32 hw    = TRACK_WIDTH * 0.5f;
        *thr_L = thr * (1.0f - hw * inv_R);
        *thr_R = thr * (1.0f + hw * inv_R);
    }
}
