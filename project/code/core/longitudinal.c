/**
 * longitudinal.c — 纵向 LADRC (降阶 LESO + PD) + 阿克曼差速
 *
 * 降阶观测器 (仅估计扰动 f̂):
 *   辅助变量 z = f̂ - wₒ·v
 *   ż = -wₒ·(z + (wₒ − α)·v + b₀·thr)    (ḟ≡0 分段常值假设)
 *   重建 f̂ = z + wₒ·v
 *
 * 控制: 用滤波 v_meas 替代 z1, 用 f̂ 替代 z2
 *   u₀ = kp·e_x + kd·(v_ref − v_meas) + a_ref + α·v_meas − f̂
 *   thr = u₀ / b₀
 */
#include "longitudinal.h"
#include "utils.h"

static f32 g_z    = 0.0f;  // 降阶 LESO 辅助变量
static f32 g_u_prev = 0.0f;  // 上周期油门 (延迟反馈抗饱和)
static f32 g_inv_b0;         // 1/b₀ (init 计算)
static f32 g_f_hat = 0.0f;   // 观测扰动 (debug 用)

void longitudinal_init(void) {
    g_z      = 0.0f;
    g_u_prev = 0.0f;
    g_f_hat  = 0.0f;
    g_inv_b0 = 1.0f / LONG_B0;
}

void longitudinal_step(f32 *thr_l, f32 *thr_r,
                       f32 v_meas, f32 v_ref, f32 a_ref, f32 e_x, f32 delta
    ) {

    // ── 降阶 LESO: 仅观测扰动 f̂ ──
    f32 z_dot = -LONG_WO * (g_z + (LONG_WO - LONG_ALPHA) * v_meas + LONG_B0 * g_u_prev);
    g_z += z_dot * LONG_DT;
    g_f_hat = g_z + LONG_WO * v_meas;
    /* 小车主要受阻力, f̂ 应为负; 正值扰动通常是轮胎打滑/负载减轻,
       将动力误判为扰动, 钳位到 0 防止错误补偿 */
    if      (g_f_hat > 0.0f)            { g_f_hat = 0.0f;            g_z = -LONG_WO * v_meas; }
    else if (g_f_hat < -LONG_FHAT_MAX)  { g_f_hat = -LONG_FHAT_MAX;  g_z = -LONG_FHAT_MAX - LONG_WO * v_meas; }

    // ── PD 控制律 (v_meas 已滤波, 来自观测层) ──
    f32 u0 = LONG_KP * e_x + LONG_KD * (v_ref - v_meas) + a_ref + LONG_ALPHA * v_meas - g_f_hat;
    f32 thr = clamp(u0 * g_inv_b0, -1.0f, 1.0f);
    g_u_prev = thr;

    // ── 阿克曼差速 ──
    f32 abs_d = delta; if (abs_d < 0.0f) abs_d = -abs_d;
    if (abs_d < 1e-4f) {
        *thr_l = thr; *thr_r = thr;
    } else {
        f32 inv_R = tanf(delta) * INV_WHEELBASE;
        f32 hw    = TRACK_WIDTH * 0.5f;
        *thr_l = thr * (1.0f - hw * inv_R);
        *thr_r = thr * (1.0f + hw * inv_R);
    }
}

f32 longitudinal_f_hat(void) { return g_f_hat; }
