/**
 * kinematics.c — FullSim 摩擦圆物理模拟
 *
 * 模型无关: 所有模型共用同一套物理.
 * 输入编码逻辑在各自的 nn_model_*.c 中.
 */
#include "kinematics.h"
#include "utils.h"
#include <math.h>

void mcu_kinematics_step(car_state_t *s, f32 a_raw, f32 w_raw,
                         f32 *a_eff, f32 *w_eff) {
    /* ① 动作限幅 */
    f32 an = clamp(a_raw, -A_BRAKE_MAX, A_LONG_MAX);
    f32 om = clamp(w_raw, -OMEGA_DELTA_MAX, OMEGA_DELTA_MAX);

    /* ② 名义速度 → 裁剪 → 反算真实加速度 */
    f32 vn = s->v + an * CTRL_DT;
    vn = clamp(vn, 0.0f, V_MAX);
    f32 al = (vn - s->v) / CTRL_DT;   /* 无需分支: 未裁剪时 al==an */

    /* ③ 摩擦圆/椭圆: 根据加速度方向选纵向半轴 */
    f32 semi = (al >= 0.0f) ? A_LONG_MAX : A_BRAKE_MAX;
    f32 r   = clamp(al / (semi + 1e-8f), -1.0f, 1.0f);
    f32 alm = A_LAT_MAX * sqrtf(1.0f - r*r + 1e-12f);

    /* ④ 转角限幅 → 反算真实 omega */
    f32 vs   = fmaxf(vn, 0.01f);
    f32 dl   = atanf(alm * WHEELBASE / (vs * vs));
    f32 dmax = fminf(DELTA_MAX, dl);
    f32 dn   = s->delta + om * CTRL_DT;
    dn = clamp(dn, -dmax, dmax);
    om = (dn - s->delta) / CTRL_DT;   /* 无需分支: 未裁剪时 om==w_raw */

    /* ⑤ 位置/航向更新 (旧速度, 旧航向) */
    s->x     += s->v * cosf(s->theta) * CTRL_DT;
    s->y     += s->v * sinf(s->theta) * CTRL_DT;
    s->theta += s->v * tanf(dn) / WHEELBASE * CTRL_DT;

    /* ⑥ 状态写入 + 有效动作输出 */
    s->v = vn;
    s->delta = dn;
    *a_eff = al;
    *w_eff = om;
}
