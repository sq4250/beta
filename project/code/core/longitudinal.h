/**
 * longitudinal.h — 纵向 LADRC (降阶 LESO + PD) + 阿克曼差速
 *
 * 模型:   v̇ = -α·v + b₀·thr + f
 * 降阶观测: ż = -wₒ·(z + (wₒ-α)·v + b₀·thr)  →  f̂ = z + wₒ·v
 * 控制律: thr = (kp·e_x + kd·(v_ref-v) + a_ref + α·v - f̂) / b₀
 */

#ifndef LONGITUDINAL_H
#define LONGITUDINAL_H

#include "common.h"
#include "config.h"

void longitudinal_init(void);

void longitudinal_step(f32 *thr_L, f32 *thr_R,
                       f32 v_meas, f32 v_ref, f32 a_ref, f32 e_x, f32 delta
    );

#endif
