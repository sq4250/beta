/**
 * kinematics.h — FullSim 摩擦圆物理模拟 (模型无关)
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "common.h"
#include "config.h"

/**
 * FullSim 单步推进 (200Hz, dt=5ms)
 * 对应 Python FullSim.simulate() — 摩擦圆约束
 *
 * @param s       小车状态 (输入/输出)
 * @param a_raw   NN 原始纵向加速度 [m/s²]
 * @param w_raw   NN 原始转角角速度 [rad/s]
 * @param a_eff   物理层实际纵向加速度 [m/s²] (输出, 前馈)
 * @param w_eff   物理层实际转角角速度 [rad/s] (输出, 前馈)
 */
void mcu_kinematics_step(car_state_t *s, f32 a_raw, f32 w_raw,
                         f32 *a_eff, f32 *w_eff);

#endif
