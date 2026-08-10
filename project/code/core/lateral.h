/**
 * lateral.h — LQR 横向跟踪器 (200Hz), 3-state + wo 一阶惯性
 *
 * 3-state 模型: [ey, eth, eth_d], Δδ → car yaw accel via -wo·eth_d - wo·v/L·Δδ
 * 3 gains: [K_ey, K_eth, K_ethd] (已翻转, u_fb = +K·x)
 * δ_cmd = δ_vst + Δδ_fb       (前馈+反馈)
 */
#ifndef LATERAL_H
#define LATERAL_H
#include "common.h"
#include "config.h"

f32 lateral_step(const car_state_t *rs, const car_state_t *vst, f32 gyro_z, f32 ey);
void lateral_reset(void);

#endif
