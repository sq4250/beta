/**
 * lateral.h — LQR 横向跟踪器 (200Hz)
 */
#ifndef LATERAL_H
#define LATERAL_H
#include "car_state.h"
#include "config.h"

// LQR 横向: 真实 vs 虚拟 → omega_cmd + e_x + e_y (全部显式返回)
f32 lateral_step(const CarState *rs, const CarState *vst, f32 omega_ff, f32 gyro_z,
                 f32 *ex, f32 *ey
    );

#endif
