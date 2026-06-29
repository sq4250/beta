/**
 * lateral.h — LQR 横向跟踪器 (200Hz)
 */
#ifndef LATERAL_H
#define LATERAL_H
#include "car_state.h"
#include "config.h"

// LQR 横向: 误差投影由调用方完成, ey 作为输入传入
f32 lateral_step(const CarState *rs, const CarState *vst, f32 omega_ff, f32 gyro_z, f32 ey);

#endif
