/**
 * lateral.h — LQR 横向跟踪器 (200Hz)
 */
#ifndef LATERAL_H
#define LATERAL_H
#include "car_state.h"
#include "config.h"

f32 lateral_step(const CarState *rs, const CarState *vst, f32 omega_ff, f32 gyro_z, f32 ey);

#endif
