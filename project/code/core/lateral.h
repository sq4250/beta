/**
 * lateral.h — LQR 横向跟踪器 (200Hz), 6-term with ey integral
 */
#ifndef LATERAL_H
#define LATERAL_H
#include "config.h"

f32 lateral_step(const car_state_t *rs, const car_state_t *vst, f32 omega_ff, f32 gyro_z, f32 ey);
void lateral_reset(void);

#endif
