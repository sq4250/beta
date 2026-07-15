/**
 * estimator.h — 状态估计: 测量 + 控制量 → 5 状态
 *
 * x̂ = f(x̂, u, y)
 *   u = ActuatorCmd, y = ImuData + Encoder
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include "common.h"
#include "car_state.h"

void car_estimate_update(CarState *car, const ImuData *imu,
                         const Encoder *enc, const ActuatorCmd *cmd);
void estimator_reset(void);   /* 重置内部 EMA 滤波状态 */

#endif
