/**
 * estimator.h — 状态估计: 测量 + 控制量 → 5 状态
 *
 * x̂ = f(x̂, u, y)
 *   u = actuator_cmd_t, y = imu_data_t + encoder_t
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include "common.h"

void car_estimate_update(car_state_t *car, const imu_data_t *imu,
                         const encoder_t *enc, const actuator_cmd_t *cmd);

#endif
