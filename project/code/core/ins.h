/**
 * ins.h — 状态估计: 测量 + 控制量 → 5状态
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void car_estimate_update(CarState *car, const ImuData *imu,
                         const Encoder *enc, const ActuatorCmd *cmd);

#endif
