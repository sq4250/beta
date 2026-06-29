/**
 * ins.h — 状态估计: 测量 + 控制量 → 5状态
 *
 * 传统观测器: x̂ = f(x̂, u, y)
 *   u = ActuatorCmd (上周期指令, δ 来自跟踪良好假设)
 *   y = ImuData + Encoder (IMU 偏航 + 编码器里程计)
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void ins_init(void);

// 状态估计 (1kHz): IMU→θ + 编码器→v,x,y + cmd→δ
void car_estimate_update(CarState *car, const ImuData *imu,
                         const Encoder *enc, const ActuatorCmd *cmd);

// 陀螺 z 轴角速度 [rad/s] (供 tracker eth_d)
f32 ins_theta_rate(void);

#endif
