/**
 * ins.h — 状态估计 (纯计算, 不访问硬件)
 *
 * 调用方持有 CarState, INS 直接读写。
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void ins_init(void);

// 状态估计: IMU→theta + 编码器→v,x,y (200Hz, 所有输入来自参数)
void car_estimate_update(CarState *car, f32 gyro_z, const f32 quat[4], bool has_quat,
                         const Encoder *enc);

// 陀螺 z 轴角速度 [rad/s] (供 tracker eth_d)
f32 ins_theta_rate(void);

#endif
