/**
 * ins.h — 航向估计 + 里程计 (纯计算, 不访问硬件)
 *
 * 调用方持有 CarState, INS 直接读写 car->theta, 不维护副本。
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void ins_init(void);

// 航向融合: 陀螺 + 四元数 → car->theta (直接写入, 调用方管理状态)
void ins_fuse_theta(CarState *car, f32 gyro_z, const f32 quat[4], bool has_quat);

// 陀螺 z 轴角速度 [rad/s] (供 tracker eth_d 使用)
f32  ins_theta_rate(void);

// 里程计: 编码器 → v, x, y (theta 由 ins_fuse_theta 维护)
void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt);

#endif
