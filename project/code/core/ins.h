/**
 * ins.h — 航向估计 + 里程计 (纯计算, 不访问硬件)
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void ins_init(void);

// 航向融合: 陀螺积分 + 四元数互补滤波 → g_theta
void ins_fuse_theta(f32 gyro_z, const f32 quat[4], bool has_quat);

// 里程计: 编码器 → v, x, y; theta 来自 g_theta
void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt);

#endif
