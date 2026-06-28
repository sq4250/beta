/**
 * ins.h — 惯性导航状态估计 (yaw + 位置)
 *
 * 纯计算, 不访问硬件。传感器数据由上层传入。
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "car_state.h"

void ins_init(void);
void ins_update_yaw(f32 gyro_z, const f32 quat[4], bool has_quat);  // 偏航融合 (数据由调用者提供)
f32  ins_yaw(void);
void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt);

#endif
