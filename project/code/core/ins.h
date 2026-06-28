/**
 * ins.h — 惯性导航状态估计 (yaw + 位置)
 */
#ifndef INS_H
#define INS_H
#include "common.h"
#include "hal_imu.h"
#include "car_state.h"

void ins_init(void);
void ins_update_yaw(ImuHandle imu);             // 100Hz: 四元数融合
f32  ins_yaw(void);                              // 当前偏航 [rad]
void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt); // 里程计位置积分

#endif
