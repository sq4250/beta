/**
 * car_state.h — 小车状态与控制类型
 */

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "common.h"

// 5D 小车状态 (世界系 NWU: X前 Y左 Z上)
typedef struct {
    f32 x, y;           // 世界坐标 [m]
    f32 theta;          // 航向 [rad]
    f32 v;              // 纵向速度 [m/s]
    f32 delta;          // 前轮转角 [rad]
} CarState;

// 航点 (世界系)
typedef struct {
    f32 x, y;
} Waypoint;

// 参考轨迹点
typedef struct {
    f32 x, y, theta, v, delta;
} PlannerRef;

// LQR 增益集 (4个CARE原始值)
typedef struct {
    f32 k[4];           // [K_e_y, K_e_theta, K_e_theta_dot, K_delta]
} LqrRaw;

#endif
