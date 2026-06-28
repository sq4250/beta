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

#endif
