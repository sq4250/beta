/**
 * car_state.h — 小车状态类型 + 几何常数
 */

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "common.h"

//===================================================车体几何===================================================
#define WHEELBASE         0.15f       // 轴距 L [m]
#define INV_WHEELBASE     6.6666667f  // 1/WHEELBASE
#define TRACK_WIDTH       0.15f       // 后轮距 [m]
//===================================================车体几何===================================================

// 5D 小车状态 (世界系 NWU: X前 Y左 Z上)
typedef struct {
    f32 x, y;
    f32 theta, v, delta;
} CarState;

typedef struct {
    f32 x, y;
} Waypoint;

#endif
