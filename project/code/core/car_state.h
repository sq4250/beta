/**
 * car_state.h — 小车状态类型
 */

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "common.h"

typedef struct {
    f32 x, y, theta, v, delta;
} CarState;

typedef struct {
    f32 x, y;
} Waypoint;

#endif
