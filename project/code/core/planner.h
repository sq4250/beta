/**
 * planner.h — 规划层 (20Hz): 车体变换 + NN 前向 → 动作
 */
#ifndef PLANNER_H
#define PLANNER_H
#include "car_state.h"
#include "config.h"

// NN 前向: vst + 3航点 + 门控 → PlannerAction ZOH
// v2/v3: 1.0=有效航点, 0.0=重复填充应忽略
void planner_forward(PlannerAction *act,
                     const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3,
                     f32 v2, f32 v3);

#endif
