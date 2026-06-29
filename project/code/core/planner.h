/**
 * planner.h — 规划层 (20Hz): 车体变换 + NN 前向 → 动作
 */
#ifndef PLANNER_H
#define PLANNER_H
#include "car_state.h"
#include "config.h"

// NN 前向: vst + 3航点 → PlannerAction ZOH
void planner_forward(PlannerAction *act,
                     const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3
    );

#endif
