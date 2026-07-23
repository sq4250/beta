/**
 * planner.h — 规划层 (20Hz): 车体变换 + NN 前向 → 动作
 */
#ifndef PLANNER_H
#define PLANNER_H
#include "config.h"

// NN 前向: vst + 3航点 → planner_action_t ZOH
void planner_forward(planner_action_t *act,
                     const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3);

#endif
