/**
 * planner.h — 规划层 (20Hz): 车体变换 + NN 前向 → ZOH 动作
 *
 * 纯函数, 不持有虚拟状态, 不做航点管理。
 * 输入: 虚拟状态 + 3航点窗口 (由上层传入)
 * 输出: ZOH (a_long, omega) 供跟踪层 200Hz 使用
 */

#ifndef PLANNER_H
#define PLANNER_H

#include "car_state.h"
#include "config.h"

void planner_init(void);

// 规划层入口: 世界→车体 8D → NN → ZOH
void planner_forward(const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3);

// ZOH 动作 (跟踪层 200Hz 直读)
f32  planner_nn_a(void);
f32  planner_nn_omega(void);

#endif
