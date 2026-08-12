/**
 * planner.h — 规划层 (20Hz): 车体变换 + NN 前向 → 动作
 */
#ifndef PLANNER_H
#define PLANNER_H
#include "common.h"
#include "config.h"

/* NN 前向: vst + 3航点 + 门控 → planner_action_t ZOH
 * v2/v3: 1.0=有效航点, 0.0=重复填充应忽略 */
void planner_forward(planner_action_t *act,
                     const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3);

/* 混合模式: is_wp=true → D5, false → Kamm */
#if NN_MODEL_VERSION == NN_VER_HYBRID
void planner_forward_hybrid(planner_action_t *act,
                            const car_state_t *vst,
                            const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                            f32 v2, f32 v3, bool is_wp);
/* 当前模型物理约束 (混合模式运行时切换) */
extern f32 g_a_long_max, g_a_brake_max, g_a_lat_max, g_v_max;
#else
#define g_a_long_max  A_LONG_MAX
#define g_a_brake_max A_BRAKE_MAX
#define g_a_lat_max   A_LAT_MAX
#define g_v_max       V_MAX
#endif

#endif