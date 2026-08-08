/**
 * planner.c — 规划层外壳 (20Hz)
 *
 * 编码 + 模型架构全在模型文件中, 此处只做薄转发.
 * 换模型: 只改 config.h 的 #include.
 */
#include "planner.h"
/* nn_planner_step 通过 planner.h→config.h→模型头文件 间接可见 */

void planner_forward(PlannerAction *act,
                     const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3,
                     f32 v2, f32 v3) {
    f32 raw[2];
    nn_planner_step(raw, vst, g1, g2, g3, v2, v3);
    act->a     = raw[0];
    act->omega = raw[1];
}
