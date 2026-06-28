/**
 * planner.c — 规划层实现: 纯 NN 推理
 */
#include "planner.h"
#include "nn_inference.h"
#include "kinematics.h"
#include <string.h>

static f32 g_nn_a     = 0.0f;
static f32 g_nn_omega = 0.0f;

void planner_init(void) {
    g_nn_a     = 0.0f;
    g_nn_omega = 0.0f;
}

// 规划层: 世界系航点 → 车体系 8D → NN → ZOH
void planner_forward(const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3) {
    f32 inp[8];
    world_to_body_8d(inp, vst, g1, g2, g3);
    f32 act[2];
    nn_forward(act, inp);
    g_nn_a     = act[0];
    g_nn_omega = act[1];
}

f32 planner_nn_a(void)     { return g_nn_a; }
f32 planner_nn_omega(void) { return g_nn_omega; }
