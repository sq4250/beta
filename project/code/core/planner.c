/**
 * planner.c — 规划层实现: 纯 NN 推理
 */
#include "planner.h"
#include "nn_inference.h"
#include "kinematics.h"

void planner_forward(const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3,
                     f32 *nn_a, f32 *nn_o) {
    f32 inp[8];
    world_to_body_8d(inp, vst, g1, g2, g3);
    f32 act[2];
    nn_forward(act, inp);
    *nn_a = act[0];
    *nn_o = act[1];
}
