/**
 * planner.c — 规划层实现: 纯 NN 推理
 */
#include "planner.h"
#include "nn_model.h"
#include "kinematics.h"

void planner_forward(PlannerAction *act,
                     const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3
    ) {
    f32 inp[8];
    world_to_body_8d(inp, vst, g1, g2, g3);
    f32 raw[2];
    nn_forward(raw, inp);
    act->a     = raw[0];
    act->omega = raw[1];
}
