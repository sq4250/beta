/**
 * kinematics.c — 自行车模型 + 车体变换实现
 */

#include "kinematics.h"
#include "utils.h"

void mcu_kinematics_step(CarState *s, f32 a_long, f32 omega) {
    s->v     += clamp(a_long, -A_LONG_MAX, A_LONG_MAX) * CTRL_DT;
    s->delta += clamp(omega, -OMEGA_DELTA_MAX, OMEGA_DELTA_MAX) * CTRL_DT;

    s->v     = clamp(s->v,     0.0f,        V_MAX);
    s->delta = clamp(s->delta, -DELTA_MAX,  DELTA_MAX);

    s->theta += bicycle_curvature(s->v, s->delta) * CTRL_DT;
    s->x     += s->v * cosf(s->theta) * CTRL_DT;
    s->y     += s->v * sinf(s->theta) * CTRL_DT;
}

void world_to_body_8d(f32 out[8], const CarState *s, const Waypoint *g1, const Waypoint *g2, const Waypoint *g3) {
    f32 ct = cosf(s->theta), st = sinf(s->theta);
    f32 dx1 = g1->x - s->x, dy1 = g1->y - s->y;
    f32 dx2 = g2->x - s->x, dy2 = g2->y - s->y;
    f32 dx3 = g3->x - s->x, dy3 = g3->y - s->y;
    out[0] = s->v;  out[1] = s->delta;
    out[2] = dx1*ct + dy1*st;  out[3] = -dx1*st + dy1*ct;
    out[4] = dx2*ct + dy2*st;  out[5] = -dx2*st + dy2*ct;
    out[6] = dx3*ct + dy3*st;  out[7] = -dx3*st + dy3*ct;
}
