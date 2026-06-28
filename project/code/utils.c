/**
 * utils.c — 通用工具函数
 */
#include "utils.h"

f32 wrap_pi(f32 rad) {
    while (rad >  M_PI_F) rad -= 2.0f * M_PI_F;
    while (rad < -M_PI_F) rad += 2.0f * M_PI_F;
    return rad;
}

f32 bicycle_curvature(f32 v, f32 delta) {
    return v * tanf(delta) * INV_WHEELBASE;
}

bool check_hit_substep(const f32 prev[2], const f32 next[2], const f32 target[2], f32 tol) {
    f32 bx = next[0], by = next[1];
    f32 px = target[0], py = target[1];
    if ((bx-px)*(bx-px) + (by-py)*(by-py) < tol*tol) return true;

    f32 abx = bx - prev[0], aby = by - prev[1];
    f32 ab2 = abx*abx + aby*aby;
    if (ab2 < 1e-12f) return false;

    f32 t = ((px-prev[0])*abx + (py-prev[1])*aby) / ab2;
    t = clamp(t, 0.0f, 1.0f);
    f32 cx = prev[0] + t*abx, cy = prev[1] + t*aby;
    return (px-cx)*(px-cx) + (py-cy)*(py-cy) < tol*tol;
}
