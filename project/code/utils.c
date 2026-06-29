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

bool check_hit_substep(f32 px, f32 py, f32 nx, f32 ny, f32 tx, f32 ty, f32 tol) {
    if ((nx-tx)*(nx-tx) + (ny-ty)*(ny-ty) < tol*tol) return true;

    f32 abx = nx - px, aby = ny - py;
    f32 ab2 = abx*abx + aby*aby;
    if (ab2 < 1e-12f) return false;

    f32 t = ((tx-px)*abx + (ty-py)*aby) / ab2;
    t = clamp(t, 0.0f, 1.0f);
    f32 cx = px + t*abx, cy = py + t*aby;
    return (tx-cx)*(tx-cx) + (ty-cy)*(ty-cy) < tol*tol;
}
