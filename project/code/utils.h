/**
 * utils.h — 通用工具函数
 */
#ifndef UTILS_H
#define UTILS_H
#include "common.h"
#include "config.h"
#include <math.h>

#define clamp(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

f32 wrap_pi(f32 rad);
f32 bicycle_curvature(f32 v, f32 delta);

// 线段 prev→next 是否穿过以 (tx,ty) 为圆心 tol 为半径的圆
bool check_hit_substep(f32 px, f32 py, f32 nx, f32 ny, f32 tx, f32 ty, f32 tol);

#endif
