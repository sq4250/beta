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

// 线段-圆相交检测
bool check_hit_substep(const Waypoint *prev, const Waypoint *next, const Waypoint *target, f32 tol);

#endif
