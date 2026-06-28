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

// 线段-圆相交检测: 线段 prev→next 是否与以 target 为圆心 tol 为半径的圆相交
bool check_hit_substep(const f32 prev[2], const f32 next[2], const f32 target[2], f32 tol);

#endif
