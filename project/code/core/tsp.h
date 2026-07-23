/**
 * tsp.h — 最近邻贪心 TSP 排序 (O(n²), n≤16)
 *
 * 从小车世界初始位置出发, 依次访问所有目标点
 */
#ifndef TSP_H
#define TSP_H
#include "common.h"
#include "config.h"

/* ── 最近邻贪心: start → nearest → next nearest → ... → 所有点 ── */
void tsp_solve(waypoint_t *ordered, const waypoint_t *unordered, u32 count,
               f32 start_x, f32 start_y);

#endif
