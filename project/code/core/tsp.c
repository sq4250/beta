/**
 * tsp.c — 最近邻贪心 TSP 排序
 */
#include "tsp.h"
#include <string.h>

void tsp_solve(waypoint_t *ordered, const waypoint_t *unordered, u32 count,
               f32 start_x, f32 start_y) {
    if (count == 0) return;
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;

    /* 拷贝输入: 调用方可能传 ordered==unordered, 直接读会 aliasing */
    waypoint_t in[MAX_WAYPOINTS];
    for (u32 i = 0; i < count; i++) in[i] = unordered[i];

    bool visited[MAX_WAYPOINTS];
    memset(visited, 0, sizeof(visited));

    f32 cur_x = start_x, cur_y = start_y;

    for (u32 i = 0; i < count; ++i) {
        u32  best_j = 0;
        f32  best_d = 1e9f;
        for (u32 j = 0; j < count; ++j) {
            if (visited[j]) continue;
            f32 dx = in[j].x - cur_x;
            f32 dy = in[j].y - cur_y;
            f32 d  = dx * dx + dy * dy;
            if (d < best_d) { best_d = d; best_j = j; }
        }
        ordered[i] = in[best_j];
        visited[best_j] = true;
        cur_x = ordered[i].x;
        cur_y = ordered[i].y;
    }
}
