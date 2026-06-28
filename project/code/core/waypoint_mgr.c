/**
 * waypoint_mgr.c — 航点窗口管理实现
 */
#include "waypoint_mgr.h"
#include "utils.h"
#include <string.h>

static Waypoint g_waypoints[MAX_WAYPOINTS];
static u32 g_wp_count = 0;
static u32 g_wp_idx = 0;
static bool g_reached[MAX_WAYPOINTS];

void waypoint_mgr_init(const Waypoint *wps, u32 count) {
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    memcpy(g_waypoints, wps, count * sizeof(Waypoint));
    g_wp_count = count;
    g_wp_idx = 0;
    for (u32 i = 0; i < MAX_WAYPOINTS; ++i) g_reached[i] = false;
}

bool waypoint_mgr_get_window(Waypoint *g1, Waypoint *g2, Waypoint *g3) {
    while (g_wp_idx < g_wp_count && g_reached[g_wp_idx]) g_wp_idx++;
    if (g_wp_idx >= g_wp_count) return false;

    u32 ci = g_wp_idx;
    u32 ni = (ci+1 < g_wp_count) ? ci+1 : g_wp_count-1;
    u32 n2i= (ci+2 < g_wp_count) ? ci+2 : g_wp_count-1;
    *g1 = g_waypoints[ci];
    *g2 = g_waypoints[ni];
    *g3 = g_waypoints[n2i];
    return true;
}

void waypoint_mgr_mark_reached(void) {
    if (g_wp_idx < g_wp_count && !g_reached[g_wp_idx]) {
        g_reached[g_wp_idx] = true;
    }
}

bool waypoint_mgr_check_hit(const f32 prev[2], const f32 next[2]) {
    if (g_wp_idx >= g_wp_count || g_reached[g_wp_idx]) return false;
    return check_hit_substep(prev, next,
        (f32[]){g_waypoints[g_wp_idx].x, g_waypoints[g_wp_idx].y}, TOL_XY);
}

u32 waypoint_mgr_reached_count(void) {
    u32 n = 0;
    for (u32 i = 0; i < g_wp_count; ++i) if (g_reached[i]) n++;
    return n;
}

