/**
 * waypoint_mgr.c — 航点窗口管理实现
 */
#include "waypoint_mgr.h"
#include "utils.h"
#include <string.h>

void waypoint_mgr_init(WaypointMgr *mgr, const Waypoint *wps, u32 count) {
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    memcpy(mgr->wps, wps, count * sizeof(Waypoint));
    mgr->count = count;
    mgr->idx   = 0;
    for (u32 i = 0; i < MAX_WAYPOINTS; ++i) mgr->reached[i] = false;
}

bool waypoint_mgr_get_window(WaypointMgr *mgr, Waypoint *g1, Waypoint *g2, Waypoint *g3) {
    while (mgr->idx < mgr->count && mgr->reached[mgr->idx]) mgr->idx++;
    if (mgr->idx >= mgr->count) return false;

    u32 ci  = mgr->idx;
    u32 ni  = (ci+1 < mgr->count) ? ci+1 : mgr->count-1;
    u32 n2i = (ci+2 < mgr->count) ? ci+2 : mgr->count-1;
    *g1 = mgr->wps[ci];
    *g2 = mgr->wps[ni];
    *g3 = mgr->wps[n2i];
    return true;
}

void waypoint_mgr_mark_reached(WaypointMgr *mgr) {
    if (mgr->idx < mgr->count && !mgr->reached[mgr->idx]) {
        mgr->reached[mgr->idx] = true;
    }
}

bool waypoint_mgr_check_hit(const WaypointMgr *mgr, const CarState *prev, const CarState *next) {
    if (mgr->idx >= mgr->count || mgr->reached[mgr->idx]) return false;
    const Waypoint *t = &mgr->wps[mgr->idx];
    return check_hit_substep(prev->x, prev->y, next->x, next->y, t->x, t->y, TOL_XY);
}

u32 waypoint_mgr_reached_count(const WaypointMgr *mgr) {
    u32 n = 0;
    for (u32 i = 0; i < mgr->count; ++i) if (mgr->reached[i]) n++;
    return n;
}
