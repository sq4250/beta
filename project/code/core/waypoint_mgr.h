/**
 * waypoint_mgr.h — 航点窗口管理 (调用方持有状态, 显式传递)
 */

#ifndef WAYPOINT_MGR_H
#define WAYPOINT_MGR_H

#include "car_state.h"
#include "config.h"

typedef struct {
    Waypoint wps[MAX_WAYPOINTS];
    u32      count;
    u32      idx;                          // 当前目标索引
    bool     reached[MAX_WAYPOINTS];
} WaypointMgr;

void waypoint_mgr_init(WaypointMgr *mgr, const Waypoint *wps, u32 count);
bool waypoint_mgr_get_window(WaypointMgr *mgr, Waypoint *g1, Waypoint *g2, Waypoint *g3);
void waypoint_mgr_mark_reached(WaypointMgr *mgr);
bool waypoint_mgr_check_hit(const WaypointMgr *mgr, const CarState *prev, const CarState *next);
u32  waypoint_mgr_reached_count(const WaypointMgr *mgr);

#endif
