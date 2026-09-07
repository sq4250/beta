#include "zf_common_headfile.h"
#include <string.h>
#include "tasks.h"
#include "core/planner.h"
#include "core/lateral.h"
#include "core/tsp.h"
#include "utils.h"

/* ═══════════════════════════════════════════════════════
 * planner_task.c — 补录分支规划层 (20Hz)
 *
 * 飞机交互 (0x10/0x30/0x31/0x32 接收) 与目标选择
 * (WP/探索双池、凸包、混合模型) 已剥离, 仅保留:
 *   MODE 2 FULL_AUTO — 本地 6 航点 TSP → 上电自跑
 *   (MODE 1 位置阶跃测试不经过规划层, 逻辑在 tasks.c)
 * ═══════════════════════════════════════════════════════ */

/* ── wp_queue — 环形路点缓冲 ── */
#define WP_MASK  (WP_QUEUE_SIZE - 1)
static waypoint_t    s_wp[WP_QUEUE_SIZE];
static volatile u8 s_head;
static u8          s_tail;

void wp_clear(void)        { s_head = s_tail = 0; }
u8   wp_count(void)        { return (s_head - s_tail) & WP_MASK; }
void wp_push(const waypoint_t *w) { s_wp[s_head] = *w; s_head = (s_head + 1) & WP_MASK; }
void wp_push_n(const waypoint_t *ws, u8 n) { for (u8 i = 0; i < n; i++) wp_push(&ws[i]); }
waypoint_t wp_peek(u8 off)   { return s_wp[(s_tail + off) & WP_MASK]; }
void wp_pop(void)          { s_tail = (s_tail + 1) & WP_MASK; }
#define SLOT_HOME  0xFF

/* ═══════════════════════════════════════════════════════
 * MODE 2 — 本地 6 航点 TSP 自跑 (上电即跑)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE == 2

static void wp_remove_id(u8 id) {
    u8 n = wp_count(); waypoint_t k[WP_QUEUE_SIZE]; u8 kn = 0;
    for (u8 i = 0; i < n; i++) { waypoint_t q = wp_peek(i); if (q.slot_id != id) k[kn++] = q; }
    wp_clear(); for (u8 i = 0; i < kn; i++) wp_push(&k[i]);
}

static inline waypoint_t wp_of(const car_state_t *s) { waypoint_t w = {SLOT_HOME, s->x, s->y}; return w; }
static void wp_activate(void) { g_wp_active = true; g_car_prev = wp_of(&g_car); lateral_reset(); }

static waypoint_t wp_from_table(u8 id) {
    waypoint_t w = {id, 0, 0};
    if (id < BEACON_COUNT) {
        w.x = BEACON_WORLD[id][0] * 0.01f;
        w.y = BEACON_WORLD[id][1] * 0.01f;
    }
    return w;
}

static void wp_load_beacons(void) {
    waypoint_t all[MAX_WAYPOINTS];
    for (u8 i = 0; i < BEACON_COUNT; i++) all[i] = wp_from_table(i);
    tsp_solve(all, (const waypoint_t *)all, BEACON_COUNT, CAR_START_X, CAR_START_Y);
    wp_push_n(all, BEACON_COUNT);
    waypoint_t h = {SLOT_HOME, CAR_START_X, CAR_START_Y};
    wp_push(&h);
}

/* ── planner_step — 路点跟随核心: 到达弹窗 → 取 3 点 → NN ── */
static u8 wp_take_n(waypoint_t *out, u8 n) {
    u8 k = 0;
    for (u8 i = 0; i < wp_count() && k < n; i++)
        out[k++] = wp_peek(i);
    return k;
}

static void planner_step(void) {
    waypoint_t g[3]; u8 gn = wp_take_n(g, 3);
    if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }

    if (check_hit_substep(g_car_prev.x, g_car_prev.y, g_car.x, g_car.y, g[0].x, g[0].y, TOL_XY)) {
        wp_remove_id(g[0].slot_id);
        gn = wp_take_n(g, 3);
        if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    }

    for (u8 i = gn; i < 3; i++) g[i] = g[gn - 1];
    f32 v2 = (gn > 1) ? 1.0f : 0.0f;
    f32 v3 = (gn > 2) ? 1.0f : 0.0f;
    planner_forward(&g_plan, &g_vst, &g[0], &g[1], &g[2], v2, v3);
    g_car_prev = wp_of(&g_car);
}

static void mode2_init(void) { wp_activate(); }

static void mode2_step(void) {
    if (!g_wp_active) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    planner_step();
}

#endif

/* ═══════════════════════════════════════════════════════
 * PUBLIC — dispatch (仅 MODE 2; MODE 1 无规划层)
 * ═══════════════════════════════════════════════════════ */
void planner_init(void) {
    wp_clear(); g_wp_active = false;
    g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0;
#if CAR_MODE == 2
    wp_load_beacons();
    mode2_init();
#endif
}

void task_20hz_planner(void) {
    if (g_ms < STARTUP_DELAY_MS) return;  /* 控制器先站稳, 规划器延后接入 */
#if CAR_MODE == 2
    mode2_step();
#endif
}
