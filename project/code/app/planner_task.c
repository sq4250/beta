#include "zf_common_headfile.h"
#include <string.h>
#include "tasks.h"
#include "car_comm.h"
#include "core/planner.h"
#include "core/kinematics.h"
#include "core/lateral.h"
#include "core/tsp.h"
#include "utils.h"

/* ═══════════════════════════════════════════════════════
 * wp_queue — 环形路点缓冲
 * ═══════════════════════════════════════════════════════ */
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

static void wp_remove_id(u8 id) {
    u8 n = wp_count(); waypoint_t k[WP_QUEUE_SIZE]; u8 kn = 0;
    for (u8 i = 0; i < n; i++) { waypoint_t q = wp_peek(i); if (q.slot_id != id) k[kn++] = q; }
    wp_clear(); for (u8 i = 0; i < kn; i++) wp_push(&k[i]);
}

/* ═══════════════════════════════════════════════════════
 * helpers
 * ═══════════════════════════════════════════════════════ */
static inline waypoint_t wp_of(const car_state_t *s) { waypoint_t w = {SLOT_HOME, s->x, s->y}; return w; }

static void wp_activate(void) { g_wp_active = true; g_vst = g_car; g_car_prev = wp_of(&g_car); lateral_reset(); }

static waypoint_t beacon_lookup(u8 slot_id) {
    waypoint_t w = {slot_id, 0, 0};
    if (slot_id < BEACON_COUNT) {
        w.x = BEACON_WORLD[slot_id][0] * 0.01f;
        w.y = BEACON_WORLD[slot_id][1] * 0.01f;
    }
    return w;
}

static waypoint_t wp_from_table(u8 id) { return beacon_lookup(id); }

static void wp_load_beacons(void) {
    waypoint_t all[MAX_WAYPOINTS];
    for (u8 i = 0; i < BEACON_COUNT; i++) all[i] = wp_from_table(i);
    tsp_solve(all, (const waypoint_t *)all, BEACON_COUNT, CAR_START_X, CAR_START_Y);
    wp_push_n(all, BEACON_COUNT);
    waypoint_t h = {SLOT_HOME, CAR_START_X, CAR_START_Y};
    wp_push(&h);
}

/* visited-beacon (MODE 3 专用, 其余 mode 下 s_vis_valid 恒为 false) */
static u8   s_vis_id;
static bool s_vis_valid;

/* ═══════════════════════════════════════════════════════
 * planner_step — 路点跟随核心 (MODE 2/3/4 共用)
 * ═══════════════════════════════════════════════════════ */
static u8 wp_take_n(waypoint_t *out, u8 n) {
    u8 k = 0;
    for (u8 i = 0; i < wp_count() && k < n; i++) {
        waypoint_t w = wp_peek(i);
        if (s_vis_valid && w.slot_id == s_vis_id) continue;
        out[k++] = w;
    }
    return k;
}

static void planner_step(void) {
    waypoint_t g[3]; u8 gn = wp_take_n(g, 3);
    if (!gn) { g_plan.a = 0; g_plan.omega = 0; return; }

    if (check_hit_substep(g_car_prev.x, g_car_prev.y, g_car.x, g_car.y, g[0].x, g[0].y, TOL_XY)) {
        wp_remove_id(g[0].slot_id);
        s_vis_id = g[0].slot_id; s_vis_valid = true;
        gn = wp_take_n(g, 3);
        if (!gn) { g_plan.a = 0; g_plan.omega = 0; return; }
    }

    for (u8 i = gn; i < 3; i++) g[i] = g[gn - 1];
    planner_forward(&g_plan, &g_vst, &g[0], &g[1], &g[2]);
    g_car_prev = wp_of(&g_car);
}

/* ═══════════════════════════════════════════════════════
 * MODE 2 — 自主巡线 (信标表, 上电即跑)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE == 2

static void mode2_init(void) { wp_activate(); }

static void mode2_step(const car_comm_rx_t *rx) {
    (void)rx;
    if (!g_wp_active) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    planner_step();
}

#endif

/* ═══════════════════════════════════════════════════════
 * MODE 3 — 飞机引导 (收 WP 帧 → 合并+TSP → 激活)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE == 3

static void mode3_merge(const car_comm_rx_t *rx) {
    u8 n = wp_count();
    for (u8 i = 0; i < rx->wp.n_wp; i++) {
        u8 sid = rx->wp.slot_ids[i];
        waypoint_t w = beacon_lookup(sid);
        bool in = false;
        for (u8 j = 0; j < n; j++) { waypoint_t q = wp_peek(j); if (q.slot_id == sid) { in = true; break; } }
        if (!in) { wp_push(&w); n++; }
    }
    if (n > 1) {
        waypoint_t a[WP_QUEUE_SIZE]; for (u8 i = 0; i < n; i++) a[i] = wp_peek(i);
        tsp_solve(a, (const waypoint_t *)a, n, g_car.x, g_car.y);
        wp_clear(); for (u8 i = 0; i < n; i++) wp_push(&a[i]);
    }
    if (!g_wp_active && wp_count()) { wp_activate(); }
}

static void mode3_step(const car_comm_rx_t *rx) {
    mode3_merge(rx);
    if (g_wp_active) planner_step();
}

#endif

/* ═══════════════════════════════════════════════════════
 * MODE 4 — 飞机遥控 (START / STOP 命令)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE == 4

static u32 s_start_s, s_stop_s;

static void mode4_step(const car_comm_rx_t *rx) {
    if (rx->stop_seq != s_stop_s) {
        s_stop_s = rx->stop_seq; wp_clear();
        wp_load_beacons();
        g_wp_active = false; g_vst = g_car; g_car_prev = wp_of(&g_car); lateral_reset();
        g_plan.a = 0; g_plan.omega = 0;
    }
    if (rx->start_seq != s_start_s) {
        s_start_s = rx->start_seq;
        wp_activate();
    }
    if (g_wp_active) planner_step();
}

#endif

/* ═══════════════════════════════════════════════════════
 * PUBLIC — dispatch
 * ═══════════════════════════════════════════════════════ */
typedef void (*init_fn)(void);
typedef void (*step_fn)(const car_comm_rx_t *);

static const struct {
    init_fn init;
    step_fn step;
} s_mode = {
#if CAR_MODE == 2
    mode2_init, mode2_step
#elif CAR_MODE == 3
    NULL,       mode3_step
#elif CAR_MODE == 4
    NULL,       mode4_step
#endif
};

void planner_init(void) {
    wp_clear(); g_wp_active = false;
    g_plan.a = 0; g_plan.omega = 0;
    wp_load_beacons();
    if (s_mode.init) s_mode.init();
}

void task_20hz_planner(void) {
    car_comm_rx_t rx = car_comm_get();
    s_mode.step(&rx);
}
