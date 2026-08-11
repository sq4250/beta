#include "zf_common_headfile.h"
#include <string.h>
#include "tasks.h"
#include "car_comm.h"
#include "core/planner.h"
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

/* ── 已访问信标集合 (MODE 3 专用) ── */
static bool s_visited[BEACON_COUNT];
static u8   s_visited_count;
static u8   s_last_visited;   /* 最后一个访问的信标 slot_id */
static bool s_auto_active;    /* 自主巡点模式 (飞机发新点后退出) */

static void visited_mark(u8 slot_id) {
    if (slot_id < BEACON_COUNT && !s_visited[slot_id]) {
        s_visited[slot_id] = true;
        s_visited_count++;
    }
}
static void visited_clear(void) {
    for (u8 i = 0; i < BEACON_COUNT; i++) s_visited[i] = false;
    s_visited_count = 0;
}
static bool visited_all(void) { return s_visited_count >= BEACON_COUNT; }

/* 加载全部未访问信标 → TSP → 推入队列 */
static void wp_load_unvisited(void) {
    waypoint_t all[BEACON_COUNT];
    u8 n = 0;
    for (u8 i = 0; i < BEACON_COUNT; i++) {
        if (!s_visited[i]) all[n++] = beacon_lookup(i);
    }
    if (n > 1) {
        tsp_solve(all, (const waypoint_t *)all, n, g_car.x, g_car.y);
    }
    for (u8 i = 0; i < n; i++) wp_push(&all[i]);
}

/* ═══════════════════════════════════════════════════════
 * planner_step — 路点跟随核心 (MODE 2/3/4 共用)
 * ═══════════════════════════════════════════════════════ */
static u8 wp_take_n(waypoint_t *out, u8 n) {
    u8 k = 0;
    for (u8 i = 0; i < wp_count() && k < n; i++) {
        waypoint_t w = wp_peek(i);
        if (w.slot_id < BEACON_COUNT && s_visited[w.slot_id]) continue;
        out[k++] = w;
    }
    return k;
}

/* 从信标表补一个离车最近的非 visited 点 */
static void wp_push_nearest(void) {
    f32 best_d2 = 1e12f; u8 best_id = 0xFF;
    for (u8 i = 0; i < BEACON_COUNT; i++) {
        if (s_visited[i]) continue;
        f32 dx = BEACON_WORLD[i][0] * 0.01f - g_car.x;
        f32 dy = BEACON_WORLD[i][1] * 0.01f - g_car.y;
        f32 d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best_id = i; }
    }
    if (best_id < BEACON_COUNT) {
        waypoint_t w = beacon_lookup(best_id);
        wp_push(&w);
    }
}

static void planner_step(void) {
    waypoint_t g[3]; u8 gn = wp_take_n(g, 3);

    /* 队列空 → 自主模式: 加载未访问信标; 全访问过 → 新一轮(留最后访问点) */
    if (!gn) {
        if (s_auto_active) {
            if (visited_all()) { u8 last = s_last_visited; visited_clear(); visited_mark(last); }
            wp_load_unvisited();
            gn = wp_take_n(g, 3);
        }
        if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    }

    if (check_hit_substep(g_car_prev.x, g_car_prev.y, g_car.x, g_car.y, g[0].x, g[0].y, TOL_XY)) {
        s_last_visited = g[0].slot_id;
        visited_mark(g[0].slot_id);
        wp_remove_id(g[0].slot_id);
        gn = wp_take_n(g, 3);
        if (!gn) {
            if (s_auto_active) {
                if (visited_all()) { u8 last = s_last_visited; visited_clear(); visited_mark(last); }
                wp_load_unvisited();
                gn = wp_take_n(g, 3);
            }
        }
        if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    }

    for (u8 i = gn; i < 3; i++) g[i] = g[gn - 1];
    f32 v2 = (gn > 1) ? 1.0f : 0.0f;
    f32 v3 = (gn > 2) ? 1.0f : 0.0f;
    planner_forward(&g_plan, &g_vst, &g[0], &g[1], &g[2], v2, v3);
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
    bool has_new = false;
    for (u8 i = 0; i < rx->wp.n_wp; i++) {
        u8 sid = rx->wp.slot_ids[i];
        waypoint_t w = beacon_lookup(sid);
        bool in = false;
        u8 n = wp_count();
        for (u8 j = 0; j < n; j++) { waypoint_t q = wp_peek(j); if (q.slot_id == sid) { in = true; break; } }
        if (!in) { wp_push(&w); has_new = true; }
    }
    if (has_new) {
        /* 飞机发新点 → 退出自主巡点, 重新处理飞机指令 */
        s_auto_active = false;
        visited_clear();
        u8 n = wp_count();
        if (n > 1) {
            waypoint_t a[WP_QUEUE_SIZE]; for (u8 i = 0; i < n; i++) a[i] = wp_peek(i);
            tsp_solve(a, (const waypoint_t *)a, n, g_car.x, g_car.y);
            wp_clear(); for (u8 i = 0; i < n; i++) wp_push(&a[i]);
        }
        if (!g_wp_active) wp_activate();
    }
}

static void mode3_step(const car_comm_rx_t *rx) {
    mode3_merge(rx);
    if (!g_wp_active) return;

    planner_step();

    /* 航点耗尽 → 刹车停车 (自主巡点暂时关闭, 测试通讯) */
    if (!wp_count()) {
        g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0;
    }
    /*
    if (!wp_count() && !s_auto_active) {
        s_auto_active = true;
        visited_clear();
        visited_mark(s_last_visited);
    }
    */
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
        g_wp_active = false; g_car_prev = wp_of(&g_car); lateral_reset();
        g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0;
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
#if CAR_MODE != 3
    wp_load_beacons();
#endif
    if (s_mode.init) s_mode.init();
}

void task_20hz_planner(void) {
    if (g_ms < STARTUP_DELAY_MS) return;  /* 控制器先站稳, 规划器延后接入 */
    car_comm_rx_t rx = car_comm_get();
    s_mode.step(&rx);
}
