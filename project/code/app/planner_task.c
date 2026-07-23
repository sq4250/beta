#include "zf_common_headfile.h"
#include <string.h>
#include "tasks.h"
#include "car_comm.h"
#include "core/planner.h"
#include "core/kinematics.h"
#include "core/lateral.h"
#include "core/tsp.h"
#include "utils.h"

/* wp_queue */
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
static bool wp_eq(const waypoint_t *a, const waypoint_t *b) { f32 dx = a->x - b->x, dy = a->y - b->y; return dx*dx + dy*dy < TOL_XY*TOL_XY; }
static void wp_remove(const waypoint_t *w) {
    u8 n = wp_count(); waypoint_t k[WP_QUEUE_SIZE]; u8 kn = 0;
    for (u8 i = 0; i < n; i++) { waypoint_t q = wp_peek(i); if (!wp_eq(&q, w)) k[kn++] = q; }
    wp_clear(); for (u8 i = 0; i < kn; i++) wp_push(&k[i]);
}
static inline waypoint_t wp_of(const car_state_t *s) { waypoint_t w = {s->x, s->y}; return w; }

/* ── MODE 1 ── */
#if CAR_MODE == 1
static u32 s_a_seq;
#endif

/* ── MODE 2/4 ── */
#if CAR_MODE == 2 || CAR_MODE == 4
static const waypoint_t s_loc[LOCAL_WP_COUNT] = {
    {1.715f, 0.815f}, {3.445f, 1.43f}, {4.565f, 0.095f},
    {2.965f, -0.075f}, {3.70f, -1.48f}, {1.91f, -1.09f},
};
#endif

/* ── MODE 3 ── */
#if CAR_MODE == 3
static waypoint_t s_vis;
static u32     s_start_s, s_stop_s;

static void m3_merge(const car_comm_rx_t *rx) {
    u8 n = wp_count();
    for (u8 i = 0; i < rx->n_wp; i++) {
        waypoint_t w = {rx->wp[i].x * 0.01f, rx->wp[i].y * 0.01f};
        bool in = false;
        for (u8 j = 0; j < n; j++) { waypoint_t q = wp_peek(j); if (wp_eq(&w, &q)) { in = true; break; } }
        if (!in) { wp_push(&w); n++; }
    }
    if (n > 1) {
        waypoint_t a[WP_QUEUE_SIZE]; for (u8 i = 0; i < n; i++) a[i] = wp_peek(i);
        tsp_solve(a, (const waypoint_t *)a, n, g_car.x, g_car.y);
        wp_clear(); for (u8 i = 0; i < n; i++) wp_push(&a[i]);
    }
    if (!g_wp_active && wp_count()) {
        g_wp_active = true; g_vst = g_car; g_vst_prev = wp_of(&g_car); lateral_reset();
    }
}

static void m3_start_cmd(const car_comm_rx_t *rx) {
    if (rx->start_seq == s_start_s) return;
    s_start_s = rx->start_seq;
    if (!g_wp_active) { g_vst = g_car; g_vst_prev = wp_of(&g_car); lateral_reset(); }
    g_wp_active = true;
}

static void m3_stop_cmd(const car_comm_rx_t *rx) {
    if (rx->stop_seq == s_stop_s) return;
    s_stop_s = rx->stop_seq;
    wp_clear(); g_wp_active = false; g_vst = g_car; g_vst_prev = wp_of(&g_car);
    lateral_reset(); g_plan.a = 0; g_plan.omega = 0;
}
#endif

/* ── MODE 4 ── */
#if CAR_MODE == 4
static u32 s_start_s, s_stop_s;
#endif

/* ── planner_step (MODE 2/3/4 共用) ── */
#if CAR_MODE >= 2

static u8 wp_take_n(waypoint_t *out, u8 n) {
    u8 k = 0;
    for (u8 i = 0; i < wp_count() && k < n; i++) {
        waypoint_t w = wp_peek(i);
#if CAR_MODE == 3
        if (wp_eq(&w, &s_vis)) continue;
#endif
        out[k++] = w;
    }
    return k;
}

static void planner_step(void) {
    waypoint_t g[3]; u8 gn = wp_take_n(g, 3);
    if (!gn) { g_plan.a = 0; g_plan.omega = 0; return; }

    if (check_hit_substep(g_vst_prev.x, g_vst_prev.y, g_vst.x, g_vst.y, g[0].x, g[0].y, TOL_XY)) {
        wp_remove(&g[0]);
#if CAR_MODE == 3
        s_vis = g[0];
#endif
        gn = wp_take_n(g, 3);
        if (!gn) { g_plan.a = 0; g_plan.omega = 0; return; }
    }

    for (u8 i = gn; i < 3; i++) g[i] = g[gn - 1];
    planner_forward(&g_plan, &g_vst, &g[0], &g[1], &g[2]);
    g_vst_prev = wp_of(&g_vst);
}
#endif

void planner_init(void) {
    wp_clear(); g_wp_active = false;
    g_plan.a = 0; g_plan.omega = 0;

#if CAR_MODE == 2 || CAR_MODE == 4
    waypoint_t o[MAX_WAYPOINTS];
    tsp_solve(o, s_loc, LOCAL_WP_COUNT, CAR_START_X, CAR_START_Y);
    wp_push_n(o, LOCAL_WP_COUNT); waypoint_t h = {CAR_START_X, CAR_START_Y}; wp_push(&h);
#endif
#if CAR_MODE == 2
    g_wp_active = true; g_vst = g_car; g_vst_prev = wp_of(&g_car); lateral_reset();
#endif
}

void task_20hz_planner(void) {
    car_comm_rx_t rx = car_comm_get();

#if CAR_MODE == 1
    static u32 stale;
    if (rx.a_seq != s_a_seq) { s_a_seq = rx.a_seq; stale = 0; }
    if (++stale >= 10) rx.a_n = rx.a_w = 0;
    f32 ct = cosf(g_car.theta), st = sinf(g_car.theta);
    g_plan.a = clamp(rx.a_n * ct + rx.a_w * st, -A_BRAKE_MAX, A_LONG_MAX);
    g_plan.omega = 0;

#elif CAR_MODE == 2
    if (!g_wp_active) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    planner_step();

#elif CAR_MODE == 3
    m3_stop_cmd(&rx); m3_merge(&rx); m3_start_cmd(&rx);
    if (g_wp_active) planner_step();

#elif CAR_MODE == 4
    if (rx.stop_seq != s_stop_s) {
        s_stop_s = rx.stop_seq; wp_clear();
        waypoint_t o[MAX_WAYPOINTS];
        tsp_solve(o, s_loc, LOCAL_WP_COUNT, CAR_START_X, CAR_START_Y);
        wp_push_n(o, LOCAL_WP_COUNT); waypoint_t h = {CAR_START_X, CAR_START_Y}; wp_push(&h);
        g_wp_active = false; g_vst = g_car; g_vst_prev = wp_of(&g_car); lateral_reset();
        g_plan.a = 0; g_plan.omega = 0;
    }
    if (rx.start_seq != s_start_s) {
        s_start_s = rx.start_seq;
        g_wp_active = true; g_vst = g_car; g_vst_prev = wp_of(&g_car); lateral_reset();
    }
    if (g_wp_active) planner_step();
#endif
}
