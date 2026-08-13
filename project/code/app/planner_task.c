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

/* ═══════════════════════════════════════════════════════
 * helpers — MODE 2/4 专用 (MODE 3 使用 waypoint_coords)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE != 3

static void wp_remove_id(u8 id) {
    u8 n = wp_count(); waypoint_t k[WP_QUEUE_SIZE]; u8 kn = 0;
    for (u8 i = 0; i < n; i++) { waypoint_t q = wp_peek(i); if (q.slot_id != id) k[kn++] = q; }
    wp_clear(); for (u8 i = 0; i < kn; i++) wp_push(&k[i]);
}

static inline waypoint_t wp_of(const car_state_t *s) { waypoint_t w = {SLOT_HOME, s->x, s->y}; return w; }
static void wp_activate(void) { g_wp_active = true; g_car_prev = wp_of(&g_car); lateral_reset(); }
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

/* ── planner_step — 路点跟随核心 (MODE 2/4 共用) ── */
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

#endif /* CAR_MODE != 3 */

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
 * MODE 3 — 飞机引导 (WP 池 + 探索池 双池串联 + 访问序列)
 *
 *  规划层纯操作 VST 世界:
 *    - 访问序列 s_seq 是访问顺序的唯一真实来源
 *    - 到达检测基于 VST 位置 (非 car 估计量)
 *    - 产出 g_plan → 控制层自己跟踪 VST
 *    - 不依赖 g_car / g_car_prev / lateral_reset
 *
 *  WP 池:  飞机 CMD 0x30 下发, 高优先级, 增量入池触发重建
 *  探索池: 全量信标中不在 WP 池的, 低优先级, 在 WP 池后无缝衔接
 *  序列 = TSP(WP池, vst位置) + TSP(探索池, 末WP / vst位置)
 * ═══════════════════════════════════════════════════════ */
#if CAR_MODE == 3

/* ── WP 池 ── */
static u8   s_wp_pool[BEACON_COUNT];
static u8   s_wp_n;
static u32  s_wp_seq;

/* ── 访问序列 (滑动窗口) ── */
static u8   s_seq[WP_QUEUE_SIZE];  /* 有序 slot_id */
static u8   s_seq_head;            /* 当前窗口首 */
static u8   s_seq_len;             /* 序列总长 */

/* ── VST 线段碰撞检测 ── */
static f32  s_vst_prev_x, s_vst_prev_y;  /* 上一规划帧的 VST 位置 */

/* ── VST 到达检测 ── */
#define VST_HIT_TOL      0.05f   /* WP 点到达圆半径 [m] */
#define EXPLORE_HIT_TOL  0.15f   /* 探索点到达圆半径 [m] */

/* ── 虚拟探索点 ──
 *   凸包: 以信标为顶点, 计算包围所有信标的凸包顶点
 *   探索点: 每个凸包顶点与凸包中心(顶点质心)的中点, 虚拟 ID = MID_BASE + 顶点 slot_id
 *   排除规则: WP TSP 末尾 WP → 排除离它最近的探索点 */
#define MID_BASE   BEACON_COUNT  /* 虚拟 ID 从 BEACON_COUNT 开始, 不碰撞真实信标 */

static u8  s_hull[BEACON_COUNT];   /* 凸包顶点 slot_id (逆时针) */
static u8  s_hull_n;               /* 凸包顶点数 */
static f32 s_mid[BEACON_COUNT][2]; /* 探索点坐标 (m), 索引 = 凸包顶点 slot_id */

/* 二维叉积: (b-a)×(c-a) 的 z 分量 */
static f32 cross2(f32 ax, f32 ay, f32 bx, f32 by, f32 cx, f32 cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/* 凸包顶点 (Andrew monotone chain), 返回顶点数, hull 存顶点 slot_id (逆时针) */
static u8 convex_hull(u8 *hull) {
    u8 idx[BEACON_COUNT];
    for (u8 i = 0; i < BEACON_COUNT; i++) idx[i] = i;

    /* 按 x 升序 (x 同按 y) — 插入排序, 点少足够 */
    for (u8 i = 1; i < BEACON_COUNT; i++) {
        u8 j = i;
        while (j > 0) {
            f32 ax = BEACON_WORLD[idx[j-1]][0], ay = BEACON_WORLD[idx[j-1]][1];
            f32 bx = BEACON_WORLD[idx[j]][0],   by = BEACON_WORLD[idx[j]][1];
            if (ax < bx || (ax == bx && ay < by)) break;
            u8 t = idx[j-1]; idx[j-1] = idx[j]; idx[j] = t;
            j--;
        }
    }

    /* 下凸包 + 上凸包 */
    u8 m = 0;
    for (u8 i = 0; i < BEACON_COUNT; i++) {
        while (m >= 2 && cross2(BEACON_WORLD[hull[m-2]][0], BEACON_WORLD[hull[m-2]][1],
                                BEACON_WORLD[hull[m-1]][0], BEACON_WORLD[hull[m-1]][1],
                                BEACON_WORLD[idx[i]][0],   BEACON_WORLD[idx[i]][1]) <= 0.0f) m--;
        hull[m++] = idx[i];
    }
    u8 lower = m + 1;
    for (i8 i = (i8)BEACON_COUNT - 2; i >= 0; i--) {
        while (m >= lower && cross2(BEACON_WORLD[hull[m-2]][0], BEACON_WORLD[hull[m-2]][1],
                                    BEACON_WORLD[hull[m-1]][0], BEACON_WORLD[hull[m-1]][1],
                                    BEACON_WORLD[idx[i]][0],   BEACON_WORLD[idx[i]][1]) <= 0.0f) m--;
        hull[m++] = idx[i];
    }
    return m - 1;  /* 去掉重复的起始点 */
}

/* 启动时调用: 计算凸包顶点 + 预计算探索点坐标 */
static void midpoints_init(void) {
    s_hull_n = convex_hull(s_hull);
    f32 cx = 0, cy = 0;
    for (u8 i = 0; i < s_hull_n; i++) {
        cx += BEACON_WORLD[s_hull[i]][0];
        cy += BEACON_WORLD[s_hull[i]][1];
    }
    cx /= (f32)s_hull_n; cy /= (f32)s_hull_n;

    /* 探索点 = 凸包顶点与凸包中心的中点 (米) */
    for (u8 i = 0; i < s_hull_n; i++) {
        u8 v = s_hull[i];
        s_mid[v][0] = (cx + BEACON_WORLD[v][0]) * 0.5f * 0.01f;
        s_mid[v][1] = (cy + BEACON_WORLD[v][1]) * 0.5f * 0.01f;
    }
}

/* 统一航点坐标查询: 真实信标 (0..BEACON_COUNT-1) + 探索点 (MID_BASE + 顶点 slot_id) */
static waypoint_t waypoint_coords(u8 slot_id) {
    waypoint_t w = {slot_id, 0, 0};
    if (slot_id < BEACON_COUNT) {
        w.x = BEACON_WORLD[slot_id][0] * 0.01f;
        w.y = BEACON_WORLD[slot_id][1] * 0.01f;
    } else {
        u8 outer = slot_id - MID_BASE;   /* 凸包顶点 slot_id */
        if (outer < BEACON_COUNT) {
            w.x = s_mid[outer][0];
            w.y = s_mid[outer][1];
        }
    }
    return w;
}

/* 从 WP 池中移除 slot_id, 返回是否成功移除 */
static bool wp_pool_remove(u8 slot_id) {
    u8 j = 0;
    for (u8 i = 0; i < s_wp_n; i++)
        if (s_wp_pool[i] != slot_id) s_wp_pool[j++] = s_wp_pool[i];
    if (j < s_wp_n) { s_wp_n = j; return true; }
    return false;
}

/* 重建访问序列: WP 池 TSP + 探索点 TSP → s_seq, 重置窗口
 *
 *  若当前 s_seq head 指向的是 WP 点 → 以该点为 WP 池 TSP 起点,
 *  保持当前目标不变; 否则正常从 VST 出发.
 *
 *  探索池 = 探索点 (中心→信标连线取中点), WP TSP 末位为信标 i 时排除对应那个 */
static void rebuild_sequence(void) {
    /* 重建前: 检查当前 head 是否在 WP 池中 */
    u8   anchor = 0xFF;
    bool anchor_is_wp = false;
    if (s_seq_len > 0 && s_seq_head < s_seq_len) {
        u8 slot = s_seq[s_seq_head];
        for (u8 i = 0; i < s_wp_n; i++)
            if (s_wp_pool[i] == slot) { anchor = slot; anchor_is_wp = true; break; }
    }

    s_seq_len   = 0;
    s_seq_head  = 0;
    waypoint_t a[WP_QUEUE_SIZE];

    /* Phase 1: WP 池 TSP */
    if (anchor_is_wp) {
        /* 锚点置首, 其余 WP 从锚点出发 TSP */
        waypoint_t awp = waypoint_coords(anchor);
        a[0] = awp; u8 k = 1;
        for (u8 i = 0; i < s_wp_n; i++)
            if (s_wp_pool[i] != anchor) a[k++] = waypoint_coords(s_wp_pool[i]);
        if (k > 2)
            tsp_solve(&a[1], (const waypoint_t *)&a[1], k - 1, awp.x, awp.y);
        for (u8 i = 0; i < k; i++)
            s_seq[s_seq_len++] = a[i].slot_id;
    } else {
        /* 正常: 全部 WP 从 VST 出发 TSP */
        for (u8 i = 0; i < s_wp_n; i++)
            a[i] = waypoint_coords(s_wp_pool[i]);
        if (s_wp_n > 1)
            tsp_solve(a, (const waypoint_t *)a, s_wp_n, g_vst.x, g_vst.y);
        for (u8 i = 0; i < s_wp_n; i++)
            s_seq[s_seq_len++] = a[i].slot_id;
    }

    /* Phase 2: 探索点 TSP
     *   探索点 = 凸包顶点与凸包中心的中点, ID = MID_BASE + 顶点 slot_id
     *   排除离 last_wp 最近的探索点, 从 last_wp 出发 TSP */
    u8 last_wp = (s_wp_n > 0) ? a[s_wp_n - 1].slot_id : 0xFF;

    u8 mids[BEACON_COUNT], mid_n = 0;
    for (u8 i = 0; i < s_hull_n; i++)
        mids[mid_n++] = MID_BASE + s_hull[i];

    /* 排除离 last_wp 最近的探索点 */
    if (last_wp < BEACON_COUNT && mid_n > 0) {
        f32 lwx = a[s_wp_n - 1].x, lwy = a[s_wp_n - 1].y;
        u8  best_j = 0;
        f32 best_d2 = 1e12f;
        for (u8 j = 0; j < mid_n; j++) {
            waypoint_t mw = waypoint_coords(mids[j]);
            f32 dx = mw.x - lwx, dy = mw.y - lwy;
            f32 d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_j = j; }
        }
        for (u8 j = best_j; j + 1 < mid_n; j++) mids[j] = mids[j + 1];
        mid_n--;
    }

    if (mid_n > 0) {
        f32 sx = (s_wp_n > 0) ? a[s_wp_n - 1].x : g_vst.x;
        f32 sy = (s_wp_n > 0) ? a[s_wp_n - 1].y : g_vst.y;
        for (u8 i = 0; i < mid_n; i++)
            a[i] = waypoint_coords(mids[i]);
        if (mid_n > 1)
            tsp_solve(a, (const waypoint_t *)a, mid_n, sx, sy);
        for (u8 i = 0; i < mid_n; i++)
            s_seq[s_seq_len++] = a[i].slot_id;
    }
}

/* MODE 3 规划步进: 访问序列 → 取 3 航点 → NN 前向 */
static void mode3_planner_step(void) {
    /* 从 s_seq_head 开始取最多 3 个航点 */
    waypoint_t g[3]; u8 gn = 0;
    for (u8 i = s_seq_head; i < s_seq_len && gn < 3; i++)
        g[gn++] = waypoint_coords(s_seq[i]);

    if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }

    u8   cur_slot  = s_seq[s_seq_head];
    bool cur_is_wp = (cur_slot < BEACON_COUNT);   /* <7=真实信标(WP), >=7=探索点 */

    /* VST 到达检测: WP 点 5cm 圆, 探索点 15cm 圆 */
    waypoint_t target = waypoint_coords(cur_slot);
    f32  hit_tol = cur_is_wp ? VST_HIT_TOL : EXPLORE_HIT_TOL;
    if (check_hit_substep(s_vst_prev_x, s_vst_prev_y, g_vst.x, g_vst.y, target.x, target.y, hit_tol)) {
        u8 slot = cur_slot;
        s_seq_head++;  /* 滑窗 */

        /* WP 点 → 从池移除变探索点 (不追加到队尾, 下次重建时自动排入) */
        wp_pool_remove(slot);

        /* 刷新航点视图 */
        gn = 0;
        for (u8 i = s_seq_head; i < s_seq_len && gn < 3; i++)
            g[gn++] = waypoint_coords(s_seq[i]);

        /* 到达事件打印 */
        printf("ARR:{%u} POOL:{", slot);
        for (u8 i = 0; i < s_wp_n; i++) { if (i) printf(","); printf("%u", s_wp_pool[i]); }
        printf("} SEQ:[");
        for (u8 i = 0; i < s_seq_len; i++) {
            if (i) printf(",");
            if (i == s_seq_head) printf(">");
            printf("%u", s_seq[i]);
        }
        printf("] WIN:[");
        u8 wc = 0;
        for (u8 i = s_seq_head; i < s_seq_len && wc < 3; i++, wc++) {
            if (wc) printf(",");
            printf("%u", s_seq[i]);
        }
        while (wc < 3) { if (wc) printf(","); printf("x"); wc++; }
        printf("]\r\n");

        if (!gn) { g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0; return; }
    }

    /* 门控填充: 不足 3 个航点用最后一个补齐, gate=0 表示无效 */
    for (u8 i = gn; i < 3; i++) g[i] = g[gn - 1];
    f32 v2 = (gn > 1) ? 1.0f : 0.0f;
    f32 v3 = (gn > 2) ? 1.0f : 0.0f;
    /* 混合模型: head 是 WP → D5, 探索 → Kamm */
#if NN_MODEL_VERSION == NN_VER_HYBRID
    {
        bool is_wp = false;
        u8 head_slot = s_seq[s_seq_head];
        for (u8 i = 0; i < s_wp_n; i++)
            if (s_wp_pool[i] == head_slot) { is_wp = true; break; }
        planner_forward_hybrid(&g_plan, &g_vst, &g[0], &g[1], &g[2], v2, v3, is_wp);
    }
#else
    planner_forward(&g_plan, &g_vst, &g[0], &g[1], &g[2], v2, v3);
#endif

    /* 记录本帧 VST 位置, 供下一帧线段碰撞检测 */
    s_vst_prev_x = g_vst.x; s_vst_prev_y = g_vst.y;
}

/* 飞机 WP 帧合并: 增量入池 + 触发重建 */
static void mode3_merge(const car_comm_rx_t *rx) {
    if (rx->wp.wp_seq == s_wp_seq) return;
    s_wp_seq = rx->wp.wp_seq;

    bool has_new = false;
    for (u8 i = 0; i < rx->wp.n_wp; i++) {
        bool in = false;
        for (u8 j = 0; j < s_wp_n; j++)
            if (s_wp_pool[j] == rx->wp.slot_ids[i]) { in = true; break; }
        if (!in && s_wp_n < BEACON_COUNT) {
            s_wp_pool[s_wp_n++] = rx->wp.slot_ids[i];
            has_new = true;
        }
    }

    if (has_new) {
        rebuild_sequence();

        /* 增点成功 → 打印飞机指令 + 增点后池 + 重建后序列 + 新窗口 */
        printf("CMD:{");
        for (u8 i = 0; i < rx->wp.n_wp; i++) { if (i) printf(","); printf("%u", rx->wp.slot_ids[i]); }
        printf("} POOL:{");
        for (u8 i = 0; i < s_wp_n; i++) { if (i) printf(","); printf("%u", s_wp_pool[i]); }
        printf("} SEQ:[");
        for (u8 i = 0; i < s_seq_len; i++) {
            if (i) printf(",");
            if (i == s_seq_head) printf(">");
            printf("%u", s_seq[i]);
        }
        printf("] WIN:[");
        u8 wc = 0;
        for (u8 i = s_seq_head; i < s_seq_len && wc < 3; i++, wc++) {
            if (wc) printf(",");
            printf("%u", s_seq[i]);
        }
        while (wc < 3) { if (wc) printf(","); printf("x"); wc++; }
        printf("]\r\n");
    }
}

static void mode3_step(const car_comm_rx_t *rx) {
    mode3_merge(rx);
    mode3_planner_step();
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
    g_plan.a = -A_BRAKE_MAX; g_plan.omega = 0;
#if CAR_MODE == 3
    midpoints_init();
#else
    wp_load_beacons();
#endif
    if (s_mode.init) s_mode.init();
}

void task_20hz_planner(void) {
    if (g_ms < STARTUP_DELAY_MS) return;  /* 控制器先站稳, 规划器延后接入 */
    car_comm_rx_t rx = car_comm_get();
    s_mode.step(&rx);
}
