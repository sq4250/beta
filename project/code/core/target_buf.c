/**
 * target_buf.c — 纯视觉多目标观测缓冲 (全局贪心 GNN)
 *
 * 状态: EMPTY / CAND(备选) / ACTIVE(活跃=WP) / INACTIVE(非活跃=探索)
 *
 * predict (200Hz ISR): P += Q (静止先验) — 只死推, 不记账
 * observe (有新鲜测量): 全局贪心 GNN + KF + 帧末 score 判候选生死 (+PROMOTE/-DROP)
 * vst    (200Hz ISR): ACTIVE 不再匹配 + 经过 → INACTIVE
 * fill   (20Hz 主循环): 活跃点锚点置首 + TSP 排序 (关中断保护, 防与 ISR 竞争)
 */
#include "target_buf.h"
#include "config.h"
#include "tsp.h"
#include "utils.h"
#include <math.h>

#define TRK_MAX       8
#define TRK_MEAS_MAX  8
#define TRK_PROMOTE   5          /* 转正门槛: score 到 +5 → CAND→ACTIVE */
#define TRK_DROP      10         /* 淘汰门槛: score 到 -10 → CAND→EMPTY (更耐心) */
#define TRK_GATE_A    1.0f       /* 门限截距 (m) */
#define TRK_GATE_B    0.4f       /* 门限斜率 (每米放宽) */
#define TRK_Q         0.0001f    /* 随机游走 (m²/帧, 5ms 步长) */
#define TRK_R         0.01f      /* 量测噪声 (m²) */
#define TRK_ARRIVE    TOL_XY     /* VST 到达判定 (m) */
#define FILL_MAX      3          /* NN 输入最大航点数 (g1/g2/g3) */

typedef enum {
    TRK_EMPTY = 0,
    TRK_CAND,
    TRK_ACTIVE,
    TRK_INACTIVE,
} trk_state_t;

typedef struct {
    trk_state_t state;
    u8   id;          /* 递增序号 (正式点分配, 永久) */
    f32  x, y;        /* 位置估计 (世界坐标 m) */
    f32  p;           /* 协方差 (对角标量) */
    i8   score;       /* 生死分: 命中+1/未命中-1, +TRK_PROMOTE 转正 / -TRK_DROP 淘汰 */
    bool matched;     /* 本帧是否被匹配 */
} trk_t;

typedef struct { f32 d2; u8 ti, mi; } heap_ent_t;   /* GNN 关联代价堆元素 */

static trk_t s_tgt[TRK_MAX];
static u8    s_id_cnt;
static u8    s_anchor_id;   /* 当前锚点 (上一帧队首 id) */
static f32   s_prev_vx, s_prev_vy;   /* 上一帧 VST (线段碰撞用) */
static bool  s_prev_valid;

static u8    s_queue_ids[TRK_MAX];   /* 缓存的有序 active id 队列 */
static u8    s_queue_len;            /* 队列长度 */
static bool  s_dirty;                /* 需要重建队列 */

void tracker_init(void) {
    for (u8 i = 0; i < TRK_MAX; i++) s_tgt[i].state = TRK_EMPTY;
    s_id_cnt     = 0;
    s_anchor_id  = 0xFF;
    s_prev_valid = false;
    s_queue_len  = 0;
    s_dirty      = true;   /* 首次重建 */
}

static i8 find_empty(void) {
    for (u8 i = 0; i < TRK_MAX; i++)
        if (s_tgt[i].state == TRK_EMPTY) return (i8)i;
    return -1;
}

/* ═══ predict (200Hz ISR): 仅静止先验死推 P += Q, 不碰 hit/miss ═══ */
void tracker_predict(void) {
    for (u8 i = 0; i < TRK_MAX; i++) {
        trk_t *t = &s_tgt[i];
        if (t->state == TRK_EMPTY) continue;
        t->p += TRK_Q;   /* 静止 + 随机游走 */
    }
}

/* ═══ observe: 全局贪心 GNN + 帧末 score 判候选生死 (有新鲜测量时调用) ═══
 *   score 单一计数器: 命中 +1 / 未命中 -1, +TRK_PROMOTE 转正 / -TRK_DROP 淘汰 */
void tracker_observe(const f32 *zx, const f32 *zy, u8 n,
                     f32 vst_x, f32 vst_y) {
    if (n == 0) return;
    if (n > TRK_MEAS_MAX) n = TRK_MEAS_MAX;

    /* 清本帧匹配标记 (上一帧结果作废) */
    for (u8 ti = 0; ti < TRK_MAX; ti++) {
        if (s_tgt[ti].state != TRK_EMPTY) s_tgt[ti].matched = false;
    }

    /* 构建所有 (目标, 测量) 对, 过滤新息 < 门限 */
    heap_ent_t heap[TRK_MAX * TRK_MEAS_MAX];
    u8 np = 0;
    for (u8 ti = 0; ti < TRK_MAX; ti++) {
        trk_t *t = &s_tgt[ti];
        if (t->state == TRK_EMPTY) continue;

        /* 距离仿射门限: gate = A + B * dist(目标→VST) */
        f32 dx = t->x - vst_x, dy = t->y - vst_y;
        f32 dist_t = sqrtf(dx * dx + dy * dy);
        f32 gate   = TRK_GATE_A + TRK_GATE_B * dist_t;
        f32 gate2  = gate * gate;

        for (u8 mi = 0; mi < n; mi++) {
            f32 ex = zx[mi] - t->x, ey = zy[mi] - t->y;
            f32 d2 = ex * ex + ey * ey;
            if (d2 < gate2) {
                heap[np].d2 = d2; heap[np].ti = ti; heap[np].mi = mi;
                np++;
            }
        }
    }

    /* 插入排序 d2 升序 */
    for (u8 i = 1; i < np; i++) {
        heap_ent_t key = heap[i];
        i8 j = (i8)i - 1;
        while (j >= 0 && heap[j].d2 > key.d2) { heap[j + 1] = heap[j]; j--; }
        heap[j + 1] = key;
    }

    /* 贪心: 测量不重用, 目标可吸收多条 */
    bool m_used[TRK_MEAS_MAX];
    for (u8 mi = 0; mi < n; mi++) m_used[mi] = false;

    for (u8 k = 0; k < np; k++) {
        u8 ti = heap[k].ti, mi = heap[k].mi;
        if (m_used[mi]) continue;

        trk_t *t = &s_tgt[ti];
        f32 S = t->p + TRK_R;
        f32 K = t->p / S;
        t->x += K * (zx[mi] - t->x);
        t->y += K * (zy[mi] - t->y);
        t->p  = (1.0f - K) * t->p;
        t->matched = true;

        if (t->state == TRK_INACTIVE) {
            t->state = TRK_ACTIVE;      /* 灯再亮 → 重新活跃 (无门槛, id 保留) */
            s_dirty  = true;
        }
        m_used[mi] = true;
    }

    /* 落单测量 → 新建候选 (score=0, 帧末 +1 后记 1 次命中) */
    for (u8 mi = 0; mi < n; mi++) {
        if (m_used[mi]) continue;
        i8 ei = find_empty();
        if (ei < 0) break;
        s_tgt[ei].state   = TRK_CAND;
        s_tgt[ei].x = zx[mi]; s_tgt[ei].y = zy[mi];
        s_tgt[ei].p      = TRK_R;
        s_tgt[ei].score  = 0;
        s_tgt[ei].matched = true;
    }

    /* 帧末: 按本帧 matched 更新 score, 判候选生死 (+TRK_PROMOTE 转正 / -TRK_DROP 淘汰) */
    for (u8 ti = 0; ti < TRK_MAX; ti++) {
        trk_t *t = &s_tgt[ti];
        if (t->state == TRK_EMPTY) continue;

        if (t->matched) { if (t->score <  TRK_PROMOTE) t->score++; }
        else            { if (t->score > -TRK_DROP)    t->score--; }

        if (t->state == TRK_CAND) {
            if (t->score >=  TRK_PROMOTE) {
                t->state = TRK_ACTIVE;      /* 候选转正 */
                t->id    = s_id_cnt++;
                s_dirty  = true;
            } else if (t->score <= -TRK_DROP) {
                t->state = TRK_EMPTY;       /* 候选淘汰 */
            }
        }
    }
}

/* ═══ vst: VST 到达(碰撞||圆内) → ACTIVE 降级 INACTIVE ═══ */
void tracker_vst(f32 vx, f32 vy) {
    for (u8 i = 0; i < TRK_MAX; i++) {
        trk_t *t = &s_tgt[i];
        if (t->state != TRK_ACTIVE) continue;   /* 只处理活跃 */
        bool hit;
        if (s_prev_valid) {
            /* 线段碰撞 + 圆内 (防高速穿过漏检) */
            hit = check_hit_substep(s_prev_vx, s_prev_vy, vx, vy, t->x, t->y, TRK_ARRIVE);
        } else {
            f32 dx = vx - t->x, dy = vy - t->y;
            hit = (dx * dx + dy * dy < TRK_ARRIVE * TRK_ARRIVE);
        }
        if (hit && !t->matched) {                /* 不再匹配保护 + VST 经过 */
            t->state = TRK_INACTIVE;
        }
    }
    s_prev_vx = vx; s_prev_vy = vy;
    s_prev_valid = true;
}

/* ═══ fill (20Hz 主循环): 惰性重建 (dirty) + 锚点 TSP + 滑窗越过 ═══
 *   关中断保护: 200Hz ISR 的 observe/vst 与本主循环 fill 并发访问 s_tgt/s_dirty */
u8 tracker_fill(waypoint_t *g, f32 vst_x, f32 vst_y) {
    __asm volatile ("cpsid i");   /* 关中断, 读出一致快照 */
    /* 惰性重建: 目标集合变化(新 active / 失活→激活)时重排 TSP */
    if (s_dirty) {
        waypoint_t pts[TRK_MAX];
        u8 n = 0;
        for (u8 i = 0; i < TRK_MAX; i++) {
            trk_t *t = &s_tgt[i];
            if (t->state != TRK_ACTIVE) continue;
            pts[n].slot_id = t->id;
            pts[n].x = t->x; pts[n].y = t->y;
            n++;
        }

        /* 锚点置首: 上一帧队首的目标保持在前, 防跳变 */
        u8 anchor_idx = 0xFF;
        for (u8 i = 0; i < n; i++) {
            if (pts[i].slot_id == s_anchor_id) { anchor_idx = i; break; }
        }
        if (anchor_idx != 0xFF && anchor_idx != 0) {
            waypoint_t tmp = pts[0]; pts[0] = pts[anchor_idx]; pts[anchor_idx] = tmp;
        }

        /* TSP 排序 */
        if (anchor_idx != 0xFF) {
            if (n > 2)
                tsp_solve(&pts[1], (const waypoint_t *)&pts[1], n - 1, pts[0].x, pts[0].y);
        } else {
            if (n > 1)
                tsp_solve(pts, (const waypoint_t *)pts, n, vst_x, vst_y);
        }

        /* 缓存有序 id 队列 */
        s_queue_len = n;
        for (u8 i = 0; i < n; i++) s_queue_ids[i] = pts[i].slot_id;
        s_dirty = false;
    }

    /* 用缓存队列填 g (跳过已降级 INACTIVE, 最多 FILL_MAX 个) */
    u8 n = 0;
    for (u8 i = 0; i < s_queue_len && n < FILL_MAX; i++) {
        for (u8 j = 0; j < TRK_MAX; j++) {
            if (s_tgt[j].state == TRK_ACTIVE && s_tgt[j].id == s_queue_ids[i]) {
                g[n].slot_id = s_tgt[j].id;
                g[n].x = s_tgt[j].x; g[n].y = s_tgt[j].y;
                n++;
                break;
            }
        }
    }
    if (n > 0) s_anchor_id = g[0].slot_id;   /* 更新锚点 */
    __asm volatile ("cpsie i");   /* 开中断 */
    return n;
}
