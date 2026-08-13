/**
 * target_buf.h — 纯视觉多目标观测缓冲
 *
 * 状态机: CAND(备选) / ACTIVE(活跃=WP) / INACTIVE(非活跃=探索)
 *   CAND  → ACTIVE   score 到 +TRK_PROMOTE (命中+1/未命中-1)
 *   CAND  → EMPTY    score 到 -TRK_DROP (候选淘汰)
 *   ACTIVE→ INACTIVE 不再匹配 + VST 经过
 *   INACTIVE→ACTIVE  被测量更新 (灯再亮, 无门槛)
 *
 * 匹配: 全局贪心 GNN, 新息=欧氏, 门限=距离仿射(1+0.4·dist), 目标可吸收多条
 * 序号: 正式点递增分配(s_id_cnt++), 永久不复用
 *
 * 分层 (观测器 200Hz ISR / 决策 20Hz 主循环):
 *   predict (200Hz): 仅 P += Q 死推
 *   observe (有新鲜测量): GNN + KF + 帧末 score 判候选生死
 *   vst     (200Hz): ACTIVE 到达降级
 *   fill    (20Hz): 活跃点锚点 TSP → 网络输入 (关中断保护, 防与 ISR 竞争)
 */
#ifndef TARGET_BUF_H
#define TARGET_BUF_H
#include "common.h"

void tracker_init(void);
void tracker_predict(void);                                            /* 200Hz: 仅 P+=Q 死推 */
void tracker_observe(const f32 *zx, const f32 *zy, u8 n,
                     f32 vst_x, f32 vst_y);                            /* 新鲜测量: 记账+GNN+KF */
void tracker_vst(f32 vx, f32 vy);                                      /* 200Hz: VST 到达降级 */
u8   tracker_fill(waypoint_t *g, f32 vst_x, f32 vst_y);                /* 20Hz: 填网络输入(关中断) */

#endif
