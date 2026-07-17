/**
 * nn_model.h — 网络模型 (换网络只需替换 nn_model.h + nn_model.c)
 *
 * 架构: 8 → 80 → 48 → 2 ReLU, 4706 参数, ~4576 MACs
 * 输入: [v, δ, dx1, dy1, dx2, dy2, dx3, dy3]  (body 系, SI)
 * 输出: a_long [m/s²], omega [rad/s]
 */

#ifndef NN_MODEL_H
#define NN_MODEL_H
#include "common.h"

/* ════════════════════════════════════════════
 *  架构
 * ════════════════════════════════════════════ */
#define NN_IN_DIM    8
#define NN_H1_DIM    80
#define NN_H2_DIM    48
#define NN_OUT_DIM   2

/* ════════════════════════════════════════════
 *  物理限制 (训练目标范围, 即加速度/角速度限幅)
 * ════════════════════════════════════════════ */
#define NN_A_MAX        2.0f    /* 最大加速度 [m/s²] (低速对称) */
#define NN_A_BRAKE_MAX  2.0f    /* 最大制动减速度 [m/s²] */
#define NN_O_MAX        14.0f   /* 最大转向角速度 [rad/s] */

/* ── 别名 ── */
#define A_LONG_MAX      NN_A_MAX
#define A_BRAKE_MAX     NN_A_BRAKE_MAX

/* ════════════════════════════════════════════
 *  权重 (extern, 定义在 nn_model.c)
 * ════════════════════════════════════════════ */
extern const f32 fc1_weight[];
extern const f32 fc1_bias[];
extern const f32 fc2_weight[];
extern const f32 fc2_bias[];
extern const f32 fc3_weight[];
extern const f32 fc3_bias[];

/* ════════════════════════════════════════════
 *  推理
 * ════════════════════════════════════════════ */
void nn_forward(f32 out[2], const f32 raw_8d[8]);

#endif
