/**
 * nn_model_a5b3l4.h — GP-Small 3.7K A5B3L4 Kamm: polar_8d + GatedConcat
 *
 * Architecture:
 *   state_enc(2→8) + target_enc(2→8)×3 → concat(32D) → 48→32→16→2 ReLU
 *   No input normalization — polar encoding IS the normalization
 */
#ifndef NN_MODEL_GP_SMALL_H
#define NN_MODEL_GP_SMALL_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H_STATE   8
#define NN_H_TGT     8
#define NN_H1        48
#define NN_H2        32
#define NN_H3        16
#define NN_OUT_DIM   2

/* Output clamp */
#define NN_A_MAX        5.0f
#define NN_A_BRAKE_MAX  3.0f
#define NN_O_MAX        14.0f

/* Input normalization constants (must match training) */
#define NN_V_SCALE      5.0f       // v / V_SCALE  ∈ [0, 1]
#define NN_D_SCALE      5.0f       // d / D_SCALE  ∈ [0, ~1.2]
#define NN_A_SCALE      3.14159265f // θ / π       ∈ [-1, 1]

/* Physical constraints (Kamm asymmetric friction ellipse) */
#define A_LONG_MAX      5.0f       // 最大纵向加速度 [m/s²]
#define A_BRAKE_MAX     3.0f       // 最大制动减速度 [m/s²]
#define A_LAT_MAX       4.0f       // 最大横向加速度 [m/s²]
#define V_MAX           2.5f       // 最大速度 [m/s]
#define DELTA_MAX       0.46364761f // 前轮转角上限 [rad] (≈26.5°)
#define OMEGA_DELTA_MAX 14.0f      // 前轮转角角速度上限 [rad/s]

/**
 * 模型入口: vst + 3航点 → [a, omega]
 *
 * 内部完成: 车体→polar编码 → gate检测 → 前向 → clamp
 * planner.c 只需调用此函数, 不关心编码细节.
 */
void nn_planner_step(f32 out[2], const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3);

#endif
