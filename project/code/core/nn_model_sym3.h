/**
 * nn_model_sym3.h — GP-Small 3.7K Sym3: polar_8d + GatedConcat
 *
 * Architecture:
 *   state_enc(2→8) + target_enc(2→8)×3 → concat(32D) → 48→32→16→2 ReLU
 */
#ifndef NN_MODEL_SYM3_H
#define NN_MODEL_SYM3_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H_STATE   8
#define NN_H_TGT     8
#define NN_H1        48
#define NN_H2        32
#define NN_H3        16
#define NN_OUT_DIM   2

/* Output clamp — 首次定义优先 */

/* Input normalization */

/* Physical constraints — 运行时由 model_desc_t 提供 */

void nn_planner_step(f32 out[2], const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3);

#endif
