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

/* Output clamp — 首次定义优先 */
#ifndef NN_A_MAX
#define NN_A_MAX        5.0f
#define NN_A_BRAKE_MAX  3.0f
#define NN_O_MAX        14.0f
#endif

/* Input normalization */
#ifndef NN_V_SCALE
#define NN_V_SCALE      5.0f
#define NN_D_SCALE      5.0f
#define NN_A_SCALE      3.14159265f
#endif

/* Physical constraints (documentation — 运行时由 model_desc_t 提供) */
#define DELTA_MAX       0.46364761f
#define OMEGA_DELTA_MAX 14.0f
#endif

void nn_planner_step_kamm(f32 out[2], const car_state_t *vst,
                          const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                          f32 v2, f32 v3);

#endif
