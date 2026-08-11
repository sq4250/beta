/**
 * nn_model_sym3.h — GP-Small 3.7K Sym3: polar_8d + GatedConcat
 *
 * Architecture:
 *   state_enc(2→8) + target_enc(2→8)×3 → concat(32D) → 48→32→16→2 ReLU
 */
#ifndef NN_MODEL_SYM3_H
#define NN_MODEL_SYM3_H
#include "common.h"
#include "car_state.h"

#define NN_IN_DIM    8
#define NN_H_STATE   8
#define NN_H_TGT     8
#define NN_H1        48
#define NN_H2        32
#define NN_H3        16
#define NN_OUT_DIM   2

/* Output clamp */
#define NN_A_MAX        3.0f
#define NN_A_BRAKE_MAX  3.0f
#define NN_O_MAX        14.0f

/* Input normalization */
#define NN_V_SCALE      5.0f
#define NN_D_SCALE      5.0f
#define NN_A_SCALE      3.14159265f

/* Physical constraints (Symmetric circle 3/3/3) */
#define A_LONG_MAX      3.0f
#define A_BRAKE_MAX     3.0f
#define A_LAT_MAX       3.0f
#define V_MAX           5.0f
#define DELTA_MAX       0.46364761f
#define OMEGA_DELTA_MAX 14.0f

void nn_planner_step(f32 out[2], const CarState *vst,
                     const Waypoint *g1, const Waypoint *g2, const Waypoint *g3,
                     f32 v2, f32 v3);

#endif
