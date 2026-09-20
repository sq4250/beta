/**
 * nn_model_gpmed_v4.h — GP-Medium 9K Kamm v4: polar_8d + GatedConcat
 *
 * Architecture:
 *   state_enc(2→10) + target_enc(2→10)×3 → concat(40D) → 64→64→32→2 ReLU
 *   No input normalization — polar encoding IS the normalization
 *   (scale 已折叠进第一层权重)
 * Params: 8990  |  a_lat=3.0  |  from runs/gpmed_v4.pt
 */
#ifndef NN_MODEL_GPMED_V4_H
#define NN_MODEL_GPMED_V4_H
#include "common.h"

#define NN_IN_DIM    40
#define NN_H_STATE   10
#define NN_H_TGT     10
#define NN_H1        64
#define NN_H2        64
#define NN_H3        32
#define NN_CAT       40
#define NN_OUT_DIM   2

void nn_planner_step_gpmed_v4(f32 out[2], const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3);

#endif
