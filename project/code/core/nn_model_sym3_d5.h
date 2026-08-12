/**
 * nn_model_sym3_d5.h — GP-Small 3.7K Sym3 D5: polar_8d + |δ|<5° arrival
 */
#ifndef NN_MODEL_SYM3_D5_H
#define NN_MODEL_SYM3_D5_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H_STATE   8
#define NN_H_TGT     8
#define NN_H1        48
#define NN_H2        32
#define NN_H3        16
#define NN_OUT_DIM   2



/* Physical constraints — 运行时由 model_desc_t 提供 */

void nn_planner_step_d5(f32 out[2], const car_state_t *vst,
                        const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                        f32 v2, f32 v3);

#endif
