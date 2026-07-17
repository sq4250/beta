/**
 * nn_model_v1p5.h — a_max=1.5  v_max=2.5
 * 架构: 8→80→48→2 ReLU, 4706参数
 */
#ifndef NN_MODEL_V1P5_H
#define NN_MODEL_V1P5_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H1_DIM    80
#define NN_H2_DIM    48
#define NN_OUT_DIM   2

#define NN_A_MAX        1.5f
#define NN_A_BRAKE_MAX  1.5f
#define NN_O_MAX        14.0f
#define V_MAX            2.5f

#define A_LONG_MAX      NN_A_MAX
#define A_BRAKE_MAX     NN_A_BRAKE_MAX

void nn_forward(f32 out[2], const f32 raw_8d[8]);

#endif
