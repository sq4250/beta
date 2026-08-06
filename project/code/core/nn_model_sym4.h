/* nn_model_sym4.h — Symmetric friction circle (a_accel=4.0 a_brake=4.0 a_lat=4.0)
   Student MicroNet 8->80->48->2 ReLU, 4706 params */
#ifndef NN_MODEL_SYM4_H
#define NN_MODEL_SYM4_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H1_DIM    80
#define NN_H2_DIM    48
#define NN_OUT_DIM   2

#define NN_A_MAX        4.0f
#define NN_A_BRAKE_MAX  4.0f
#define NN_O_MAX        14.0f
#define V_MAX            3.0f

#define A_LONG_MAX      NN_A_MAX
#define A_BRAKE_MAX     NN_A_BRAKE_MAX

void nn_forward(f32 out[2], const f32 raw_8d[8]);

#endif
