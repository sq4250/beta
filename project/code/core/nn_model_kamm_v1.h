/* nn_model_kamm_v1.h — Kamm ellipse asymmetric (a_accel=8.0 a_brake=4.0 a_lat=5.0)
   Student MicroNet 8->80->48->2 ReLU, 4722 params */
#ifndef NN_MODEL_KAMM_V1_H
#define NN_MODEL_KAMM_V1_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H1_DIM    80
#define NN_H2_DIM    48
#define NN_OUT_DIM   2

#define NN_A_MAX        8.0f
#define NN_A_BRAKE_MAX  4.0f
#define NN_O_MAX        14.0f
#define V_MAX            5.0f

#define A_LONG_MAX      NN_A_MAX
#define A_BRAKE_MAX     NN_A_BRAKE_MAX

void nn_forward(f32 out[2], const f32 raw_8d[8]);

#endif
