/**
 * nn_inference.c — 8D MicroNet 推理实现 (纯C, 可部署)
 */
#include "nn_inference.h"
#include "nn_weights.h"
#include "config.h"

static void dense_relu(f32 *out, const f32 *in, const f32 *w, const f32 *b,
                       u32 in_dim, u32 out_dim) {
    for (u32 i = 0; i < out_dim; ++i) {
        f32 s = b[i];
        for (u32 j = 0; j < in_dim; ++j) {
            s += w[i * in_dim + j] * in[j];
        }
        out[i] = (s > 0.0f) ? s : 0.0f;
    }
}

static void dense_linear(f32 *out, const f32 *in, const f32 *w, const f32 *b,
                          u32 in_dim, u32 out_dim) {
    for (u32 i = 0; i < out_dim; ++i) {
        f32 s = b[i];
        for (u32 j = 0; j < in_dim; ++j) {
            s += w[i * in_dim + j] * in[j];
        }
        out[i] = s;
    }
}

void nn_forward(f32 out[2], const f32 raw_8d[8]) {
    f32 h1[NN_H1_DIM];
    f32 h2[NN_H2_DIM];

    dense_relu(h1, raw_8d, fc1_weight, fc1_bias, NN_IN_DIM, NN_H1_DIM);
    dense_relu(h2, h1, fc2_weight, fc2_bias, NN_H1_DIM, NN_H2_DIM);
    dense_linear(out, h2, fc3_weight, fc3_bias, NN_H2_DIM, NN_OUT_DIM);

    if (out[0] >  NN_A_MAX) out[0] =  NN_A_MAX;
    if (out[0] < -NN_A_MAX) out[0] = -NN_A_MAX;
    if (out[1] >  NN_O_MAX) out[1] =  NN_O_MAX;
    if (out[1] < -NN_O_MAX) out[1] = -NN_O_MAX;
}
