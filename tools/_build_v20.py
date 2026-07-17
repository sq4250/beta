import re

BASE = r"C:\Users\31903\Desktop\FLY\refactor-pilot\devil-may-drive\project\code\core"

# Read old nn_model.c to extract ver-2.0 weights
with open(r"C:\Users\31903\Desktop\FLY\refactor-pilot\devil-may-drive\project\code\core\nn_model.c", "r", encoding="utf-8") as f:
    old = f.read()

arrays = {}
for name in ["fc1_weight", "fc1_bias", "fc2_weight", "fc2_bias", "fc3_weight", "fc3_bias"]:
    m = re.search(rf"const f32 {name}\[\]\s*=\s*\{{(.*?)\}};", old, re.DOTALL)
    if m:
        arrays[name] = m.group(1).strip()

src = """/* nn_model_v20.c — 低速版  a_max=2.0  v_max=3.5 */

#include "nn_model_v20.h"
#include <string.h>

#if NN_MODEL_VERSION == NN_VER_20

static const f32 fc1_weight[] = {
"""
src += arrays["fc1_weight"] + "\n};\n\n"

for name in ["fc1_bias", "fc2_weight", "fc2_bias", "fc3_weight", "fc3_bias"]:
    src += f"static const f32 {name}[] = {{\n{arrays[name]}\n}};\n\n"

src += """static void dense_relu(f32 *out, const f32 *in, const f32 *w, const f32 *b,
                       u32 in_dim, u32 out_dim) {
    for (u32 i = 0; i < out_dim; ++i) {
        f32 s = b[i];
        for (u32 j = 0; j < in_dim; ++j)
            s += w[i * in_dim + j] * in[j];
        out[i] = (s > 0.0f) ? s : 0.0f;
    }
}

static void dense_linear(f32 *out, const f32 *in, const f32 *w, const f32 *b,
                          u32 in_dim, u32 out_dim) {
    for (u32 i = 0; i < out_dim; ++i) {
        f32 s = b[i];
        for (u32 j = 0; j < in_dim; ++j)
            s += w[i * in_dim + j] * in[j];
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

#endif /* NN_MODEL_VERSION == NN_VER_20 */
"""

with open(f"{BASE}/nn_model_v20.c", "w", encoding="utf-8") as f:
    f.write(src)
print("nn_model_v20.c  OK")
