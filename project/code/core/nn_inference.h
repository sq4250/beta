/**
 * nn_inference.h — 8D MicroNet 推理
 */

#ifndef NN_INFERENCE_H
#define NN_INFERENCE_H

#include "common.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     NN 前向推理 (8D → 2D, 归一化已融合)
// 参数说明     out         输出 [a_long, omega] (m/s², rad/s)
// 参数说明     raw_8d      输入 [v,δ,dx1,dy1,dx2,dy2,dx3,dy3] (SI单位)
// 返回参数     void
// 使用示例     f32 out[2]; nn_forward(out, body_input);
// 备注信息     架构 8→80→48→2 ReLU, 4706参数, ~4576 MACs
//-------------------------------------------------------------------------------------------------------------------
void nn_forward(f32 out[2], const f32 raw_8d[8]);

#endif
