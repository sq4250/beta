/**
 * nn_model_a5b3l3.h — GP-Small 3.7K A5B3L3 Kamm: polar_8d + GatedConcat
 *
 * a_long=5 a_brake=3 a_lat=3 — Kamm 发动机 + 低横向
 * 归一化融合进权重, polar_encode 直接输出物理量
 */
#ifndef NN_MODEL_A5B3L3_H
#define NN_MODEL_A5B3L3_H
#include "common.h"

#define NN_IN_DIM    8
#define NN_H_STATE   8
#define NN_H_TGT     8
#define NN_H1        48
#define NN_H2        32
#define NN_H3        16
#define NN_OUT_DIM   2

/* Physical constraints (documentation — 运行时由 model_desc_t 提供) */

void nn_planner_step_a5b3l3(f32 out[2], const car_state_t *vst,
                            const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                            f32 v2, f32 v3);

#endif
