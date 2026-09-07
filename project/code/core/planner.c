/**
 * planner.c — 规划层外壳 (20Hz)
 *
 * 补录分支: 单一模型 (无混合/探索切换)
 */
#include "planner.h"
#include "model_factory.h"

/* ── 活跃模型指针 (kinematics 读取约束) ── */
#if   NN_MODEL_VERSION == NN_VER_A5B3L4
const model_desc_t *g_model_active = &g_model_kamm;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3
const model_desc_t *g_model_active = &g_model_sym3;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5
const model_desc_t *g_model_active = &g_model_d5;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5V15
const model_desc_t *g_model_active = &g_model_d5v15;
#else
#error "NN_MODEL_VERSION 未匹配 — 需更新 planner.c 的 g_model_active 初始化"
#endif

void planner_forward(planner_action_t *act,
                     const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3) {
    f32 raw[2];
    g_model_active->forward(raw, vst, g1, g2, g3, v2, v3);
    act->a     = raw[0];
    act->omega = raw[1];
}
