/**
 * planner.c — 规划层外壳 (20Hz)
 */
#include "planner.h"
#include "model_factory.h"

/* ── 活跃模型指针 (kinematics 读取约束) ── */
const model_desc_t *g_model_active = &g_model_d5v15;

/* ── 非混合模式: 单一模型 ── */
#if NN_MODEL_VERSION != NN_VER_HYBRID

void planner_forward(planner_action_t *act,
                     const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3) {
    f32 raw[2];
#if   NN_MODEL_VERSION == NN_VER_A5B3L4
    g_model_active = &g_model_kamm;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3
    g_model_active = &g_model_sym3;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5
    g_model_active = &g_model_d5;
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5V15
    g_model_active = &g_model_d5v15;
#endif
    g_model_active->forward(raw, vst, g1, g2, g3, v2, v3);
    act->a     = raw[0];
    act->omega = raw[1];
}

/* ── 混合模式: D5V15(WP) + Kamm(探索) ── */
#else

void planner_forward_hybrid(planner_action_t *act,
                            const car_state_t *vst,
                            const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                            f32 v2, f32 v3, bool is_wp) {
    f32 raw[2];
    g_model_active = is_wp ? &g_model_d5v15 : &g_model_kamm;
    g_model_active->forward(raw, vst, g1, g2, g3, v2, v3);
    act->a     = raw[0];
    act->omega = raw[1];
}

#endif