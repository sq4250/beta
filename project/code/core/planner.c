/**
 * planner.c — 规划层外壳 (20Hz)
 */
#include "planner.h"

/* ── 非混合模式: 单一模型 ── */
#if NN_MODEL_VERSION != NN_VER_HYBRID

void planner_forward(planner_action_t *act,
                     const car_state_t *vst,
                     const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                     f32 v2, f32 v3) {
    f32 raw[2];
#if   NN_MODEL_VERSION == NN_VER_A5B3L4
    nn_planner_step_kamm(raw, vst, g1, g2, g3, v2, v3);
#elif NN_MODEL_VERSION == NN_VER_A3B3L3
    nn_planner_step(raw, vst, g1, g2, g3, v2, v3);
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5
    nn_planner_step_d5(raw, vst, g1, g2, g3, v2, v3);
#elif NN_MODEL_VERSION == NN_VER_A3B3L3_D5V15
    nn_planner_step_d5v15(raw, vst, g1, g2, g3, v2, v3);
#endif
    act->a     = raw[0];
    act->omega = raw[1];
}

/* ── 混合模式: D5(WP) + Kamm(探索) ── */
#else

f32 g_a_long_max  = D5V15_A_LONG_MAX;
f32 g_a_brake_max = D5V15_A_BRAKE_MAX;
f32 g_a_lat_max   = D5V15_A_LAT_MAX;
f32 g_v_max       = D5V15_V_MAX;

void planner_forward_hybrid(planner_action_t *act,
                            const car_state_t *vst,
                            const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                            f32 v2, f32 v3, bool is_wp) {
    f32 raw[2];
    if (is_wp) {
        g_a_long_max  = D5V15_A_LONG_MAX;
        g_a_brake_max = D5V15_A_BRAKE_MAX;
        g_a_lat_max   = D5V15_A_LAT_MAX;
        g_v_max       = D5V15_V_MAX;
        nn_planner_step_d5v15(raw, vst, g1, g2, g3, v2, v3);
    } else {
        g_a_long_max  = KAMM_A_LONG_MAX;
        g_a_brake_max = KAMM_A_BRAKE_MAX;
        g_a_lat_max   = KAMM_A_LAT_MAX;
        g_v_max       = KAMM_V_MAX;
        nn_planner_step_kamm(raw, vst, g1, g2, g3, v2, v3);
    }
    act->a     = raw[0];
    act->omega = raw[1];
}

#endif