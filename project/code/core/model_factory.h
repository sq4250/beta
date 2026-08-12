/**
 * model_factory.h — 模型工厂: 统一接口 + 自描述约束
 *
 * 每个模型 .c 提供 const model_desc_t 实例, 包含:
 *   forward          — 模型入口函数指针
 *   a_long/brake/lat/v     — 物理约束 (kinematics 用)
 *   nn_a_max/nn_a_brake/nn_o — NN 输出钳位 (模型内部用)
 */
#ifndef MODEL_FACTORY_H
#define MODEL_FACTORY_H
#include "common.h"

typedef struct {
    void (*forward)(f32 out[2], const car_state_t *vst,
                    const waypoint_t *g1, const waypoint_t *g2, const waypoint_t *g3,
                    f32 v2, f32 v3);
    f32 a_long_max;
    f32 a_brake_max;
    f32 a_lat_max;
    f32 v_max;
    f32 nn_a_max;
    f32 nn_a_brake_max;
    f32 nn_o_max;
} model_desc_t;

/* ── 当前活跃模型 (planner 设置, kinematics/NN 读取) ── */
extern const model_desc_t *g_model_active;

/* ── 模型实例 (各 .c 定义, 按编译条件可见) ── */
extern const model_desc_t g_model_kamm;
extern const model_desc_t g_model_sym3;
extern const model_desc_t g_model_d5;
extern const model_desc_t g_model_d5v15;
extern const model_desc_t g_model_a5b3l3;

#endif