/**
 * estimator.c — 状态估计: 测量 + 控制量 → x̂ (纯计算)
 */
#include "estimator.h"
#include "config.h"
#include "utils.h"
#include <math.h>

static f32 g_v_filt     = 0.0f;  // EMA 滤波速度
static f32 g_yaw_offset  = 0.0f;  // 初始航向偏置 (上电时刻归零)

void car_estimate_set_yaw_offset(f32 offset) {
    g_yaw_offset = wrap_pi(offset);
}

void car_estimate_update(car_state_t *car, const imu_data_t *imu,
                         const encoder_t *enc, const actuator_cmd_t *cmd
    ) {
    // ── 偏航融合 ──
    if (imu->has_quat) {
        const f32 *q = imu->quat;
        f32 yq = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]),
                        1.0f - 2.0f*(q[0]*q[0] + q[2]*q[2]));
        car->theta = wrap_pi(M_PI_F - yq);  /* 180°偏移 + 符号翻转, 纯四元数 */
    } else {
        car->theta += imu->gyro[2] * ISR_DT;
    }

    car->theta = wrap_pi(car->theta - g_yaw_offset);

    // ── 速度: EMA 低通滤波 ──
    f32 vl = enc->left  * (f32)TRACKER_FREQ;
    f32 vr = enc->right * (f32)TRACKER_FREQ;
    f32 v_raw = (vl + vr) * 0.5f;
    g_v_filt += SPEED_FLT_ALPHA * (v_raw - g_v_filt);
    car->v  = g_v_filt;

    // ── 里程计 (使用滤波速度积分) ──
    car->x += car->v * cosf(car->theta) * ISR_DT;
    car->y += car->v * sinf(car->theta) * ISR_DT;

    // ── δ = 上周期舵机指令 (跟踪良好假设) ──
    car->delta = cmd->servo_delta;
}
