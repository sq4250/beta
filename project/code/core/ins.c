/**
 * ins.c — 状态估计实现 (纯计算, 不访问硬件)
 */
#include "ins.h"
#include "config.h"
#include <math.h>

static f32 g_v_filt = 0.0f;  // EMA 滤波速度

void car_estimate_update(CarState *car, const ImuData *imu,
                         const Encoder *enc, const ActuatorCmd *cmd) {
    // ── 偏航融合 ──
    if (imu->has_quat) {
        f32 yq = atan2f(2.0f*(imu->quat[0]*imu->quat[3]+imu->quat[1]*imu->quat[2]),
                        1.0f-2.0f*(imu->quat[2]*imu->quat[2]+imu->quat[3]*imu->quat[3]));
        car->theta = IMU_YAW_ALPHA * (car->theta + imu->gyro[2] * ISR_DT)
                   + IMU_QUAT_ALPHA * yq;
    } else {
        car->theta += imu->gyro[2] * ISR_DT;
    }

    // ── 速度: EMA 低通滤波 ──
    f32 vl = enc->left  * (f32)TRACKER_FREQ;
    f32 vr = enc->right * (f32)TRACKER_FREQ;
    f32 v_raw = (vl + vr) * 0.5f;
    g_v_filt += SPEED_FILT_ALPHA * (v_raw - g_v_filt);
    car->v  = g_v_filt;

    // ── 里程计 (使用滤波速度积分) ──
    car->x += car->v * cosf(car->theta) * ISR_DT;
    car->y += car->v * sinf(car->theta) * ISR_DT;

    // ── δ = 上周期舵机指令 (跟踪良好假设) ──
    car->delta = cmd->servo_delta;
}
