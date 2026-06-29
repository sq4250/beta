/**
 * ins.c — 状态估计: IMU 偏航融合 + 编码器里程计 (纯计算)
 */
#include "ins.h"
#include "config.h"
#include <math.h>

static f32 g_theta_rate = 0.0f;

void ins_init(void) { g_theta_rate = 0.0f; }

f32 ins_theta_rate(void) { return g_theta_rate; }

void car_estimate_update(CarState *car, f32 gyro_z, const f32 quat[4], bool has_quat,
                         f32 enc_l, f32 enc_r) {
    // ── 偏航融合 (1kHz, ISR_DT) ──
    g_theta_rate = gyro_z;
    if (has_quat) {
        f32 yq = atan2f(2.0f*(quat[0]*quat[3]+quat[1]*quat[2]),
                        1.0f-2.0f*(quat[2]*quat[2]+quat[3]*quat[3]));
        car->theta = IMU_YAW_ALPHA * (car->theta + g_theta_rate * ISR_DT)
                   + IMU_QUAT_ALPHA * yq;
    } else {
        car->theta += g_theta_rate * ISR_DT;
    }

    // ── 里程计 (编码器 ZOH, 200Hz 读一次, 1kHz 保持) ──
    f32 vl = enc_l * (f32)TRACKER_FREQ;   // 5ms 平均速度
    f32 vr = enc_r * (f32)TRACKER_FREQ;
    car->v  = (vl + vr) * 0.5f;
    car->x += car->v * cosf(car->theta) * ISR_DT;  // 1kHz 积分
    car->y += car->v * sinf(car->theta) * ISR_DT;
}
