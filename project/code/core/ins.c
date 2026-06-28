/**
 * ins.c — 惯性导航: yaw 融合 + 里程计 (纯计算, 不访问硬件)
 */
#include "ins.h"
#include "config.h"
#include <math.h>

static f32 g_yaw = 0.0f;
static f32 g_yaw_rate = 0.0f;

void ins_init(void) { g_yaw = 0.0f; g_yaw_rate = 0.0f; }

void ins_update_yaw(f32 gyro_z, const f32 quat[4], bool has_quat) {
    g_yaw_rate = gyro_z;
    if (has_quat) {
        f32 yq = atan2f(2.0f*(quat[0]*quat[3]+quat[1]*quat[2]),
                        1.0f-2.0f*(quat[2]*quat[2]+quat[3]*quat[3]));
        g_yaw = IMU_YAW_ALPHA * (g_yaw + g_yaw_rate * ISR_DT) + IMU_QUAT_ALPHA * yq;
    } else {
        g_yaw += g_yaw_rate * ISR_DT;
    }
}

f32 ins_yaw(void) { return g_yaw; }

void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt) {
    car->v = (vl + vr) * 0.5f;
    car->theta = ins_yaw();
    car->x += car->v * cosf(car->theta) * dt;
    car->y += car->v * sinf(car->theta) * dt;
}
