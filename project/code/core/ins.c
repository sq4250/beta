/**
 * ins.c — 航向估计 + 里程计 (纯计算, 不访问硬件)
 */
#include "ins.h"
#include "config.h"
#include <math.h>

static f32 g_theta_rate = 0.0f;  // 陀螺 z 轴 [rad/s] (最新测量值)

void ins_init(void) { g_theta_rate = 0.0f; }

f32 ins_theta_rate(void) { return g_theta_rate; }

void ins_fuse_theta(CarState *car, f32 gyro_z, const f32 quat[4], bool has_quat) {
    g_theta_rate = gyro_z;
    if (has_quat) {
        f32 yq = atan2f(2.0f*(quat[0]*quat[3]+quat[1]*quat[2]),
                        1.0f-2.0f*(quat[2]*quat[2]+quat[3]*quat[3]));
        car->theta = IMU_YAW_ALPHA * (car->theta + g_theta_rate * ISR_DT)
                   + IMU_QUAT_ALPHA * yq;
    } else {
        car->theta += g_theta_rate * ISR_DT;
    }
}

void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt) {
    car->v  = (vl + vr) * 0.5f;
    car->x += car->v * cosf(car->theta) * dt;
    car->y += car->v * sinf(car->theta) * dt;
}
