/**
 * ins.c — 惯性导航: yaw 融合 + 里程计
 */
#include "ins.h"
#include "config.h"
#include <math.h>

static f32 g_yaw = 0.0f;
static f32 g_yaw_rate = 0.0f;

void ins_init(void) { g_yaw = 0.0f; g_yaw_rate = 0.0f; }

void ins_update_yaw(ImuHandle imu) {
    f32 g[3]; hal_imu_read_gyro(g, imu);
    g_yaw_rate = g[2];  // z 轴已去 bias, rad/s

    if (hal_imu_has_quat(imu)) {
        f32 q[4]; hal_imu_read_quat(q, imu);
        f32 yq = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));
        g_yaw = IMU_YAW_ALPHA * (g_yaw + g_yaw_rate * SENSE_DT) + IMU_QUAT_ALPHA * yq;
    } else {
        g_yaw += g_yaw_rate * SENSE_DT;
    }
}

f32 ins_yaw(void) { return g_yaw; }

void ins_update_odom(CarState *car, f32 vl, f32 vr, f32 dt) {
    car->v = (vl + vr) * 0.5f;
    car->theta = ins_yaw();  // 跟随融合偏航 (100Hz 陀螺+四元数), 唯一真值
    car->x += car->v * cosf(car->theta) * dt;
    car->y += car->v * sinf(car->theta) * dt;
}
