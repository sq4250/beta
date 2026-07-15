/**
 * hal_imu.c — IMU 工厂 + 标定
 *
 * 标定: 前N样本递推平均, 后续 EMA
 *       运动中加速度超限/陀螺超阈值 → 跳过该样本
 */
#include "hal_imu.h"
#include <math.h>
#include "system_cyt2bl.h"
#include "zf_driver_delay.h"
#include "config.h"

struct ImuHandle_ {
    const ImuDriver *drv;
    f32 acc_factor, gyro_factor;
    f32 bias_gz;
};

static struct ImuHandle_ g_imu_inst;

ImuHandle hal_imu_create(const ImuDriver *drv) {
    struct ImuHandle_ *h = &g_imu_inst;
    h->drv = drv;

    f32 acc_scale, gyro_scale;
    if (!drv->init(&acc_scale, &gyro_scale)) return NULL;
    h->acc_factor  = 1.0f / acc_scale;
    h->gyro_factor = 1.0f / gyro_scale;

    f32 bias = 0.0f;
    u32 step = 0;
    while (step < (u32)IMU_BIAS_TOTAL) {
        system_delay_ms(1);
        i16 g_raw[3], a_raw[3]; drv->read_raw(g_raw, a_raw);
        f32 ax = (f32)a_raw[0]*h->acc_factor, ay = (f32)a_raw[1]*h->acc_factor, az = (f32)a_raw[2]*h->acc_factor;
        f32 acc_norm = sqrtf(ax*ax + ay*ay + az*az);
        if (acc_norm > IMU_ACC_NORM_MAX || acc_norm < IMU_ACC_NORM_MIN) continue;
        if (abs(g_raw[0]) > IMU_MOTION_THR || abs(g_raw[1]) > IMU_MOTION_THR || abs(g_raw[2]) > IMU_MOTION_THR) continue;
        ++step;
        f32 gz = (f32)g_raw[2] * h->gyro_factor;
        if (step <= IMU_BIAS_FAST) bias += (gz - bias) / (f32)step;
        else                       bias += IMU_BIAS_ALPHA * (gz - bias);
    }
    h->bias_gz = bias;
    return h;
}

void hal_imu_destroy(ImuHandle h) { (void)h; }

void hal_imu_read_all(f32 g[3], f32 a[3], const ImuHandle h) {
    i16 raw_g[3], raw_a[3]; h->drv->read_raw(raw_g, raw_a);
    g[0] = (f32)raw_g[0] * h->gyro_factor * DEG2RAD;
    g[1] = (f32)raw_g[1] * h->gyro_factor * DEG2RAD;
    g[2] = ((f32)raw_g[2] * h->gyro_factor - h->bias_gz) * DEG2RAD;
    a[0] = (f32)raw_a[0] * h->acc_factor;
    a[1] = (f32)raw_a[1] * h->acc_factor;
    a[2] = (f32)raw_a[2] * h->acc_factor;
}
void hal_imu_read_gyro(f32 g[3], const ImuHandle h) {
    f32 a[3]; hal_imu_read_all(g, a, h);
}
void hal_imu_read_accel(f32 a[3], const ImuHandle h) {
    f32 g[3]; hal_imu_read_all(g, a, h);
}
bool hal_imu_has_quat(const ImuHandle h)         { return h->drv->read_quat != NULL; }
void hal_imu_read_quat(f32 q[4], const ImuHandle h) { h->drv->read_quat(q); }
f32  hal_imu_gyro_bias_z(const ImuHandle h)         { return h->bias_gz; }
