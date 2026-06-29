/**
 * hal_imu.h — IMU 硬件抽象接口
 *
 * 驱动层只做 init + read_raw, hal_imu 只做工厂 + 标定
 * yaw 计算在 core/ins 状态估计层
 */
#ifndef HAL_IMU_H
#define HAL_IMU_H
#include "common.h"

typedef struct {
    bool (*init)(f32 *acc_scale, f32 *gyro_scale);
    void (*read_raw)(i16 g[3], i16 a[3]);
    bool (*has_quat)(void);
    void (*read_quat)(f32 q[4]);
} ImuDriver;

typedef struct ImuHandle_ *ImuHandle;

ImuHandle hal_imu_create(const ImuDriver *drv);
void      hal_imu_destroy(ImuHandle h);
void      hal_imu_read_gyro(f32 g[3], const ImuHandle h);
void      hal_imu_read_accel(f32 a[3], const ImuHandle h);
bool      hal_imu_has_quat(const ImuHandle h);
void      hal_imu_read_quat(f32 q[4], const ImuHandle h);
f32       hal_imu_gyro_bias_z(const ImuHandle h);

#endif
