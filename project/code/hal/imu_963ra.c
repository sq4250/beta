/**
 * imu_963ra.c — IMU963RA 驱动包装 (原始传感器模式, 同步 SPI)
 *
 * 963RA 是 9 轴 IMU (陀螺+加速度+磁力计), ImuDriver 接口只暴露 gyro/accel,
 * 磁力计数据可通过 zf 库全局变量 imu963ra_mag_x/y/z 直接访问
 */
#include "imu_963ra.h"
#include "zf_common_headfile.h"

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (imu963ra_init()) return false;
    *acc_scale  = imu963ra_transition_factor[0];  // LSB/g
    *gyro_scale = imu963ra_transition_factor[1];  // LSB/(deg/s)
    return true;
}

static void read_raw(i16 g[3], i16 a[3]) {
    imu963ra_get_gyro(); imu963ra_get_acc();
    g[0]=imu963ra_gyro_x; g[1]=imu963ra_gyro_y; g[2]=imu963ra_gyro_z;
    a[0]=imu963ra_acc_x;  a[1]=imu963ra_acc_y;  a[2]=imu963ra_acc_z;
}

const ImuDriver imu_963ra_driver = { init, read_raw, NULL };
