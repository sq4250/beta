/**
 * imu_660rb.c — IMU660RB 驱动包装 (原始传感器模式, 同步 SPI)
 */
#include "imu_660rb.h"
#include "zf_common_headfile.h"

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (imu660rb_init()) return false;
    *acc_scale  = 4098.0f;  // LSB/g  (默认 ±8g, IMU660RB_ACC_SAMPLE=0x3C)
    *gyro_scale = 14.3f;    // LSB/(deg/s) (默认 ±2000dps, IMU660RB_GYR_SAMPLE=0x5C)
    return true;
}

static void read_raw(i16 g[3], i16 a[3]) {
    imu660rb_get_gyro(); imu660rb_get_acc();
    g[0]=imu660rb_gyro_x; g[1]=imu660rb_gyro_y; g[2]=imu660rb_gyro_z;
    a[0]=imu660rb_acc_x;  a[1]=imu660rb_acc_y;  a[2]=imu660rb_acc_z;
}

const imu_driver_t imu_660rb_driver = { init, read_raw, NULL };
