/**
 * icm20602.c — ICM20602 驱动包装 (原始传感器模式, 同步 SPI)
 */
#include "icm20602.h"
#include "zf_common_headfile.h"

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (icm20602_init()) return false;
    *acc_scale  = icm20602_transition_factor[0];  // LSB/g
    *gyro_scale = icm20602_transition_factor[1];  // LSB/(deg/s)
    return true;
}

static void read_raw(i16 g[3], i16 a[3]) {
    icm20602_get_gyro(); icm20602_get_acc();
    g[0]=icm20602_gyro_x; g[1]=icm20602_gyro_y; g[2]=icm20602_gyro_z;
    a[0]=icm20602_acc_x;  a[1]=icm20602_acc_y;  a[2]=icm20602_acc_z;
}

const imu_driver_t icm20602_driver = { init, read_raw, NULL };
