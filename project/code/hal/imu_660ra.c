/**
 * imu_660ra.c — 仅 init + read_raw
 */
#include "imu_660ra.h"
#include "zf_common_headfile.h"

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (imu660ra_init()) return false;
    *acc_scale  = imu660ra_transition_factor[0];  // LSB/g  (校准用 g 单位, 匹配 IMU_ACC_NORM)
    *gyro_scale = imu660ra_transition_factor[1];  // LSB/(deg/s) → hal_imu ×DEG2RAD → rad/s
    return true;
}
static void read_raw(i16 g[3], i16 a[3]) {
    imu660ra_get_gyro(); imu660ra_get_acc();
    g[0]=imu660ra_gyro_x; g[1]=imu660ra_gyro_y; g[2]=imu660ra_gyro_z;
    a[0]=imu660ra_acc_x;  a[1]=imu660ra_acc_y;  a[2]=imu660ra_acc_z;
}
const ImuDriver imu_660ra_driver = { init, read_raw, NULL };
