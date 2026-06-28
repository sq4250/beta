/**
 * imu_660ra.c — 仅 init + read_raw
 */
#include "imu_660ra.h"
#include "zf_common_headfile.h"
#include <math.h>

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    imu660ra_init();
    *acc_scale  = 1.0f;   // 因子由 hal_imu 管理, 这里返回1
    *gyro_scale = 1.0f;
    return true;
}
static void read_raw(i16 g[3], i16 a[3]) {
    imu660ra_get_gyro(); imu660ra_get_acc();
    g[0]=imu660ra_gyro_x; g[1]=imu660ra_gyro_y; g[2]=imu660ra_gyro_z;
    a[0]=imu660ra_acc_x;  a[1]=imu660ra_acc_y;  a[2]=imu660ra_acc_z;
}
static bool has_quat(void) { return false; }
static void read_quat(f32 q[4]) { (void)q; }

const ImuDriver imu_660ra_driver = { init, read_raw, has_quat, read_quat };
