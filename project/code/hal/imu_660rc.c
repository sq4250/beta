/**
 * imu_660rc.c — 仅 init + read_raw + quat
 */
#include "imu_660rc.h"
#include "zf_common_headfile.h"
#include <math.h>

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);
    *acc_scale  = 1.0f;
    *gyro_scale = 1.0f;
    return true;
}
static void read_raw(i16 g[3], i16 a[3]) {
    imu660rc_get_gyro(); imu660rc_get_acc();
    g[0]=imu660rc_gyro_x; g[1]=imu660rc_gyro_y; g[2]=imu660rc_gyro_z;
    a[0]=imu660rc_acc_x;  a[1]=imu660rc_acc_y;  a[2]=imu660rc_acc_z;
}
static bool has_quat(void) { return true; }
static void read_quat(f32 q[4]) {
    imu660rc_get_quarternion();
    q[0]=imu660rc_quarternion[0]; q[1]=imu660rc_quarternion[1];
    q[2]=imu660rc_quarternion[2]; q[3]=imu660rc_quarternion[3];
}

const ImuDriver imu_660rc_driver = { init, read_raw, has_quat, read_quat };
