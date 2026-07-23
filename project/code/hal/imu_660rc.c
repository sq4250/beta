/**
 * imu_660rc.c — IMU660RC 驱动包装
 *
 * IMU660RC_QUAT_MODE  1: 四元数模式 (默认)  INT2 中断驱动, 120Hz 异步更新
 *                     0: 原始传感器模式      同步 SPI 读取 gyro/acc
 */
#include "imu_660rc.h"
#include "zf_common_headfile.h"

#define IMU660RC_QUAT_MODE  1

//===================================================================================================================
// 四元数模式: INT2 (P18_5) 上升沿 → gpio_18_exti_isr (cm4_isr.c) → imu660rc_callback → 更新全局缓存
// read_raw / read_quat 只从缓存拷贝, 零 SPI 开销
//===================================================================================================================
#if IMU660RC_QUAT_MODE

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (imu660rc_init(IMU660RC_QUARTERNION_480HZ)) return false;
    *acc_scale  = imu660rc_transition_factor[0];  // LSB/g
    *gyro_scale = imu660rc_transition_factor[1];  // LSB/(deg/s)
    return true;
}

static void read_raw(i16 g[3], i16 a[3]) {
    // imu660rc_get_gyro/acc 在四元数模式下为空操作, 数据由 INT2 ISR 更新
    g[0]=imu660rc_gyro_x; g[1]=imu660rc_gyro_y; g[2]=imu660rc_gyro_z;
    a[0]=imu660rc_acc_x;  a[1]=imu660rc_acc_y;  a[2]=imu660rc_acc_z;
}

static void read_quat(f32 q[4]) {
    q[0]=imu660rc_quarternion[0]; q[1]=imu660rc_quarternion[1];
    q[2]=imu660rc_quarternion[2]; q[3]=imu660rc_quarternion[3];
}

const imu_driver_t imu_660rc_driver = { init, read_raw, read_quat };

//===================================================================================================================
// 原始传感器模式: 同步调用 imu660rc_get_gyro/acc 直接读寄存器
// 不支持四元数, hal_imu_has_quat 返回 false
//===================================================================================================================
#else

static bool init(f32 *acc_scale, f32 *gyro_scale) {
    if (imu660rc_init(IMU660RC_QUARTERNION_DISABLE)) return false;
    *acc_scale  = imu660rc_transition_factor[0];  // LSB/g
    *gyro_scale = imu660rc_transition_factor[1];  // LSB/(deg/s)
    return true;
}

static void read_raw(i16 g[3], i16 a[3]) {
    imu660rc_get_gyro(); imu660rc_get_acc();
    g[0]=imu660rc_gyro_x; g[1]=imu660rc_gyro_y; g[2]=imu660rc_gyro_z;
    a[0]=imu660rc_acc_x;  a[1]=imu660rc_acc_y;  a[2]=imu660rc_acc_z;
}

const imu_driver_t imu_660rc_driver = { init, read_raw, NULL };

#endif
