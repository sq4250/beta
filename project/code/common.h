/**
 * common.h - 共享类型定义与系统配置
 *
 * 本文件及 project/code/ 目录下所有代码为用户自主编写，
 * 非逐飞开源库的一部分，版权归用户所有。
 */

#ifndef COMMON_H
#define COMMON_H

#include "zf_common_typedef.h"

//===================================================类型别名===================================================
// 简洁类型命名，禁止使用 C 原生类型
//
// | 原生类型           | 项目类型 |
// |-------------------|---------|
// | unsigned char     | u8      |
// | unsigned short    | u16     |
// | unsigned int      | u32     |
// | unsigned long long| u64     |
// | signed char       | i8      |
// | signed short      | i16     |
// | signed int        | i32     |
// | signed long long  | i64     |
// | float             | f32     |
// | double            | f64     |
//===================================================
typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned int        u32;
typedef unsigned long long  u64;

typedef signed char         i8;
typedef signed short        i16;
typedef signed int          i32;
typedef signed long long    i64;

typedef float               f32;
typedef double              f64;
//===================================================类型别名===================================================

#define M_PI_F   3.14159265f
#define DEG2RAD  0.017453292f   // = π/180

//===================================================管道数据协议===================================================
// IMU 采样数据 (imu_read → car_estimate_update)
typedef struct {
    f32 gyro[3];             // 陀螺三轴 [rad/s] (已去 bias)
    f32 quat[4];             // 四元数 [w,x,y,z]
    bool has_quat;           // 四元数是否有效
} ImuData;

// 执行器指令 (跟踪层 → 执行层)
typedef struct {
    f32 servo_delta;         // 舵机前轮转角 [rad]
    f32 motor_l;             // 左电机油门 [-1, 1]
    f32 motor_r;             // 右电机油门 [-1, 1]
} ActuatorCmd;
//===================================================管道数据协议===================================================

#endif
