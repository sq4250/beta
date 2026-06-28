/**
 * config.h — 阿克曼小车硬件与控制器参数
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

//===================================================小车几何 (与pipeline一致)===================================================
#define WHEELBASE         0.15f       // 轴距 L [m]
#define INV_WHEELBASE     6.6666667f  // 1/WHEELBASE (乘逆元替代除法)
#define TRACK_WIDTH       0.15f       // 后轮距 [m] (阿克曼差速用)
#define ENCODER_SCALE     0.0001f     // 编码器差值→线速度 [m/s per count/s] (上车标定)
//===================================================小车几何===================================================

//===================================================舵机 PWM===================================================
#define SERVO_FREQ_HZ       200         // PWM 频率 = TRACKER_FREQ (1k/5)
#define SERVO_PWM_DUTY_MAX  10000       // zf_driver 占空比满量程
#define SERVO_PERIOD_US     5000u       // = 1e6/SERVO_FREQ_HZ (200Hz周期)
#define SERVO_DUTY_PER_US   2.0f        // = SERVO_PWM_DUTY_MAX/SERVO_PERIOD_US (pulse[μs]→duty)
// 脉宽 → 占空比: duty = pulse_us * SERVO_DUTY_PER_US
#define SERVO_CTR_US        1500
#define SERVO_ANGLE_MAX_DEG  180        // 舵机物理最大角度 (参考, 用于推导 SERVO_RAD_MAX)
#define SERVO_ANGLE_CTR_DEG   90        // 舵机物理中位角度 (参考, 用于推导 SERVO_RAD_MAX)
#define STEERING_RATIO        2.0f      // 前轮/舵机 = 2:1 (舵机转1°→前轮转2°)
#define INV_STEERING_RATIO   0.5f       // = 1/2

#define SERVO_RAD_MAX       1.57079633f   // = π/2 (舵机最大转角 rad, =90°*DEG2RAD)
#define SERVO_US_PER_RAD    636.61977f   // = 1000/(π/2) μs/rad (舵机脉宽-弧度斜率)
#define SERVO_TCPWM_CH       TCPWM_CH00_P06_1
//===================================================舵机 PWM===================================================

//===================================================IMU 标定===================================================
#define IMU_BIAS_TOTAL      400         // 总采样数
#define IMU_BIAS_FAST       200         // 前N样本用递推平均
#define IMU_BIAS_ALPHA      0.005f      // 后续 EMA 系数
#define IMU_ACC_NORM_MIN    0.9f        // 运动拒绝: 加速度下限 [g]
#define IMU_ACC_NORM_MAX    1.1f        // 运动拒绝: 加速度上限 [g]
#define IMU_MOTION_THR      500         // 运动拒绝: 陀螺阈值 [raw]
#define IMU_YAW_ALPHA       0.95f       // 互补滤波: 陀螺权重
#define IMU_QUAT_ALPHA      0.05f       // 互补滤波: 四元数权重
//===================================================IMU 标定===================================================

//===================================================控制器频率===================================================
#define TRACKER_FREQ       200          // LQR+PWM 频率 Hz (1k/5, 同步无相差)
#define CTRL_DT            0.005f       // = 1/200 (5ms)
#define ISR_DT             0.001f       // 1kHz PIT 中断周期
#define INV_ISR_DT         1000.0f      // = 1/ISR_DT (速度计算: 位移*频率)
//===================================================控制器频率===================================================

//===================================================Planner===================================================
#define TOL_XY             0.05f        // 航点到达容差 [m] (与训练一致)
#define MAX_WAYPOINTS      16           // 最大航点数
//===================================================Planner===================================================

//===================================================LQR===================================================
// 增益表由 tools/run_sim.py 离线计算 → core/lqr_gains.h
// MCU 只需查表插值, 无需 wo/alpha/q_hdg/r 运行时参数
#define LQR_V_MIN          0.1f         // 查表最小速度
//===================================================LQR===================================================

//===================================================纵向 LADRC===================================================
// 一阶惯性模型: alpha*v + v_dot = b0 * throttle + f
// LESO: 降阶观测器, 从 v 估计总扰动 f̂
// 控制律: thr = (kp*e_x + kd*(v_ref - v) + a_ref + alpha*v - f̂) * inv_b0
#define LONG_B0            7.0f        // 控制增益默认值 (稳态 v=b0/alpha≈3.5m/s); 可调参数, 逆元在 init 时计算
#define LONG_ALPHA         2.0f        // 速度衰减系数 (时间常数 1/alpha=0.5s)
#define LONG_WO            15.0f       // LESO 观测带宽 [rad/s]
#define LONG_KP            8.0f        // 纵向位置 P 增益
#define LONG_KD            6.0f        // 纵向速度 D 增益
#define LONG_DT            CTRL_DT     // 控制周期
//===================================================纵向 LADRC===================================================

//===================================================3-clamp 范围 (与pipeline一致)===================================================
#define A_LONG_MAX          4.0f        // 最大纵向加速度 [m/s²]
#define OMEGA_DELTA_MAX    14.0f        // 最大前轮转角角速度 [rad/s]
#define V_MAX               3.5f        // 最大速度 [m/s]
#define DELTA_MAX           0.4636f     // 最大前轮转角 [rad] (≈26.5°)
//===================================================3-clamp 范围===================================================

#endif
