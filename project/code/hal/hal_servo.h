/**
 * hal_servo.h — 前轮转向舵机 PWM 驱动
 *
 * 映射: delta[rad] → srad=delta*INV_RATIO → pulse=CTR+srad*US_PER_RAD → duty*DUTY_PER_US
 */

#ifndef HAL_SERVO_H
#define HAL_SERVO_H

#include "common.h"

//===================================================舵机硬件常数===================================================
#define SERVO_FREQ_HZ       200
#define SERVO_PWM_DUTY_MAX  10000       // zf_driver 满量程
#define SERVO_PERIOD_US     5000u       // = 1e6/SERVO_FREQ_HZ
#define SERVO_DUTY_PER_US   2.0f        // = PWM_DUTY_MAX/PERIOD_US (pulse[μs]→duty)
#define SERVO_CTR_US        1500
#define SERVO_TRIM_US       -35         // 舵机中位微调 [μs], 正值左转, 负值右转
#define SERVO_RAD_MAX       1.57079633f // = π/2 (舵机最大转角 rad)
#define SERVO_US_PER_RAD    636.61977f  // = (MAX_US-CTR_US)/SERVO_RAD_MAX
#define SERVO_TCPWM_CH      TCPWM_CH25_P23_4
//===================================================舵机硬件常数===================================================

void hal_servo_init(void);
void hal_servo_set_delta(f32 delta);

#endif
