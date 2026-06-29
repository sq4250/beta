/**
 * hal_motor.h — 双后轮电机驱动 (油门接口)
 */
#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H
#include "common.h"

void hal_motor_init(void);

// 左右油门 [-1, 1], HAL 层负责归一化到 PWM
void hal_motor_set_thr(f32 thr_l, f32 thr_r);

#endif
