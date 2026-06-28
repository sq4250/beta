/**
 * hal_motor.h — 双后轮电机驱动
 */
#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H
#include "common.h"

typedef struct { i32 left; i32 right; } MotorPwm;

void hal_motor_init(void);
void hal_motor_set(const MotorPwm *pwm);

#endif
