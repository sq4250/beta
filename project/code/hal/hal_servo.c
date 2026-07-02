/**
 * hal_servo.c — 前轮转向舵机 (200Hz PWM)
 */
#include "hal_servo.h"
#include "config.h"
#include "zf_common_headfile.h"
#include "utils.h"

static bool g_init = false;

void hal_servo_init(void) {
    if (g_init) return; g_init = true;
    u32 center_duty = (u32)((f32)(SERVO_CTR_US + SERVO_TRIM_US) * SERVO_DUTY_PER_US);
    pwm_init(SERVO_TCPWM_CH, SERVO_FREQ_HZ, center_duty);
}

void hal_servo_set_delta(f32 delta) {
    f32 srad  = delta * INV_STEERING_RATIO;
    srad      = clamp(srad, -SERVO_RAD_MAX, SERVO_RAD_MAX);
    f32 pulse = (f32)(SERVO_CTR_US + SERVO_TRIM_US) + srad * SERVO_US_PER_RAD;
    u32 duty  = (u32)(pulse * SERVO_DUTY_PER_US);
    pwm_set_duty(SERVO_TCPWM_CH, duty);
}
